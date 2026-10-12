#!/usr/bin/env python3
"""One-shot, terminal-started ArduCopter velocity identification. Read README.md."""
import argparse
import csv
import json
import math
import os
import select
import signal
import sys
import time
from dataclasses import asdict
from pathlib import Path

from sysid_core import Config, State, Policy, Excitation, VELOCITY_MASK


class Recorder:
    def __init__(self, link, config, axis, out, modes, clock=time.monotonic):
        self.link, self.c, self.axis = link, config, axis
        self.clock = clock
        self.modes = modes
        self.mode_names = {v: k for k, v in modes.items()}
        self.state, self.policy = State(), Policy(config)
        self.excitation = Excitation(config)
        self.out = Path(out)
        self.out.mkdir(parents=True, exist_ok=False)
        self.files, self.writers = {}, {}
        headers = {
            "commands": ["host_mono_s", "elapsed_s", "phase", "vx_cmd", "vy_cmd", "vz_cmd",
                         "fc_boot_s"],
            "telemetry": ["host_mono_s", "fc_boot_s", "x", "y", "z", "vx", "vy", "vz",
                          "mode", "armed", "relative_alt_m", "ekf_ok"],
            "attitude": ["host_mono_s", "fc_boot_s", "roll", "pitch", "yaw",
                         "rollspeed", "pitchspeed", "yawspeed"],
            "timesync": ["host_mid_s", "fc_s", "rtt_s", "offset_s"],
        }
        for name, header in headers.items():
            f = (self.out/(name+".csv")).open("w", newline="", buffering=1)
            self.files[name] = f
            self.writers[name] = csv.writer(f)
            self.writers[name].writerow(header)
        self.events = (self.out/"events.jsonl").open("w", buffering=1)
        self.next_send = None
        self.last_phase = None
        self.handoff_sent = False
        self.last_position_ms = None
        self.last_attitude_ms = None
        self.last_altitude_ms = None
        self.pending_sync = {}
        self.sync_samples = []
        self.last_command_fc = None
        self.clock_abort_reason = None
        self.exit_requested = False
        self.exit_t = None
        self.exit_mode = None
        self.last_flush = -math.inf
        self.meta = {"format_version": 1, "axis": axis, "frame": "LOCAL_NED",
                     "config": asdict(config), "velocity_mask": VELOCITY_MASK,
                     "system": link.target_system, "component": link.target_component,
                     "created_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ",time.gmtime()),
                     "start_host_mono_s": None, "outcome": "waiting"}
        self.save_meta()

    def save_meta(self):
        temp = self.out/"metadata.tmp"
        temp.write_text(json.dumps(self.meta, indent=2)+"\n")
        temp.replace(self.out/"metadata.json")

    def event(self, now, name, **fields):
        self.events.write(json.dumps({"host_mono_s":now,"event":name,**fields})+"\n")

    def clock_abort(self, reason):
        if self.c.sim_time:
            self.state.clock_offset_s=math.nan
            # Defer cleanup until all queued mode changes have been observed.
            self.clock_abort_reason=reason
        else:
            self.policy.finish(reason)

    def ingest(self, msg, now):
        if (msg.get_srcSystem() != self.link.target_system or
                msg.get_srcComponent() != self.link.target_component):
            return
        s, kind = self.state, msg.get_type()
        if kind == "HEARTBEAT":
            old = s.mode
            s.mode = self.mode_names.get(msg.custom_mode, "MODE_"+str(msg.custom_mode))
            s.armed = bool(msg.base_mode & 128)
            s.failsafe = msg.system_status in (5, 6, 8)  # critical/emergency/termination
            s.hb_t = now
            self.policy.heartbeat(s, now)
            if old != s.mode:
                self.event(now,"mode",mode=s.mode,armed=s.armed)
            if self.exit_requested and s.mode != "GUIDED":
                self.event(now,"exit_mode_observed",mode=s.mode,
                           requested=self.exit_mode,confirmed=s.mode==self.exit_mode)
                self.exit_requested = False
        elif kind == "LOCAL_POSITION_NED":
            stamp = int(msg.time_boot_ms)
            if self.last_position_ms is not None and stamp <= self.last_position_ms:
                if stamp < self.last_position_ms:
                    self.clock_abort("FC position clock reset/out of order")
                return
            self.last_position_ms = stamp
            s.x,s.y,s.z,s.pos_t = msg.x,msg.y,msg.z,now
            s.pos_boot_s = stamp/1000.
            self.writers["telemetry"].writerow([now,stamp/1000.,msg.x,msg.y,msg.z,
                msg.vx,msg.vy,msg.vz,s.mode,int(s.armed),s.relative_alt,int(s.ekf_ok)])
        elif kind == "ATTITUDE":
            stamp = int(msg.time_boot_ms)
            if self.last_attitude_ms is not None and stamp <= self.last_attitude_ms:
                if stamp < self.last_attitude_ms:
                    self.clock_abort("FC attitude clock reset/out of order")
                return
            self.last_attitude_ms = stamp
            s.roll,s.pitch,s.att_t = msg.roll,msg.pitch,now
            s.att_boot_s = stamp/1000.
            self.writers["attitude"].writerow([now,stamp/1000.,msg.roll,msg.pitch,msg.yaw,
                msg.rollspeed,msg.pitchspeed,msg.yawspeed])
        elif kind == "GLOBAL_POSITION_INT":
            stamp=int(msg.time_boot_ms)
            if self.last_altitude_ms is not None and stamp <= self.last_altitude_ms:
                if stamp < self.last_altitude_ms:
                    self.clock_abort("FC altitude clock reset/out of order")
                return
            self.last_altitude_ms=stamp
            s.relative_alt,s.alt_t = msg.relative_alt/1000.,now
            s.alt_boot_s=stamp/1000.
        elif kind == "EXTENDED_SYS_STATE":
            # LAND may report LANDING: acceptable only with the altitude guard.
            s.in_air,s.air_t = msg.landed_state in (2,4),now
        elif kind == "EKF_STATUS_REPORT":
            required = 1|2|4|16|32  # attitude, both velocities, absolute XY and Z
            forbidden = 128|1024|2048  # constant-position, GPS glitch, accel error
            s.ekf_ok = (msg.flags & required)==required and not (msg.flags & forbidden)
            s.ekf_t = now
        elif kind == "TIMESYNC" and msg.tc1 != 0:
            sent = self.pending_sync.pop(int(msg.ts1),None)
            if sent is not None:
                mid = (sent+now)/2
                fc = msg.tc1/1e9
                rtt = now-sent
                sample = {"host_mid_s":mid,"fc_s":fc,"rtt_s":rtt,"offset_s":fc-mid}
                if 0 <= rtt <= .05:
                    self.sync_samples.append(sample)
                    recent=self.sync_samples[-3:] if self.c.sim_time else self.sync_samples[-10:]
                    if len(recent)>=3:
                        if self.c.sim_time:
                            # Short local fit tracks SITL speed changes. Wall-clock
                            # receipt gates remain active, including during pause.
                            dh=[x["host_mid_s"]-mid for x in recent]
                            df=[x["fc_s"]-fc for x in recent]
                            mh,mf=sum(dh)/3,sum(df)/3
                            denom=sum((x-mh)**2 for x in dh)
                            advancing=all(b["fc_s"]>a["fc_s"] and
                                          b["host_mid_s"]>a["host_mid_s"]
                                          for a,b in zip(recent,recent[1:]))
                            rate=sum((x-mh)*(y-mf) for x,y in zip(dh,df))/denom if denom>0 else 0.
                            residuals=[y-rate*x for x,y in zip(dh,df)]
                            recent_enough=mid-recent[0]["host_mid_s"]<=.5
                            if advancing and recent_enough and rate>0 and max(residuals)-min(residuals)<=.02:
                                s.clock_rate=rate
                                s.clock_offset_s=fc-rate*mid
                                s.sync_t=now
                            else:
                                s.clock_offset_s=math.nan
                                if not advancing and self.policy.stage in ("RUN","HANDOFF","READY"):
                                    self.clock_abort("FC simulation clock stalled/reset")
                        else:
                            offsets=sorted(x["offset_s"] for x in recent)
                            if offsets[-1]-offsets[0] <= .02:
                                s.clock_offset_s=offsets[len(offsets)//2]
                                s.sync_t=now
                            else:
                                s.clock_offset_s=math.nan
                self.writers["timesync"].writerow([mid,fc,rtt,fc-mid])
        elif kind in ("COMMAND_ACK","STATUSTEXT","AUTOPILOT_VERSION","SYS_STATUS"):
            self.event(now,kind,fields=msg.to_dict())

    def send_velocity(self, now, phase, u):
        # Recheck ownership at the last possible point before an I/O mutation.
        now=max(now,self.clock())
        if not self.policy.can_send(self.state,now):
            if self.policy.stage=="RUN":
                self.shutdown(self.policy.health_reason(self.state,now) or "mode override",now)
            return
        elapsed=self.policy.elapsed(self.state,now)
        phase,u=self.excitation.value(elapsed)
        if phase=="COMPLETE":
            self.shutdown("complete",now)
            return
        fc_now=self.state.fc_now(now)
        if self.c.sim_time and self.last_command_fc is not None and fc_now<=self.last_command_fc:
            if fc_now<self.last_command_fc-.02:
                self.shutdown("FC command clock moved backwards",now)
            return
        v = [0.,0.,0.]
        v["xyz".index(self.axis)] = u
        self.link.mav.set_position_target_local_ned_send(
            0,self.link.target_system,self.link.target_component,1,VELOCITY_MASK,
            0,0,0,*v,0,0,0,0,0)
        self.writers["commands"].writerow(
            [now,elapsed,phase,*v,fc_now])
        self.last_command_fc=fc_now

    def request_mode(self, mode, now):
        self.link.mav.set_mode_send(self.link.target_system,1,self.modes[mode])
        self.event(now,"mode_requested",mode=mode)

    def command(self, text, now):
        """Process a terminal command after draining the MAVLink receive queue."""
        now=max(now,self.clock())
        text=text.strip().lower()
        if text == "quit":
            self.shutdown("operator stopped script",now)
            return "Stopping"
        if self.policy.stage == "DONE":
            return "Experiment already stopped"
        if text == "start":
            why=self.policy.start(self.state,now)
            if why is None:
                self.event(now,"terminal_start")
            return why or "Start accepted; monitoring FC mode and safety bounds"
        if text == "guided":
            why=self.policy.request_guided(self.state,now)
            if why is None:
                self.request_mode("GUIDED",now)
                self.handoff_sent = True
            return why or "GUIDED requested once; waiting for FC confirmation. Type start after confirmation"
        return "Commands: guided (request mode), start (begin only in GUIDED), quit"

    def step(self, now):
        now=max(now,self.clock())
        if self.clock_abort_reason is not None:
            reason,self.clock_abort_reason=self.clock_abort_reason,None
            if self.policy.stage!="DONE":
                self.shutdown(reason,now)
            return
        self.policy.tick(self.state,now)
        if self.policy.stage == "HANDOFF" and not self.handoff_sent:
            self.request_mode("GUIDED",now)
            self.handoff_sent = True
        if self.policy.stage == "RUN":
            if not self.policy.can_send(self.state,now):
                self.shutdown(self.policy.health_reason(self.state,now) or "mode override",now)
                return
            if self.next_send is None:
                self.next_send = now
                self.meta["start_host_mono_s"] = self.policy.start_t
                self.meta["start_fc_boot_s"] = self.policy.start_fc_t
                self.meta["origin_ned_m"] = self.policy.origin
                self.meta["outcome"] = "running"
                self.save_meta()
                self.event(now,"start",axis=self.axis)
            phase,u = self.excitation.value(self.policy.elapsed(self.state,now))
            if phase == "COMPLETE":
                self.shutdown("complete",now)
                return
            if phase != self.last_phase:
                self.event(now,"phase",phase=phase)
                self.last_phase = phase
                print(phase,flush=True)
            if now >= self.next_send:
                if now-self.next_send > .2:
                    self.shutdown("setpoint scheduler overrun",now)
                    return
                self.send_velocity(now,phase,u)
                # Skip missed slots; never burst stale commands.
                self.next_send += (math.floor((now-self.next_send)*self.c.rate)+1)/self.c.rate
        if now-self.last_flush >= 1:
            for f in self.files.values(): f.flush()
            self.last_flush = now

    def shutdown(self, reason, now):
        now=max(now,self.clock())
        # A pilot override is terminal: no zero setpoint or mode change after it.
        owns = self.policy.stage == "RUN"
        s = self.state
        if (owns and s.mode == "GUIDED" and s.armed and not s.failsafe
                and now-s.hb_t <= self.c.heartbeat_timeout):
            # LOITER request requires no healthy local sample. Do not emit a stale
            # velocity command on safety abort. FC owns the braking transition.
            self.request_mode("LOITER",now)
            self.exit_requested,self.exit_t,self.exit_mode = True,now,"LOITER"
        self.policy.finish(reason)
        self.meta["outcome"] = reason
        self.event(now,"stop",reason=reason)
        self.save_meta()

    def sync_request(self, now):
        stamp = int(now*1e9)
        self.pending_sync = {k:v for k,v in self.pending_sync.items() if now-v < 2}
        self.pending_sync[stamp] = now
        self.link.mav.timesync_send(0,stamp)

    def close(self):
        self.meta["outcome"] = self.policy.reason
        self.meta["timesync_samples"] = self.sync_samples
        self.save_meta()
        for f in self.files.values(): f.close()
        self.events.close()


def parser():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--port",default="udpin:0.0.0.0:14551")
    p.add_argument("--baud",type=int,default=921600)
    p.add_argument("--system",type=int,default=1,help="Target autopilot system ID")
    p.add_argument("--component",type=int,default=1)
    p.add_argument("--source-system",type=int,default=245)
    p.add_argument("--axis",choices=["x","y","z"],default="x")
    p.add_argument("--sim-time",action="store_true",
                   help="SITL only: track the FC simulation clock; retain freshness and flight safety checks")
    p.add_argument("--trigger-mode",choices=["TERMINAL","GUIDED","LAND"],default="TERMINAL",
                   help="Default TERMINAL: type guided or start; legacy mode triggers require explicit opt-in")
    p.add_argument("--amp",type=float,default=None)
    p.add_argument("--rate",type=float,default=50.)
    p.add_argument("--chirp",type=float,default=50.)
    p.add_argument("--prbs",type=float,default=45.)
    p.add_argument("--hover",type=float,default=5.)
    p.add_argument("--f0",type=float,default=.1)
    p.add_argument("--f1",type=float,default=2.)
    p.add_argument("--switch-period",type=float,default=.2)
    p.add_argument("--radius",type=float,default=15.)
    p.add_argument("--min-alt",type=float,default=3.,help="Above home, not EKF origin or terrain")
    p.add_argument("--max-alt",type=float,default=10.)
    p.add_argument("--out",type=Path,required=True,help="New output directory")
    return p


def main():
    p=parser();a=p.parse_args()
    try:
        c=Config(trigger_mode=a.trigger_mode, sim_time=a.sim_time, amplitude=a.amp if a.amp is not None else
                 (.2 if a.axis=="z" else .3),rate=a.rate,chirp_s=a.chirp,prbs_s=a.prbs,
                 hover_s=a.hover,f0=a.f0,f1=a.f1,switch_s=a.switch_period,
                 radius_m=a.radius,min_alt_m=a.min_alt,max_alt_m=a.max_alt)
    except ValueError as e:
        p.error(str(e))
    if a.out.exists(): p.error("Output directory already exists; choose a new --out")
    try:
        from pymavlink import mavutil
    except ImportError:
        p.error("Install pymavlink first; see requirements.txt")
    link=mavutil.mavlink_connection(a.port,baud=a.baud,source_system=a.source_system,
                                  source_component=191,dialect="ardupilotmega")
    print("Waiting for selected ArduCopter autopilot heartbeat...",flush=True)
    deadline=time.monotonic()+20
    first=None
    while time.monotonic()<deadline:
        msg=link.recv_match(type="HEARTBEAT",blocking=True,timeout=.2)
        if msg and msg.get_srcSystem()==a.system and msg.get_srcComponent()==a.component:
            first=msg;break
    if first is None: raise SystemExit("No heartbeat from selected system/component")
    if first.autopilot != mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA or first.type != 2:
        raise SystemExit("This script requires ArduCopter QUADROTOR")
    link.target_system,link.target_component=a.system,a.component
    modes=link.mode_mapping()
    if any(k not in modes for k in ("GUIDED","LOITER","LAND")):
        raise SystemExit("Required modes missing")
    r=Recorder(link,c,a.axis,a.out,modes)
    r.ingest(first,time.monotonic())
    stop=[False]
    for sig in (signal.SIGINT,signal.SIGTERM):
        signal.signal(sig,lambda *_:stop.__setitem__(0,True))
    # Individual rates; actual rates are measured offline and may be lower.
    intervals=[(32,c.rate),(30,c.rate),(33,10),(245,2),(193,2),(1,1)]
    interval_index=0
    next_interval=next_sync=next_hb=time.monotonic()
    last_note=None
    terminal = c.trigger_mode == "TERMINAL"
    stdin_open = terminal and sys.stdin.isatty()
    input_buffer = ""
    if terminal:
        if not stdin_open:
            print("No interactive terminal; cannot start. Exiting without mode commands.",flush=True)
            r.shutdown("interactive terminal unavailable",time.monotonic())
        print("Current FC mode: " + r.state.mode,flush=True)
        print("Terminal control: take off and settle first. In GUIDED type start then Enter; "
              "otherwise type guided then Enter, wait for GUIDED confirmed, then type start. "
              "Type quit to exit. No automatic arming/takeoff.",flush=True)
        if c.sim_time:
            print("Simulation time: schedule and fitting use FC seconds; allow clock synchronization before start.",flush=True)
    else:
        print("Legacy trigger: enter " + c.trigger_mode + " from another mode; one run only.",flush=True)
    try:
        while True:
            # Cap receive work so a saturated link cannot starve the scheduler.
            for _ in range(200):
                msg=link.recv_match(blocking=False)
                if msg is None: break
                r.ingest(msg,time.monotonic())
            else:
                # An unseen override could still be queued. Yield without
                # sending a cleanup mode change from this uncertain state.
                r.policy.finish("receive queue saturation; control yielded")
            now=time.monotonic()
            if stdin_open and r.policy.stage in ("WAIT","READY","HANDOFF","RUN"):
                if select.select([sys.stdin],[],[],0)[0]:
                    chunk=os.read(sys.stdin.fileno(),4096).decode(errors="replace")
                    if not chunk:
                        stdin_open=False
                        r.shutdown("terminal input closed",now)
                    else:
                        for char in chunk:
                            if char in "\r\n":
                                if input_buffer:
                                    print(r.command(input_buffer,now),flush=True)
                                    input_buffer=""
                            elif len(input_buffer)<80:
                                input_buffer+=char
            if stop[0]:
                r.shutdown("operator stopped script",now)
                stop[0]=False
            r.step(now)
            if r.policy.reason!=last_note:
                print(r.policy.reason,flush=True);last_note=r.policy.reason
            if r.policy.stage=="DONE":
                if not r.exit_requested: break
                if now-r.exit_t>c.handoff_timeout:
                    r.event(now,"exit_mode_unconfirmed",mode=r.exit_mode)
                    print("LOITER not confirmed; use RC to take control now.",flush=True)
                    break
            else:
                if interval_index<len(intervals) and now>=next_interval:
                    mid,hz=intervals[interval_index]
                    link.mav.command_long_send(a.system,a.component,511,0,mid,1e6/hz,0,0,0,0,0)
                    interval_index+=1;next_interval=now+.15
                if now>=next_sync:
                    r.sync_request(now);next_sync=now+(.1 if c.sim_time else 1.)
                if now>=next_hb:
                    link.mav.heartbeat_send(18,8,0,0,4)  # onboard controller, invalid autopilot
                    next_hb=now+1
            time.sleep(.002)
    except Exception as e:
        # Process failure may leave the FC holding its last setpoint until its
        # configured Guided timeout; request LOITER only with fresh ownership.
        try: r.shutdown("exception: "+repr(e),time.monotonic())
        finally: raise
    finally:
        r.close()
        link.close()
    print("Saved "+str(a.out),flush=True)


if __name__=="__main__": main()
