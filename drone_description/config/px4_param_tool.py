#!/usr/bin/env python3
"""Read or apply a PX4 .parm file on a running PX4 over MAVLink.

Usage:
  python3 px4_param_tool.py config/px4_sitl_docking.parm            # read only
  python3 px4_param_tool.py config/px4_sitl_docking.parm --apply    # write

PX4's SITL GCS instance listens on UDP 18570 and, while no ground station is
attached, sends to 127.0.0.1:14550. So:

  * no GCS connected -> default --url udpin:127.0.0.1:14550 works;
  * a GCS (e.g. QGroundControl) is connected -> PX4 sends to that GCS instead,
    so use QGC's parameter editor / MAVLink console, or reconnect PX4 first.

Both `param set` (MAVLink PARAM_SET) and QGC persist the values, so this only
has to be done once per build/px4_sitl_default/rootfs tree.
"""
import argparse
import time

from pymavlink import mavutil


def parse_parm(path):
    params = {}
    with open(path, encoding="utf-8") as handle:
        for line in handle:
            line = line.split("#", 1)[0].strip()
            if not line:
                continue
            name, _, value = line.partition(" ")
            if name and value:
                params[name.strip()] = value.strip()
    return params


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("parm")
    parser.add_argument("--apply", action="store_true")
    parser.add_argument(
        "--url", default="udpin:127.0.0.1:14550",
        help="MAVLink endpoint; PX4 sends here when no GCS is attached "
             "(alternatively udpout:127.0.0.1:18570 to PX4's listener)")
    args = parser.parse_args()

    wanted = parse_parm(args.parm)
    master = mavutil.mavlink_connection(args.url, source_system=255,
                                        source_component=0)
    try:
        master.wait_heartbeat(timeout=20)
    except Exception:
        pass
    if not master.target_system:
        master.target_system = 1
        master.target_component = 1
    print(f"target {master.target_system}/{master.target_component}")
    time.sleep(1)

    def pump(seconds):
        end = time.time() + seconds
        value = None
        while time.time() < end:
            master.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            msg = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.3)
            if msg is not None:
                value = msg
        return value

    results = {}
    for name, text in wanted.items():
        master.mav.param_request_read_send(
            master.target_system, master.target_component,
            name.encode("ascii"), -1)
        msg = None
        deadline = time.time() + 3
        while time.time() < deadline and msg is None:
            msg = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
            if msg is not None and msg.param_id.strip("\x00") != name:
                msg = None
        if msg is None:
            print(f"  {name:<18} <no response>")
            continue
        current = msg.param_value
        ptype = msg.param_type
        if args.apply:
            master.mav.param_set_send(
                master.target_system, master.target_component,
                name.encode("ascii"), float(text), ptype)
            pump(0.4)
        results[name] = (current, text, ptype)

    print(f"\n{'param':<18} {'current':>10} {'wanted':>10} {'type':>6}")
    for name, (current, text, ptype) in results.items():
        flag = ""
        if args.apply:
            flag = "  <- set" if abs(current - float(text)) < 1e-3 else "  (changed)"
        print(f"{name:<18} {current:>10.4f} {float(text):>10.4f} {ptype:>6}{flag}")
    if not args.apply:
        print("\n(read-only run; pass --apply to write these values)")


if __name__ == "__main__":
    main()
