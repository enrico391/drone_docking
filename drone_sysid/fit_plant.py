#!/usr/bin/env python3
"""Fit command-to-velocity lag models from log_flight.py output (offline)."""
import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np
from scipy import optimize, signal
from scipy.integrate import cumulative_trapezoid


def read_csv(path):
    with Path(path).open(newline="") as f:
        return list(csv.DictReader(f))


def column(rows, key):
    try:
        a=np.array([float(row[key]) for row in rows])
    except (ValueError,KeyError) as e:
        raise ValueError("Missing/non-numeric column: "+key) from e
    if not np.all(np.isfinite(a)):
        raise ValueError("Nonfinite column: "+key)
    return a


def increasing(a, name):
    if len(a)<2 or np.any(np.diff(a)<=0):
        raise ValueError(name+" clock is nonincreasing (duplicate/reset/out of order)")


def zoh(t, values, query):
    """Previous-value hold, never interpolate a command before it was sent."""
    idx=np.searchsorted(t,query,side="right")-1
    if np.any(idx<0): raise ValueError("Query precedes first command")
    return np.asarray(values)[idx]


def sync_offset(meta, telem_host, telem_fc):
    raw=meta.get("timesync_samples",[])
    good=[x for x in raw if all(math.isfinite(x.get(k,math.nan)) for k in
           ("rtt_s","offset_s","host_mid_s","fc_s")) and 0<=x["rtt_s"]<=.05
           and telem_host[0]<=x["host_mid_s"]<=telem_host[-1]]
    if len(good)<5: raise ValueError("TIMESYNC: need at least 5 usable samples")
    good=sorted(good,key=lambda x:x["rtt_s"])[:max(5,len(good)//2)]
    offsets=np.array([x["offset_s"] for x in good])
    offset=float(np.median(offsets))
    if np.ptp(offsets)>.02: raise ValueError("TIMESYNC offset jitter/drift exceeds 20 ms")
    # Reject remote clocks not on the same epoch as LOCAL_POSITION_NED. A
    # receipt-time comparison only checks the epoch, not exact synchronization.
    discrepancies=[abs(x["fc_s"]-np.interp(x["host_mid_s"],telem_host,telem_fc)) for x in good]
    if np.median(discrepancies)>.1:
        raise ValueError("TIMESYNC clock does not agree with FC boot timestamps")
    return offset,{"samples":len(good),"offset_s":offset,
                   "offset_span_s":float(np.ptp(offsets)),
                   "median_rtt_s":float(np.median([x["rtt_s"] for x in good]))}


def load_run(directory, dt=.02, timebase="auto", max_gap=.15, allow_partial=False):
    p=Path(directory)
    meta=json.loads((p/"metadata.json").read_text())
    if meta.get("axis") not in ("x","y","z"): raise ValueError("Invalid/missing axis metadata")
    if meta.get("outcome")!="complete" and not allow_partial:
        raise ValueError("Run was not complete; inspect events.jsonl or use --allow-partial")
    commands,telem=read_csv(p/"commands.csv"),read_csv(p/"telemetry.csv")
    if len(commands)<100 or len(telem)<100: raise ValueError("Too few command/state samples")
    ct=column(commands,"host_mono_s")
    th,tf=column(telem,"host_mono_s"),column(telem,"fc_boot_s")
    increasing(ct,"command host");increasing(th,"telemetry host");increasing(tf,"FC")
    notices=[]
    sync=None
    if meta.get("config",{}).get("sim_time") and timebase=="host":
        raise ValueError("Simulation runs require FC timebase; use --timebase auto or fc")
    if meta.get("config",{}).get("sim_time") and timebase in ("auto","fc"):
        ct=column(commands,"fc_boot_s")
        increasing(ct,"command FC")
        timebase="fc"
        sync={"mode":"simulation","command_clock":"fc_boot_s"}
        notices.append("Simulation run: durations, frequencies and model parameters use FC seconds.")
    elif meta.get("transport") == "px4_offboard_dds" and timebase in ("auto", "fc"):
        ct=column(commands,"fc_boot_s")
        increasing(ct,"command FC")
        timebase="fc"
        sync={"mode":"px4_dds","command_clock":"fc_boot_s",
              "note":"VehicleLocalPosition.timestamp is FC time; no TIMESYNC required"}
        notices.append("PX4 DDS run: fc_boot_s is VehicleLocalPosition.timestamp/1e6.")
    elif timebase in ("auto","fc"):
        try:
            offset,sync=sync_offset(meta,th,tf)
        except ValueError as e:
            if timebase=="fc": raise
            notices.append(str(e)+"; using receipt-time alignment")
            timebase="host"
        else:
            timebase="fc";ct=ct+offset
    if timebase=="host":
        mt=th
        notices.append("Delay includes telemetry transport and receipt jitter; not physical FC delay.")
    else:
        mt=tf
        notices.append("TIMESYNC estimates clock offset; command delivery latency remains in this plant.")
    if not math.isfinite(dt) or dt<=0 or not math.isfinite(max_gap) or max_gap<=0:
        raise ValueError("dt and max_gap must be finite and positive")
    lo,hi=max(ct[0],mt[0]),min(ct[-1],mt[-1])
    keep=(mt>=lo-max_gap)&(mt<=hi+max_gap)
    mt=mt[keep]
    if len(mt)<100 or hi-lo<15: raise ValueError("Overlap too short")
    measured_dt=float(np.median(np.diff(mt)))
    command_dt=float(np.median(np.diff(ct)))
    if dt<.8*max(measured_dt,command_dt):
        raise ValueError("dt is faster than actual data; set --dt to at least %.4f s"%
                         max(measured_dt,command_dt))
    if np.max(np.diff(mt))>max_gap or np.max(np.diff(ct))>max_gap:
        raise ValueError("Telemetry or command gap exceeds --max-gap; do not interpolate across outages")
    grid=np.arange(lo,hi,dt)
    axis=meta["axis"]
    u=zoh(ct,column(commands,"v"+axis+"_cmd"),grid)
    phases=zoh(ct,np.array([r["phase"] for r in commands]),grid)
    y=np.interp(grid,mt,column(telem,"v"+axis)[keep])
    pos=np.interp(grid,mt,column(telem,axis)[keep])
    all_vel=np.vstack([np.interp(grid,mt,column(telem,"v"+a)[keep]) for a in "xyz"])
    for phase in ("PRE_HOVER","CHIRP","PRBS"):
        minimum=1 if phase=="PRE_HOVER" else 8
        if np.count_nonzero(phases==phase)*dt<minimum:
            raise ValueError("Need at least %g s of %s"%(minimum,phase))
    if np.std(u[phases=="CHIRP"])<.03 or np.std(u[phases=="PRBS"])<.03:
        raise ValueError("Insufficient input excitation")
    return {"t":grid-grid[0],"u":u,"y":y,"position":pos,"all_vel":all_vel,
            "phase":phases,"dt":dt,"metadata":meta,"timebase":timebase,
            "timesync":sync,"notices":notices,"measured_dt_s":measured_dt,
            "command_dt_s":command_dt}


def denominator(taus):
    den=np.array([1.])
    for tau in taus: den=np.polymul(den,[tau,1.])
    return den


def simulate(u, dt, gain, taus, delay_samples, bias=0.):
    delayed=np.zeros_like(u,dtype=float)
    if delay_samples==0: delayed[:]=u
    elif delay_samples<len(u): delayed[delay_samples:]=u[:-delay_samples]
    if len(taus)==1:
        a=math.exp(-dt/taus[0])
        return signal.lfilter([0.,gain*(1-a)],[1.,-a],delayed)+bias
    b,a,_=signal.cont2discrete(([gain],denominator(taus)),dt,method="zoh")
    return signal.lfilter(b.ravel(),a,delayed)+bias


def fit_candidate(u, y, dt, train_mask, max_delay=.8, order=1):
    u,y=np.asarray(u),np.asarray(y)
    train_mask=np.asarray(train_mask,dtype=bool)
    if order not in (1,2) or dt<=0 or max_delay<0 or not math.isfinite(max_delay):
        raise ValueError("Invalid model order, dt or max_delay")
    if np.count_nonzero(train_mask)<100 or np.std(u[train_mask])<.02:
        raise ValueError("Insufficient training excitation")
    best=None
    initial=[1.,.4] if order==1 else [1.,.3,.1]
    initial+=[float(np.clip(np.median(y[:min(100,len(y))]),-.5,.5))]
    bounds=([.05]+[.02]*order+[-1.],[3.]+[5.]*order+[1.])
    # Enumerate sample delays instead of differentiating a discontinuous ZOH.
    for d in range(int(max_delay/dt+1e-6)+1):
        def residual(p):
            return (simulate(u,dt,p[0],p[1:1+order],d,p[-1])-y)[train_mask]
        opt=optimize.least_squares(residual,initial,bounds=bounds,max_nfev=100,
                                   ftol=1e-7,xtol=1e-7,gtol=1e-7)
        mse=float(np.mean(opt.fun**2))
        if opt.success and (best is None or mse<best[0]):
            best=(mse,opt.x.copy(),d)
    if best is None: raise ValueError("Model optimization did not converge")
    mse,parameters,d=best
    return {"order":order,"gain":float(parameters[0]),
            "tau_s":sorted(float(x) for x in parameters[1:1+order]),
            "delay_samples":d,"delay_s":d*dt,"bias_m_s":float(parameters[-1]),
            "train_mse":mse,"prediction":simulate(u,dt,parameters[0],
              parameters[1:1+order],d,parameters[-1])}


def metrics(y, prediction, mask):
    yy,ee=y[mask],(y-prediction)[mask]
    scale=np.linalg.norm(yy-np.mean(yy))
    return {"rmse_m_s":float(np.sqrt(np.mean(ee**2))),
            "fit_percent":float(100*(1-np.linalg.norm(ee)/scale)) if scale>1e-8 else None,
            "mean_residual_m_s":float(np.mean(ee))}


def identify(data, max_delay=.8):
    train=np.isin(data["phase"],["PRE_HOVER","CHIRP"])
    valid=data["phase"]=="PRBS"
    u,y,dt=data["u"],data["y"],data["dt"]
    models=[]
    for order in (1,2):
        m=fit_candidate(u,y,dt,train,max_delay,order)
        m["training"]=metrics(y,m["prediction"],train)
        m["validation"]=metrics(y,m["prediction"],valid)
        models.append(m)
    # Prefer the smaller model unless its held-out RMSE improves by >= 5%.
    selected=1 if models[1]["validation"]["rmse_m_s"] < .95*models[0]["validation"]["rmse_m_s"] else 0
    m=models[selected]
    warnings=list(data["notices"])
    score=m["validation"]["fit_percent"]
    accepted=score is not None and score>=60
    if not accepted: warnings.append("Validation fit <60%: reject this model for controller tuning.")
    if max_delay>0 and m["delay_s"]>=max_delay-dt:
        warnings.append("Delay reached search bound; rerun with a larger --max-delay.")
        accepted=False
    if any(t<=.021 or t>=4.99 for t in m["tau_s"]) or m["gain"]<=.051 or m["gain"]>=2.99:
        warnings.append("A model parameter reached its bound; model family may be inadequate.")
        accepted=False
    residual=y-m["prediction"]
    r=residual[valid]-np.mean(residual[valid])
    lag=min(int(.5/dt),len(r)//4)
    autocorr=float(np.dot(r[:-lag],r[lag:])/np.dot(r,r)) if lag and np.dot(r,r)>0 else 0.
    if abs(autocorr)>.3:
        warnings.append("Residuals remain correlated at 0.5 s; check unmodeled dynamics/disturbances.")
        accepted=False
    untested=data["all_vel"][[i for i in range(3) if i!="xyz".index(data["metadata"]["axis"])]]
    cross_ratio=float(np.max(np.std(untested[:,valid],axis=1))/max(np.std(y[valid]),1e-9))
    if cross_ratio>.3:
        warnings.append("Cross-axis velocity is large; inspect coupling and wind before SISO tuning.")
        accepted=False
    # Measured spectrum from the stationary PRBS, not the nonstationary chirp.
    uv,yv=u[valid],y[valid]
    nperseg=min(len(uv)//4,max(128,int(8/dt)))
    freq,puu=signal.welch(uv,fs=1/dt,nperseg=nperseg)
    _,puy=signal.csd(uv,yv,fs=1/dt,nperseg=nperseg)
    _,coh=signal.coherence(uv,yv,fs=1/dt,nperseg=nperseg)
    response=puy/np.maximum(puu,1e-20)
    f0=data["metadata"]["config"]["f0"];f1=data["metadata"]["config"]["f1"]
    usable=(freq>=f0)&(freq<=f1)&(puu>max(puu)*.001)
    median_coherence=float(np.median(coh[usable])) if np.any(usable) else 0.
    if median_coherence<.6:
        warnings.append("Median in-band coherence <0.6; collect a cleaner/longer experiment.")
        accepted=False
    den=denominator(m["tau_s"])
    b,a,_=signal.cont2discrete(([m["gain"]],den),dt,method="zoh")
    clean=[{k:v for k,v in model.items() if k!="prediction"} for model in models]
    report={"axis":data["metadata"]["axis"],"frame":"LOCAL_NED","timebase":data["timebase"],
        "plant":"sent velocity command -> estimated vehicle velocity",
        "selected_order":m["order"],"gain":m["gain"],"tau_s":m["tau_s"],
        "delay_s":m["delay_s"],"delay_resolution_s":dt,"bias_m_s":m["bias_m_s"],
        "velocity_tf":{"numerator":[m["gain"]],"denominator":den.tolist(),"input_delay_s":m["delay_s"]},
        "position_tf":{"numerator":[m["gain"]],"denominator":np.r_[den,0.].tolist(),"input_delay_s":m["delay_s"]},
        "discrete_velocity":{"b":b.ravel().tolist(),"a":a.tolist(),"dt_s":dt,
                             "additional_input_delay_samples":m["delay_samples"]},
        "candidates":clean,"accepted_for_further_review":accepted,
        "residual_autocorr_at_0_5s":autocorr,"cross_axis_std_ratio":cross_ratio,
        "median_in_band_coherence":median_coherence,"timesync":data["timesync"],
        "actual_state_period_s":data["measured_dt_s"],"actual_command_period_s":data["command_dt_s"],
        "warnings":warnings,"assessment":"Chirp-fit; PRBS is held out for model comparison. Confirm on a new flight.",
        "limitations":["Two real poles only: oscillatory plants may require a different model family.",
                        "No confidence intervals; sample-delay search quantizes transport plus response delay.",
                        "Does not include camera/AprilTag latency or docking-relative measurement noise.",
                        "Position model adds an integrator; EKF origin changes or resets invalidate position comparison."]}
    return report,m["prediction"],(freq,response,coh)


def export(data, report, prediction, spectrum, out):
    out=Path(out);out.mkdir(parents=True,exist_ok=False)
    (out/"model.json").write_text(json.dumps(report,indent=2,allow_nan=False)+"\n")
    with (out/"resampled.csv").open("w",newline="") as f:
        w=csv.writer(f);w.writerow(["t_s","phase","u_m_s","v_m_s","predicted_v_m_s","position_m"])
        w.writerows(zip(data["t"],data["phase"],data["u"],data["y"],prediction,data["position"]))
    frequency,response,coherence=spectrum
    with (out/"frequency_response.csv").open("w",newline="") as f:
        w=csv.writer(f);w.writerow(["frequency_hz","gain_abs","phase_deg","coherence"])
        w.writerows(zip(frequency,np.abs(response),np.angle(response,deg=True),coherence))
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(3,2,figsize=(13,10),layout="constrained")
    t=data["t"];valid=data["phase"]=="PRBS"
    axes[0,0].plot(t,data["u"],label="Sent command",lw=1)
    axes[0,0].plot(t,data["y"],label="Measured velocity",lw=1)
    axes[0,0].plot(t,prediction,label="Model",lw=1)
    if np.any(valid): axes[0,0].axvspan(t[valid][0],t[valid][-1],alpha=.08,color="green",label="Held-out PRBS")
    axes[0,0].set(xlabel="Time [s]",ylabel="Velocity [m/s]");axes[0,0].legend(fontsize=8)
    pos=data["position"]-data["position"][0]
    predicted_pos=cumulative_trapezoid(prediction,t,initial=0)
    axes[0,1].plot(t,pos,label="Measured displacement")
    axes[0,1].plot(t,predicted_pos,label="Integrated model")
    axes[0,1].set(xlabel="Time [s]",ylabel="Displacement [m]");axes[0,1].legend(fontsize=8)
    f0=data["metadata"]["config"]["f0"];f1=data["metadata"]["config"]["f1"]
    band=(frequency>=f0)&(frequency<=f1)
    s=2j*np.pi*frequency[band]
    model_response=report["gain"]*np.exp(-s*report["delay_s"])
    for tau in report["tau_s"]: model_response/=1+tau*s
    axes[1,0].semilogx(frequency[band],20*np.log10(np.maximum(np.abs(response[band]),1e-8)),label="PRBS estimate")
    axes[1,0].semilogx(frequency[band],20*np.log10(np.abs(model_response)),label="Model")
    axes[1,0].set(xlabel="Frequency [Hz]",ylabel="Magnitude [dB]");axes[1,0].legend(fontsize=8)
    axes[1,1].semilogx(frequency[band],coherence[band])
    axes[1,1].set(xlabel="Frequency [Hz]",ylabel="Coherence",ylim=(0,1.05))
    axes[2,0].plot(t,data["y"]-prediction,lw=.7)
    axes[2,0].set(xlabel="Time [s]",ylabel="Velocity residual [m/s]")
    for i,axis in enumerate("xyz"):
        axes[2,1].plot(t,data["all_vel"][i],label="v"+axis,lw=.8)
    axes[2,1].set(xlabel="Time [s]",ylabel="Axis velocities [m/s]");axes[2,1].legend(fontsize=8)
    for ax in axes.flat: ax.grid(alpha=.25)
    verdict="REVIEW" if report["accepted_for_further_review"] else "REJECT / INVESTIGATE"
    fig.suptitle("%s-axis identification | %s | %s time"%(report["axis"].upper(),verdict,report["timebase"]),fontsize=14)
    fig.savefig(out/"diagnostics.png",dpi=150);plt.close(fig)
    notes=["Model identification: "+verdict,"",json.dumps(report["velocity_tf"]),
           "Position TF: "+json.dumps(report["position_tf"]),"",*report["warnings"],"",report["assessment"]]
    (out/"report.txt").write_text("\n".join(notes)+"\n")


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("run",type=Path,help="Directory created by log_flight.py")
    p.add_argument("--out",type=Path,default=None,help="New directory; default RUN/model")
    p.add_argument("--dt",type=float,default=.02)
    p.add_argument("--timebase",choices=["auto","fc","host"],default="auto")
    p.add_argument("--max-gap",type=float,default=.15)
    p.add_argument("--max-delay",type=float,default=.8)
    p.add_argument("--allow-partial",action="store_true")
    a=p.parse_args()
    out=a.out if a.out else a.run/"model"
    if out.exists(): p.error("Output exists; choose a new --out")
    if not math.isfinite(a.max_delay) or not 0<=a.max_delay<=3:
        p.error("--max-delay must be between 0 and 3 seconds")
    try:
        data=load_run(a.run,a.dt,a.timebase,a.max_gap,a.allow_partial)
        report,prediction,spectrum=identify(data,a.max_delay)
        export(data,report,prediction,spectrum,out)
    except (ValueError,FileNotFoundError,KeyError) as e:
        p.error(str(e))
    print("Gv(s):",report["velocity_tf"])
    print("Gp(s):",report["position_tf"])
    print("Saved:",out)
    for warning in report["warnings"]: print("NOTE:",warning)
    if not report["accepted_for_further_review"]: raise SystemExit(2)


if __name__=="__main__": main()
