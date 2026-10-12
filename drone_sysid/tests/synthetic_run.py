"""Generate reproducible SYNTHETIC data, never connect to a vehicle."""
import argparse
import csv
import json
import sys
from pathlib import Path

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import numpy as np
from scipy.signal import lfilter
from scipy.integrate import cumulative_trapezoid
from sysid_core import Config, Excitation


def generate(path):
    path=Path(path);path.mkdir(parents=True,exist_ok=False)
    c=Config();e=Excitation(c);dt=1/c.rate
    t=np.arange(0,e.total,dt)
    phases,commands=zip(*(e.value(float(x)) for x in t))
    u=np.array(commands)
    gain,tau,delay=.92,.42,.12
    n=int(round(delay/dt));a=np.exp(-dt/tau)
    y=lfilter([0,gain*(1-a)],[1,-a],np.r_[np.zeros(n),u[:-n]])
    position=cumulative_trapezoid(y,t,initial=0)
    y+=np.random.default_rng(17).normal(0,.004,len(y))
    with (path/'commands.csv').open('w',newline='') as f:
        w=csv.writer(f);w.writerow(['host_mono_s','elapsed_s','phase','vx_cmd','vy_cmd','vz_cmd'])
        w.writerows(zip(1000+t,t,phases,u,np.zeros(len(t)),np.zeros(len(t))))
    with (path/'telemetry.csv').open('w',newline='') as f:
        w=csv.writer(f);w.writerow(['host_mono_s','fc_boot_s','x','y','z','vx','vy','vz'])
        w.writerows(zip(1000+t+.004,50+t,position,np.zeros(len(t)),np.full(len(t),-5),
                       y,np.zeros(len(t)),np.zeros(len(t))))
    samples=[dict(host_mid_s=1000+tt,fc_s=50+tt,rtt_s=.002,offset_s=-950.) for tt in range(2,108)]
    (path/'metadata.json').write_text(json.dumps({'axis':'x','outcome':'complete',
        'synthetic':True,'config':vars(c),'timesync_samples':samples,
        'ground_truth':{'gain':gain,'tau_s':tau,'delay_s':delay}},indent=2))
    print(path)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--out',required=True,type=Path)
    generate(p.parse_args().out)
