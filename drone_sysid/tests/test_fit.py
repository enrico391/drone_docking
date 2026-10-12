import csv
import json
import tempfile
import unittest
from pathlib import Path

import numpy as np
from scipy.signal import lfilter, cont2discrete

from fit_plant import fit_candidate, load_run, zoh, identify


class FitTests(unittest.TestCase):
    def test_recovers_known_plant_on_independent_binary_excitation(self):
        rng=np.random.default_rng(19)
        dt=.05
        u=np.repeat(rng.choice([-0.3,0.3],size=180),5)
        u[:100]=0
        a=np.exp(-dt/.45)
        delayed=np.r_[np.zeros(3),u[:-3]]
        y=lfilter([0, .9*(1-a)],[1,-a],delayed)+rng.normal(0,.002,len(u))
        mask=np.arange(len(u))<600
        m=fit_candidate(u,y,dt,mask,max_delay=.3,order=1)
        self.assertAlmostEqual(m["gain"],.9,delta=.02)
        self.assertAlmostEqual(m["tau_s"][0],.45,delta=.03)
        self.assertEqual(m["delay_samples"],3)
        pred=np.asarray(m["prediction"])
        self.assertLess(np.sqrt(np.mean((pred[~mask]-y[~mask])**2)),.006)

    def test_zoh_never_uses_future_command(self):
        self.assertEqual(zoh(np.array([0.,1.,2.]),np.array([2.,4.,9.]),
                             np.array([.5,1.,1.9])).tolist(),[2.,4.,4.])

    def test_recovers_two_pole_plant(self):
        rng=np.random.default_rng(24)
        dt=.05
        u=np.repeat(rng.choice([-.3,.3],size=160),5);u[:100]=0
        b,a,_=cont2discrete(([1.1],[.5*.12,.5+.12,1.]),dt)
        y=lfilter(b.ravel(),a,np.r_[np.zeros(2),u[:-2]])
        m=fit_candidate(u,y,dt,np.arange(len(u))<550,max_delay=.2,order=2)
        self.assertAlmostEqual(m['gain'],1.1,delta=.01)
        self.assertTrue(np.allclose(m['tau_s'],[.12,.5],atol=.01))
        self.assertEqual(m['delay_samples'],2)

    def test_zero_delay_search_does_not_raise_delay_bound_warning(self):
        from sysid_core import Config, Excitation
        c=Config(rate=20.)
        dt=.05;t=np.arange(0,110,dt)
        phase,u=zip(*(Excitation(c).value(float(x)) for x in t))
        u=np.asarray(u);a=np.exp(-dt/.4)
        y=lfilter([0,.9*(1-a)],[1,-a],u)+np.random.default_rng(3).normal(0,.003,len(t))
        data={'phase':np.array(phase),'u':u,'y':y,'dt':dt,'notices':[],
              'all_vel':np.vstack([y,np.zeros(len(t)),np.zeros(len(t))]),
              'metadata':{'axis':'x','config':vars(c)},'timebase':'fc','timesync':None,
              'measured_dt_s':dt,'command_dt_s':dt}
        report,_,_=identify(data,max_delay=0.)
        self.assertFalse(any('Delay reached' in s for s in report['warnings']))

    def fixture(self, d):
        p=Path(d)
        (p/"metadata.json").write_text(json.dumps({"axis":"x","outcome":"complete",
          "config":{"f0":.1,"f1":2.},"timesync_samples":[]}))
        with (p/"commands.csv").open('w',newline='') as f:
            w=csv.writer(f);w.writerow(['host_mono_s','elapsed_s','phase','vx_cmd','vy_cmd','vz_cmd'])
            for i,t in enumerate(np.arange(0,30,.05)):
                phase='PRE_HOVER' if t<2 else 'CHIRP' if t<15 else 'PRBS'
                w.writerow([100+t,t,phase,.3*np.sin(t),0,0])
        with (p/"telemetry.csv").open('w',newline='') as f:
            w=csv.writer(f);w.writerow(['host_mono_s','fc_boot_s','x','y','z','vx','vy','vz'])
            for t in np.arange(0,30,.05): w.writerow([100+t,10+t,0,0,-5,0,0,0])
        return p

    def test_rejects_fc_clock_reset_even_in_host_timebase(self):
        with tempfile.TemporaryDirectory() as d:
            p=self.fixture(d)
            with (p/'telemetry.csv').open() as f:
                rows=list(csv.reader(f))
            rows[150][1]='0'
            with (p/'telemetry.csv').open('w',newline='') as f: csv.writer(f).writerows(rows)
            with self.assertRaisesRegex(ValueError,'clock'):
                load_run(p,dt=.05,timebase='host')

    def test_rejects_gaps(self):
        with tempfile.TemporaryDirectory() as d:
            p=self.fixture(d)
            with (p/'telemetry.csv').open() as f:
                rows=list(csv.reader(f))
            rows=rows[:150]+rows[160:]
            with (p/'telemetry.csv').open('w',newline='') as f: csv.writer(f).writerows(rows)
            with self.assertRaisesRegex(ValueError,'gap'):
                load_run(p,dt=.05,timebase='host')

    def test_fc_timebase_requires_verified_timesync(self):
        with tempfile.TemporaryDirectory() as d:
            with self.assertRaisesRegex(ValueError,'TIMESYNC'):
                load_run(self.fixture(d),dt=.05,timebase='fc')

    def simulation_fixture(self, d):
        p = self.fixture(d)
        meta = json.loads((p/'metadata.json').read_text())
        meta['config']['sim_time'] = True
        meta['timesync_samples'] = [dict(host_mid_s=100.+i, fc_s=10.+i/2,
                                        rtt_s=.004, offset_s=-90.-i/2) for i in range(60)]
        (p/'metadata.json').write_text(json.dumps(meta))
        with (p/'commands.csv').open() as f:
            rows = list(csv.reader(f))
        rows[0].append('fc_boot_s')
        for row in rows[1:]:
            row[0] = str(100.+2*float(row[1]))
            row.append(str(10.+float(row[1])))
        with (p/'commands.csv').open('w', newline='') as f:
            csv.writer(f).writerows(rows)
        with (p/'telemetry.csv').open() as f:
            rows = list(csv.reader(f))
        for row in rows[1:]: row[0] = str(100.+2*(float(row[1])-10.))
        with (p/'telemetry.csv').open('w', newline='') as f:
            csv.writer(f).writerows(rows)
        return p

    def test_simulation_fitting_uses_fc_command_timestamps(self):
        with tempfile.TemporaryDirectory() as d:
            data = load_run(self.simulation_fixture(d), dt=.05)
            self.assertEqual(data['timebase'], 'fc')
            self.assertAlmostEqual(data['command_dt_s'], .05, places=6)
            self.assertAlmostEqual(data['t'][-1], 29.9, places=6)
            self.assertAlmostEqual(data['u'][100], .3*np.sin(5.), places=6)

    def test_simulation_fitting_rejects_nonincreasing_command_clock(self):
        with tempfile.TemporaryDirectory() as d:
            p = self.simulation_fixture(d)
            with (p/'commands.csv').open() as f: rows = list(csv.reader(f))
            rows[150][-1] = '0'
            with (p/'commands.csv').open('w', newline='') as f: csv.writer(f).writerows(rows)
            with self.assertRaisesRegex(ValueError, 'clock'):
                load_run(p, dt=.05)

    def test_simulation_fitting_rejects_host_timebase(self):
        with tempfile.TemporaryDirectory() as d:
            with self.assertRaisesRegex(ValueError, 'Simulation.*FC timebase'):
                load_run(self.simulation_fixture(d),dt=.1,timebase='host')

    def test_recovers_plant_in_fc_seconds_when_simulation_speed_changes(self):
        with tempfile.TemporaryDirectory() as d:
            p = self.simulation_fixture(d)
            # Ten FC seconds at half speed, then twenty at 1.5x speed.
            def wall_time(t): return 100.+(2*t if t<10. else 20.+(t-10.)/1.5)
            with (p/'commands.csv').open() as f: commands = list(csv.reader(f))
            u = np.array([float(row[3]) for row in commands[1:]])
            a = np.exp(-.05/.45)
            y = lfilter([0., .9*(1-a)],[1.,-a],np.r_[np.zeros(3),u[:-3]])
            for row in commands[1:]: row[0] = str(wall_time(float(row[1])))
            with (p/'commands.csv').open('w',newline='') as f:
                csv.writer(f).writerows(commands)
            with (p/'telemetry.csv').open() as f: telemetry = list(csv.reader(f))
            for row,value in zip(telemetry[1:],y):
                row[0],row[5] = str(wall_time(float(row[1])-10.)),str(value)
            with (p/'telemetry.csv').open('w',newline='') as f:
                csv.writer(f).writerows(telemetry)
            meta = json.loads((p/'metadata.json').read_text())
            meta['timesync_samples'] = [dict(host_mid_s=wall_time(i),fc_s=10.+i,
                                            rtt_s=.004,offset_s=10.+i-wall_time(i))
                                        for i in range(30)]
            (p/'metadata.json').write_text(json.dumps(meta))
            data = load_run(p,dt=.05,timebase='fc')
            model = fit_candidate(data['u'],data['y'],.05,data['t']<20.,
                                  max_delay=.3,order=1)
            self.assertAlmostEqual(model['gain'],.9,delta=.01)
            self.assertAlmostEqual(model['tau_s'][0],.45,delta=.01)
            self.assertAlmostEqual(model['delay_s'],.15,delta=.001)


if __name__=='__main__': unittest.main()
