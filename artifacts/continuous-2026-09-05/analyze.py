"""Compare matched floor returns to an independently fitted parked baseline."""
from pathlib import Path
import json
import numpy as np
root=Path(__file__).resolve().parent
w=np.load(root/'step_points.npz')['points']
r=np.hypot(w[:,0],w[:,1]); w=w[(abs(w[:,2])<.18)&(r>.5)&(r<3)]
a=np.c_[w[:,:2],np.ones(len(w))]; b=w[:,2]
coef=np.linalg.lstsq(a,b,rcond=None)[0]
for _ in range(8):
    residual=abs(b-a@coef); mask=residual<np.quantile(residual,.65)
    coef=np.linalg.lstsq(a[mask],b[mask],rcond=None)[0]
norm=np.sqrt(1+coef[0]**2+coef[1]**2)
all_points=np.load(root/'step_points.npz')['points']
all_r=np.hypot(all_points[:,0],all_points[:,1]);all_points=all_points[(all_r>.5)&(all_r<3)]
all_err=abs(all_points[:,2]-np.c_[all_points[:,:2],np.ones(len(all_points))]@coef)/norm
cells=np.floor(all_points[:,:2]/.1).astype(int);key=cells[:,0]*10000+cells[:,1]
u,inv,ct=np.unique(key,return_inverse=True,return_counts=True)
near=np.bincount(inv,weights=(all_err<.05).astype(float))
floor_cells=u[(ct>=20)&(near/ct>.8)]
report={'baseline_floor_z_ax_by_c':coef.tolist(),'baseline_candidates':len(w),'reliable_floor_xy_cells':len(floor_cells)}
for speed in (20,45):
    path=root/('continuous_%d_result.json'%speed)
    if not path.exists():continue
    result=json.loads(path.read_text());data=np.load(root/('continuous_%d_points.npz'%speed))
    d,rigid=data['matched'],data['rigid'];mid=(d+rigid)/2
    residual=lambda p:(p[:,2]-np.c_[p[:,:2],np.ones(len(p))]@coef)/norm
    r=np.hypot(mid[:,0],mid[:,1])
    # Symmetric selection based on pair midpoint; neither method gets its own mask.
    cells=np.floor(mid[:,:2]/.1).astype(int);key=cells[:,0]*10000+cells[:,1]
    mask=(abs(residual(mid))<.25)&(r>.5)&(r<3)&np.isin(key,floor_cells)
    comparison={'selected_pairs':int(mask.sum())}
    for name,points in [('deskew',d),('rigid',rigid)]:
        err=abs(residual(points[mask]));comparison[name]={'residual_m_p50_p90_p95':np.quantile(err,[.5,.9,.95]).tolist(),'fraction_within_5cm':float(np.mean(err<.05))}
    j=data['joints']; t0=result['active_stamp_start'];t1=result['active_stamp_end']
    grid=np.arange(t0,t1,.15);p=np.interp(grid,j[:,0],np.degrees(j[:,1]));v=np.diff(p)/.15
    central=(abs(p[:-1])<30)&(abs(p[1:])<30)
    comparison['central_speed_deg_s_p10_p50_p90']=np.quantile(abs(v[central]),[.1,.5,.9]).tolist()
    comparison['cloud_hz']=result['active_counts'].get('cloud',0)/result['active_seconds']
    comparison['raw_hz']=result['active_counts'].get('raw',0)/result['active_seconds']
    comparison['active_scan_leaks']=result['active_scans']
    comparison['system_busy_percent']=result['cpu']['system_busy_percent']
    comparison['end_latency_ms_p50_p95_max']=result['end_latency_ms_p50_p95_max']
    report[str(speed)]=comparison
(root/'analysis.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))

# Export a compact scientific figure for the 45 deg/s physical run.
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
fig,axes=plt.subplots(1,2,figsize=(10,3.6))
axes[0].plot(grid-grid[0],p,lw=1.5)
axes[0].set(xlabel='Time (s)',ylabel='Measured tilt (degrees)',title='Parked 45°/s continuous sweep',ylim=(-50,50))
for label,points in [('Rigid projection',rigid),('Deskew',d)]:
    errors=np.sort(abs(residual(points[mask])))*100
    ix=np.arange(0,len(errors),max(1,len(errors)//2500))
    axes[1].plot(errors[ix],ix/len(errors)*100,label=label)
axes[1].set(xlabel='Deviation from baseline floor plane (cm)',ylabel='Returns within deviation (%)',title='Same selected floor returns',xlim=(0,15),ylim=(0,100))
axes[1].legend()
for ax in axes:ax.grid(alpha=.2)
fig.tight_layout();fig.savefig(root/'continuous_validation.png',dpi=170)
