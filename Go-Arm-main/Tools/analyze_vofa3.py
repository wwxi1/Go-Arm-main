from pathlib import Path
import json
import numpy as np
import pandas as pd

root = Path(__file__).resolve().parents[1]
src = root / '第三次调试vofa+数据.csv'
out = root / 'DOCS' / '第三次波形分析'
out.mkdir(exist_ok=True)
d = pd.read_csv(src, encoding='gbk')
a = d.to_numpy(dtype=float)
t = np.arange(len(a)) * .01
delta = np.max(np.abs(np.diff(a[:, [0,2]], axis=0)), axis=1)
active = np.flatnonzero(delta > 0.00001) + 1
groups = np.split(active, np.flatnonzero(np.diff(active) > 50) + 1)
segments = []
for g in groups:
    if len(g) < 8: continue
    s,e = int(g[0]),int(g[-1])
    segments.append({'start_row':s,'end_row':e,'start_s':s*.01,'end_s':e*.01,
                     'duration_s':(e-s)*.01,'target_end':a[e,[0,2]].tolist(),
                     'max_error_deg':np.max(np.abs(a[s:e+1,[1,3]]-a[s:e+1,[0,2]]),axis=0).__mul__(180/np.pi).tolist(),
                     'max_step_deg':np.max(np.abs(np.diff(a[s:e+1][:,[0,2]],axis=0)),axis=0).__mul__(180/np.pi).tolist()})
result = {'rows':len(a),'columns':list(d.columns),'nonfinite':int(np.sum(~np.isfinite(a))),
          'min':a.min(axis=0).tolist(),'max':a.max(axis=0).tolist(),'segments':segments,
          'time_note':'Nominal 10 ms per CSV row; no recorded timestamp.'}
(out/'metrics.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf8')
print(json.dumps(result,ensure_ascii=False,indent=2))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.signal import welch, detrend
plt.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False
fig,axs=plt.subplots(3,1,figsize=(15,9),sharex=True)
for m in range(2):
    axs[m].plot(t,a[:,2*m]*180/np.pi,label='目标',lw=1)
    axs[m].plot(t,a[:,2*m+1]*180/np.pi,label='反馈',lw=.7,alpha=.8)
    axs[m].set_ylabel(f'电机{m}角度 (°)'); axs[m].legend(loc='upper right')
for m in range(2): axs[2].plot(t,a[:,4+m],lw=.7,label=f'电机{m}')
axs[2].set_ylabel('反馈力矩 (N·m)'); axs[2].legend(loc='upper right')
axs[2].set_xlabel('相对时间 (s)，按每行10 ms估算')
for ax in axs: ax.grid(alpha=.2)
fig.suptitle('第三次 VOFA 数据：目标、反馈和力矩全程')
fig.tight_layout(); fig.savefig(out/'overview.png',dpi=145); plt.close(fig)

for name,lo,hi in [('start_motion',611,619),('small_motion',730,738),('middle_motion',636,643)]:
    z=(t>=lo)&(t<=hi)
    fig,axs=plt.subplots(3,2,figsize=(13,9),sharex=True)
    for m in range(2):
        axs[0,m].plot(t[z],a[z,2*m]*180/np.pi,label='目标',lw=1.4)
        axs[0,m].plot(t[z],a[z,2*m+1]*180/np.pi,label='反馈',lw=.9)
        axs[0,m].set_title(f'电机{m}');axs[0,m].set_ylabel('角度 (°)');axs[0,m].legend()
        axs[1,m].plot(t[z],(a[z,2*m+1]-a[z,2*m])*180/np.pi,color='#ac3795',lw=1)
        axs[1,m].set_ylabel('反馈－目标 (°)')
        axs[2,m].plot(t[z],a[z,4+m],color='#167e70',lw=1)
        axs[2,m].set_ylabel('反馈力矩 (N·m)');axs[2,m].set_xlabel('相对时间 (s)，按10 ms/行估算')
    for ax in axs.flat: ax.grid(alpha=.22)
    fig.suptitle('位置大图容易掩盖抖动：放大跟随误差与力矩')
    fig.tight_layout();fig.savefig(out/(name+'.png'),dpi=140);plt.close(fig)

from scipy.signal import savgol_filter, find_peaks
details=[]
for lo,hi in [(612,615),(625,628),(637,640),(650,653),(656,659),(692.5,695.5),(731,734),(617,618),(736,737)]:
    z=(t>=lo)&(t<hi)
    entry={'window_s':[lo,hi],'motors':[]}
    for m in range(2):
        err=(a[z,2*m+1]-a[z,2*m])*180/np.pi
        tau=a[z,4+m]
        # Separate slowly varying tracking/load trend from oscillation; not raw peak error.
        er=err-savgol_filter(err,81,3)
        tr=tau-savgol_filter(tau,81,3)
        freq,pow=welch(er,fs=100,nperseg=min(256,len(er)))
        band=(freq>=2)&(freq<=25)
        peak=float(freq[band][np.argmax(pow[band])])
        entry['motors'].append({'error_range_deg':[float(err.min()),float(err.max())],
             'osc_error_rms_deg':float(np.std(er)), 'osc_torque_rms_nm':float(np.std(tr)),
             'error_torque_corr':float(np.corrcoef(er,tr)[0,1]),'dominant_hz':peak})
    details.append(entry)
(out/'oscillation_metrics.json').write_text(json.dumps(details,ensure_ascii=False,indent=2),encoding='utf8')
print('OSCILLATION',json.dumps(details,ensure_ascii=False,indent=2))

boundary=[]
for seg in segments:
    s=seg['start_row']
    boundary.append({'start_s':s*.01,
        'target_jump_deg':((a[s,[0,2]]-a[s-1,[0,2]])*180/np.pi).tolist(),
        'previous_tracking_error_deg':((a[s-1,[1,3]]-a[s-1,[0,2]])*180/np.pi).tolist(),
        'new_target_minus_previous_feedback_deg':((a[s,[0,2]]-a[s-1,[1,3]])*180/np.pi).tolist()})
(out/'boundary_metrics.json').write_text(json.dumps(boundary,ensure_ascii=False,indent=2),encoding='utf8')
for lo,hi in [(730.2,731.5),(613.1,614.2)]:
    z=(t>=lo)&(t<hi)
    err=(a[z,1]-a[z,0])*180/np.pi
    pp,_=find_peaks(err,prominence=.1,distance=12)
    print('PEAKS',lo, t[z][pp].tolist(), 'periods',np.diff(t[z][pp]).tolist())
print('BOUNDARY_EXAMPLE',a[73022:73030].tolist())

lo,hi=730.1,731.6
z=(t>=lo)&(t<=hi)
fig,axs=plt.subplots(3,1,figsize=(10,8),sharex=True)
axs[0].plot(t[z],a[z,0]*180/np.pi,label='目标角',lw=2)
axs[0].plot(t[z],a[z,1]*180/np.pi,label='反馈角',lw=1.4)
axs[0].set_ylabel('电机0角度 (°)');axs[0].legend()
axs[0].annotate('新动作起点：目标跳变约0.8°',xy=(730.25,a[73025,0]*180/np.pi),xytext=(730.53,95.5),arrowprops={'arrowstyle':'->'})
axs[1].plot(t[z],(a[z,1]-a[z,0])*180/np.pi,color='#ac3795');axs[1].set_ylabel('反馈－目标 (°)')
axs[2].plot(t[z],a[z,4],color='#167e70');axs[2].set_ylabel('反馈力矩 (N·m)')
for ax in axs:
    ax.axvline(730.25,color='gray',ls='--',alpha=.5);ax.grid(alpha=.2)
axs[2].set_xlabel('CSV相对时间 (s)，按10 ms/行估算')
fig.suptitle('起点跳变后出现衰减振荡；不是每个采样点都跳变')
fig.tight_layout();fig.savefig(out/'boundary_zoom.png',dpi=145);plt.close(fig)
