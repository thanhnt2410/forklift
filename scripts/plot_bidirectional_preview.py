#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Draw motion direction separately from vehicle/fork heading in a static plan."""
import argparse,json,math,csv
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Rectangle
p=argparse.ArgumentParser();p.add_argument('directory');p.add_argument('--actual-csv');a=p.parse_args();out=Path(a.directory)
actual=list(csv.DictReader(open(a.actual_csv))) if a.actual_csv else []
r=json.loads((out/'proposed_route.json').read_text());segments={s['stage']:s for s in r['segments']}
forward='#168642';reverse='#db7200'
groups=[(['NAV_PICK','ALIGN_PICK','INSERT_PICK'],'1. Đến lấy hàng'),(['RETREAT_PICK','NAV_DROP','ALIGN_DROP','INSERT_DROP'],'2. Mang hàng A → B'),(['RETREAT_DROP','RETURN_HOME'],'3. Thả hàng xong → Home')]
fig,axes=plt.subplots(1,3,figsize=(18,7))
for ax,(names,title) in zip(axes,groups):
    if actual:
        first=next(i for i,r in enumerate(actual) if r['state']==names[0])
        stop_state={'NAV_PICK':'RETREAT_PICK','RETREAT_PICK':'RETREAT_DROP','RETREAT_DROP':'DONE'}[names[0]]
        last=next((i for i in range(first+1,len(actual)) if actual[i]['state']==stop_state),len(actual))
        trace=actual[first:last]
        ax.plot([float(r['robot_x']) for r in trace],[float(r['robot_y']) for r in trace],color='#747474',lw=1.5,ls='--',zorder=4)
    last_sign=None;total=backward=0.;changes=0;heading_marks=[]
    for name in names:
        seg=segments[name];points=seg['poses'];total+=seg['statistics']['length_m'];backward+=seg['statistics']['reverse_m']
        for i,(u,v) in enumerate(zip(points,points[1:])):
            dx,dy=v[0]-u[0],v[1]-u[1];distance=math.hypot(dx,dy)
            if distance<1e-5:continue
            sign=1 if dx*math.cos(u[2])+dy*math.sin(u[2])>0 else -1;color=forward if sign>0 else reverse
            ax.plot([u[0],v[0]],[u[1],v[1]],color=color,lw=3,zorder=3)
            if last_sign is not None and sign!=last_sign:
                ax.scatter([u[0]],[u[1]],s=48,facecolors='white',edgecolors='crimson',linewidths=2,zorder=6);changes+=1
            last_sign=sign
            if i%max(1,len(points)//5)==0:
                ax.annotate('',xy=(u[0]+.075*dx/distance,u[1]+.075*dy/distance),xytext=u[:2],arrowprops=dict(arrowstyle='->',color=color,lw=1.8),zorder=4)
        if seg['kind']=='navigation':
            for index in (len(points)//3,2*len(points)//3):heading_marks.append(points[index])
    for point in heading_marks:
        ax.arrow(point[0],point[1],.14*math.cos(point[2]),.14*math.sin(point[2]),width=.006,head_width=.04,color='#333333',length_includes_head=True,zorder=5)
    start=segments[names[0]]['poses'][0];end=segments[names[-1]]['poses'][-1]
    ax.scatter([start[0]],[start[1]],color='royalblue',s=40,zorder=7);ax.scatter([end[0]],[end[1]],color='black',s=40,zorder=7)
    ax.arrow(end[0],end[1],.18*math.cos(end[2]),.18*math.sin(end[2]),width=.008,head_width=.05,color='#333333',length_includes_head=True,zorder=6)
    for point,label in [(r['lift_at'],'A: lấy/nâng'),(r['release_at'],'B: đặt hàng')]:
        ax.add_patch(Rectangle((.95,point[1]-.13),.14,.26,color='goldenrod',alpha=.5,zorder=1))
        ax.text(.95,point[1]+.17,label,fontsize=9,ha='center')
    ax.scatter([0],[0],s=20,color='royalblue');ax.text(.025,.035,'Home',fontsize=9)
    ax.set_xlim(-1.25,1.2);ax.set_ylim(-.92,.97);ax.set_aspect('equal');ax.grid(alpha=.25);ax.set_xlabel('x (m)');ax.set_ylabel('y (m)')
    ax.set_title(f'{title}\nLùi {backward:.2f}/{total:.2f} m · đổi chiều {changes} lần',fontsize=12)
axes[0].text(-1.15,-.78,'Lùi theo cung để vào trục pallet,\nsau đó mới tiến thẳng.',fontsize=10)
axes[1].text(-1.15,-.78,'Lùi cả chặng vận chuyển theo cung;\ntiến lại khi đã hướng vào pallet B.',fontsize=10)
angle=r['home'][2]*180/math.pi
axes[2].text(-1.15,.65,f'Home cho phép hướng đỗ tự do\n(đề xuất {angle:.0f}°). Có đoạn tiến ngắn\nđể mở cung rồi tiếp tục lùi.',fontsize=10)
handles=[Line2D([0],[0],color=forward,lw=3,label='Tiến'),Line2D([0],[0],color=reverse,lw=3,label='Lùi'),Line2D([0],[0],marker='o',color='crimson',markerfacecolor='white',lw=0,label='Điểm đổi chiều'),Line2D([0],[0],color='#333333',marker='>',label='Mũi tên đen: hướng càng')]
if actual:handles.append(Line2D([0],[0],color='#747474',ls='--',label='Xe trong Gazebo'))
fig.legend(handles=handles,loc='lower center',ncol=4,fontsize=12,bbox_to_anchor=(.5,.04))
fig.suptitle('PATH ĐÃ DUYỆT VÀ CHUYỂN ĐỘNG GAZEBO\nXanh: tiến · Cam: lùi · Nét xám: quỹ đạo xe từ mô phỏng vật lý' if actual else 'PATH ĐỀ XUẤT TRƯỚC MÔ PHỎNG — TIẾN/LÙI TRÊN TOÀN TUYẾN\nNav2 trên bản đồ tĩnh; chưa chạy Gazebo, chưa kiểm chứng động lực học',fontsize=16,y=.97)
fig.tight_layout(rect=(0,.11,1,.88));fig.savefig(out/('route_simulation.png' if actual else 'route_preview.png'),dpi=160)
summary=dict(preview_only=True,gazebo_started=False,runtime_configuration_changed=False,total_length_m=sum(s['statistics']['length_m'] for s in r['segments']),reverse_m=sum(s['statistics']['reverse_m'] for s in r['segments']),home_heading_degrees=angle,stages={s['stage']:s['statistics'] for s in r['segments']})
if not actual:(out/'preview_summary.json').write_text(json.dumps(summary,indent=2)+'\n')
