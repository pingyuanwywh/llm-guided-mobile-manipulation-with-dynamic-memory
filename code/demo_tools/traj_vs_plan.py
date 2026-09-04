#!/usr/bin/env python3
"""
traj_vs_plan.py -- 【本机, 需 source ROS】从 rosbag 量两件一直没量过的事 (2026-08-08)。

  ① 车的**真实轨迹** vs move_base **全局路径** 的偏差 —— 只在**导航窗口**内统计。
     (逼近/抓取阶段车由 approach.py 直发 /cmd_vel, 不跟全局路径; 不切窗口会算出 3m 的假偏差)
  ② 真实轨迹离每个罐子最近多少 —— 只统计该罐**还站着**的时段, 且**排除去接它自己**那条腿。
     (罐子被收走后车会开过它原来的位置; 去接目标罐按设计就要进到 0.45m 内)

几何门的告警线 WARN=0.50 一直是照"TEB 大概偏多少"猜的。这是第一次实测。
用法: source /opt/ros/noetic/setup.bash
      /usr/bin/python3 ~/traj_vs_plan.py <bag> <state.yaml> <mission输出日志>
"""
import argparse, math, re, sys
import rosbag, yaml

def seg_pt(a,b,p):
    ax,ay=a; bx,by=b; px,py=p
    dx,dy=bx-ax, by-ay; L2=dx*dx+dy*dy
    if L2==0: return math.hypot(px-ax,py-ay)
    t=max(0.0,min(1.0,((px-ax)*dx+(py-ay)*dy)/L2))
    return math.hypot(px-(ax+t*dx), py-(ay+t*dy))

def poly_min(poly,p): return min(seg_pt(poly[i],poly[i+1],p) for i in range(len(poly)-1))

def pct(v,q): return sorted(v)[int(q*(len(v)-1))]

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("bag"); ap.add_argument("state"); ap.add_argument("log")
    a=ap.parse_args()

    cans={n:(c["x"],c["y"]) for n,c in (yaml.safe_load(open(a.state)) or {}).get("cans",{}).items()}

    # --- 从 mission 日志切出: 每条腿的目标罐 + 该罐"被抓住"的 epoch ---
    grabbed={}      # can -> epoch(抓住那一刻, 之后它不在地上了)
    legstart={}     # can -> epoch(这条腿开始, 即开始朝它开)
    cur=None; last_epoch=None; pending=False
    for ln in open(a.log):
        m=re.search(r'=+\s+(can\d) \(\w+\) -> \w+\s+phase=', ln)
        if m: cur=m.group(1); pending=True; continue
        m=re.search(r'\[INFO\] \[(\d+\.\d+)\]', ln)
        if m:
            last_epoch=float(m.group(1))
            if pending and cur: legstart[cur]=last_epoch; pending=False
        if cur and 'GRAB: success=True' in ln and cur not in grabbed:
            grabbed[cur]=last_epoch
    print("每条腿: " + ", ".join("%s[%.0f→%.0f]"%(k,legstart.get(k,0),grabbed[k])
                                for k in sorted(grabbed,key=lambda x:grabbed[x])))

    traj=[]; plans=[]; goals=[]
    with rosbag.Bag(a.bag) as bag:
        for topic,msg,t in bag.read_messages(topics=["/tf","/move_base/NavfnROS/plan","/move_base/current_goal"]):
            ts=t.to_sec()
            if topic=="/tf":
                for tr in msg.transforms:
                    if tr.header.frame_id=="map" and tr.child_frame_id=="base_footprint":
                        traj.append((ts,tr.transform.translation.x,tr.transform.translation.y))
            elif topic.endswith("NavfnROS/plan"):
                pts=[(p.pose.position.x,p.pose.position.y) for p in msg.poses]
                if len(pts)>1: plans.append((ts,pts))
            else: goals.append((ts,msg.pose.position.x,msg.pose.position.y))

    # --- 导航窗口: 从发目标 到 车进入目标 0.15m 内 ---
    wins=[]
    for gt,gx,gy in goals:
        end=None
        for ts,x,y in traj:
            if ts>=gt and math.hypot(x-gx,y-gy)<0.15: end=ts; break
        if end: wins.append((gt,end))
    print("导航窗口 %d 段, 合计 %.0f 秒 (整趟 %.0f 秒)"
          %(len(wins), sum(e-s for s,e in wins), traj[-1][0]-traj[0][0]))

    def in_nav(ts): return any(s<=ts<=e for s,e in wins)

    # --- ① 偏差, 只在导航窗口内 ---
    devs=[]
    for i,(t0,pts) in enumerate(plans):
        t1=plans[i+1][0] if i+1<len(plans) else 1e18
        for ts,x,y in traj:
            if t0<=ts<t1 and in_nav(ts): devs.append(poly_min(pts,(x,y)))
    print("\n=== ① 真实轨迹偏离全局路径 (导航窗口内 %d 采样) ==="%len(devs))
    if devs:
        print("  中位 %.3f   P90 %.3f   P95 %.3f   P99 %.3f   最大 %.3f m"
              %(pct(devs,.5),pct(devs,.90),pct(devs,.95),pct(devs,.99),max(devs)))

    # --- ② 离罐子最近距离: 只算该罐还站着 + 不是去接它那条腿 ---
    print("\n=== ② 真实轨迹离罐子最近多少 (只算它还站着的时段) ===")
    print("  罐    还站着时最近   判定       (必撞 0.193 / 告警 0.50)")
    worst=(9,None)
    for n in sorted(cans):
        tg=grabbed.get(n); ls=legstart.get(n)
        if tg is None: continue
        # 该罐还站着(ts<tg), 且**排除去接它自己那条腿**(ls<=ts<tg)
        ds=[math.hypot(x-cans[n][0],y-cans[n][1]) for ts,x,y in traj
            if ts<tg and not (ls is not None and ls<=ts)]
        if not ds:
            print("  %-5s (它是第一个收的, 车一开始就停在它跟前, 无\"别的腿\"可比)"%n); continue
        d=min(ds)
        flag="⛔必撞" if d<0.193 else ("⚠️进告警区" if d<0.50 else "✅")
        if d<worst[0]: worst=(d,n)
        print("  %-5s %.3f m       %s"%(n,d,flag))
    print("\n  全程最紧: %s %.3f m"%(worst[1],worst[0]))

if __name__=="__main__": main()
