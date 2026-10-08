import sqlite3, glob, sys, datetime
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

def read(bag, topic):
    db = glob.glob(bag+"/*.db3")[0]
    con = sqlite3.connect(db)
    r = con.execute("SELECT id,type FROM topics WHERE name=?", (topic,)).fetchone()
    if r is None: con.close(); return []
    tid, tn = r; mt = get_message(tn)
    out=[(t/1e9, deserialize_message(bytes(b), mt)) for t,b in
         con.execute("SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp",(tid,))]
    con.close(); return out

def series(bag, me):
    pose = read(bag, f"/{me}/pose")
    pt=np.array([t for t,_ in pose]); pxy=np.array([(m.x,m.y) for _,m in pose])
    out=[]
    for t,m in read(bag,f"/{me}/comms/neighbours"):
        for n in m.neighbours:
            if not n.valid: continue
            j=int(np.argmin(np.abs(pt-t)))
            if abs(pt[j]-t)>0.5: continue
            out.append((t, float(np.hypot(pxy[j][0]-n.x, pxy[j][1]-n.y)),
                        pxy[j][0], pxy[j][1], n.x, n.y))
    return out

B2=sys.argv[1]; B6=sys.argv[2]
s2=series(B2,"RM2"); s6=series(B6,"RM6")
a2=np.array([x[0] for x in s2]); a6=np.array([x[0] for x in s6])
w2={t:np.hypot(m.force.x,m.force.y) for t,m in read(B2,"/RM2/avoidance_wrench")}
w6={t:np.hypot(m.force.x,m.force.y) for t,m in read(B6,"/RM6/avoidance_wrench")}
k2=np.array(sorted(w2)); k6=np.array(sorted(w6))
f=lambda n: datetime.datetime.fromtimestamp(n).strftime('%H:%M:%S')

lo=min(a2[0],a6[0]); hi=max(a2[-1],a6[-1])
print("  time      RM2 sees   RM6 sees   |diff|   F(RM2)  F(RM6)    RM2 pos          RM6 pos")
t=lo
while t<=hi:
    r2=r6=None
    if abs(a2-t).min()<0.6:
        i=int(np.argmin(np.abs(a2-t))); r2=s2[i]
    if abs(a6-t).min()<0.6:
        j=int(np.argmin(np.abs(a6-t))); r6=s6[j]
    d2 = f"{r2[1]:7.3f}" if r2 else "    -- "
    d6 = f"{r6[1]:7.3f}" if r6 else "    -- "
    dif = f"{abs(r2[1]-r6[1]):6.3f}" if (r2 and r6) else "   -- "
    fa = f"{w2[k2[np.argmin(np.abs(k2-t))]]:6.3f}" if len(k2) and abs(k2-t).min()<0.6 else "   -- "
    fb = f"{w6[k6[np.argmin(np.abs(k6-t))]]:6.3f}" if len(k6) and abs(k6-t).min()<0.6 else "   -- "
    p2 = f"({r2[2]:+6.3f},{r2[3]:+6.3f})" if r2 else "        --       "
    p6 = f"({r6[2]:+6.3f},{r6[3]:+6.3f})" if r6 else "        --       "
    print(f"{f(t)}  {d2}   {d6}   {dif}  {fa}  {fb}   {p2}  {p6}")
    t+=2.0
