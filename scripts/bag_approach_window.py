import sqlite3, glob, datetime
import numpy as np
from pathlib import Path
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

ROOT = Path.home()/"rosbags/onrobot"
def read(bagdir, topic):
    db = glob.glob(str(bagdir)+"/*.db3")[0]
    con = sqlite3.connect(db)
    r = con.execute("SELECT id,type FROM topics WHERE name=?", (topic,)).fetchone()
    tid, tn = r; mt = get_message(tn)
    out=[(t/1e9, deserialize_message(bytes(b), mt)) for t,b in
         con.execute("SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp",(tid,))]
    con.close(); return out

B6 = glob.glob(str(ROOT/"_RM6/rosbags/*"))[0]
RANT, GAIN, FMAX = 0.70, 0.78, 1.0
def want(d):
    if d>=RANT: return 0.0
    return min(GAIN*(1.0/d - 1.0/RANT)/d**2, FMAX)

pose = read(B6,"/RM6/pose")
pt=np.array([t for t,_ in pose]); pxy=np.array([(m.x,m.y) for _,m in pose])
tbl=[(t,n) for t,m in read(B6,"/RM6/comms/neighbours") for n in m.neighbours if n.robot_id==2]
at=np.array([t for t,_ in tbl])
w = read(B6,"/RM6/avoidance_wrench")
f=lambda n: datetime.datetime.fromtimestamp(n).strftime('%H:%M:%S.%f')[:-3]

print("  wall-clock     |F| N   want N   sep m  age ms valid   fx     fy")
for t,m in w:
    if t < 19e8: pass
    mag=np.hypot(m.force.x,m.force.y)
    j=int(np.argmin(np.abs(at-t))); k=int(np.argmin(np.abs(pt-t)))
    n=tbl[j][1]
    sep=np.hypot(pxy[k][0]-n.x, pxy[k][1]-n.y)
    if sep>0.85 and mag<1e-9: continue
    print(f"{f(t)}  {mag:5.3f}   {want(sep):5.3f}  {sep:5.3f}  {n.age*1000:5.0f}  {str(n.valid):5s} {m.force.x:+6.3f} {m.force.y:+6.3f}")
