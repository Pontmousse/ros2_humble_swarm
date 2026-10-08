"""Cross-check the two on-robot bags on a common wall clock."""
import sqlite3, glob, sys
import numpy as np
from pathlib import Path
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def read(bagdir, topic):
    db = glob.glob(str(bagdir) + "/*.db3")[0]
    con = sqlite3.connect(db)
    r = con.execute("SELECT id,type FROM topics WHERE name=?", (topic,)).fetchone()
    if r is None: con.close(); return []
    tid, tn = r; mt = get_message(tn)
    out = [(t/1e9, deserialize_message(bytes(b), mt)) for t,b in
           con.execute("SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp",(tid,))]
    con.close(); return out

B2, B6 = sys.argv[1], sys.argv[2]

# --- true publish rate of the comms node, from cumulative counters ---
print("=== comms node health (from 1 Hz cumulative counters, immune to recorder loss) ===")
for name, bag in (("RM2",B2),("RM6",B6)):
    diag = read(bag, f"/{name}/comms/diagnostics")
    tx=[]; rx=[]; loss=[]
    for t,m in diag:
        for s in m.status:
            kv={k.key:k.value for k in s.values}
            if s.name=="swarm_comms/self":
                tx.append((t,int(kv["tx_count"]))); rx.append((t,int(kv["rx_count"])))
            elif "link" in s.name:
                loss.append(float(kv["loss_fraction"]))
    if len(tx)>2:
        dt=tx[-1][0]-tx[0][0]
        print(f"{name}: tx {(tx[-1][1]-tx[0][1])/dt:5.2f} Hz (nominal 10)   "
              f"rx {(rx[-1][1]-rx[0][1])/dt:5.2f} Hz   "
              f"loss_fraction p50 {np.median(loss):.3f} max {max(loss):.3f}  n_diag={len(diag)}")

# --- do the two robots agree about RM2's position at the same wall clock? ---
print("\n=== RM2 self-pose vs RM6's received view of RM2, same wall clock ===")
p2 = read(B2, "/RM2/pose")
t6 = [(t,n) for t,m in read(B6,"/RM6/comms/neighbours") for n in m.neighbours
      if n.robot_id==2 and n.valid]
a=np.array([t for t,_ in t6]); X=np.array([(n.x,n.y) for _,n in t6])
errs=[]
for t,p in p2:
    j=int(np.argmin(np.abs(a-t)))
    if abs(a[j]-t)>0.05: continue
    errs.append(np.hypot(p.x-X[j][0], p.y-X[j][1]))
errs=np.array(errs)
print(f"paired samples {len(errs)}   position error p50 {np.median(errs):.3f}  "
      f"p95 {np.percentile(errs,95):.3f}  max {errs.max():.3f} m")

# --- where in wall clock did the close approach happen, vs RM2's bag end ---
w6 = read(B6, "/RM6/avoidance_wrench")
nz = [t for t,m in w6 if np.hypot(m.force.x,m.force.y) > 1e-9]
end2 = max(t for t,_ in read(B2,"/RM2/avoidance_wrench"))
import datetime
f=lambda n: datetime.datetime.fromtimestamp(n).strftime('%H:%M:%S.%f')[:-3]
print(f"\nRM6 first non-zero avoidance force : {f(nz[0])}")
print(f"RM6 last  non-zero avoidance force : {f(nz[-1])}")
print(f"RM2 bag last message               : {f(end2)}")
print(f"=> RM2 stopped recording {nz[0]-end2:+.2f} s relative to the force onset")
