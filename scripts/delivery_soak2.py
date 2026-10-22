"""Like delivery_soak.py, plus probes on /cmd_vel with different QoS, each in its own
participant-less subscription on one node: depth10 (watcher), depth1 reliable (the driver's
QoS), depth1 best-effort.  Per second: counts for each probe vs the driver's cmd_cb."""
import sys, time, json, collections, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
dur=float(sys.argv[1]); label=sys.argv[2] if len(sys.argv)>2 else ""
rclpy.init(); n=Node("delivery_soak2")
cnt={k:collections.Counter() for k in ("d10","d1rel","d1be")}; diag=[]
def mk(k): 
    return lambda m: cnt[k].__setitem__(int(time.time()),cnt[k][int(time.time())]+1)
n.create_subscription(Twist,"/cmd_vel",mk("d10"),10)
n.create_subscription(Twist,"/cmd_vel",mk("d1rel"),QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,history=HistoryPolicy.KEEP_LAST))
n.create_subscription(Twist,"/cmd_vel",mk("d1be"),QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT,history=HistoryPolicy.KEEP_LAST))
n.create_subscription(String,"/driver_diag",lambda m:diag.append(json.loads(m.data)),10)
t0=time.time()
while time.time()-t0<dur: rclpy.spin_once(n,timeout_sec=0.05)
seen=set(); tot=collections.Counter(); drv=0; lossd=[]; 
for d in diag:
    if d["t"] in seen: continue
    seen.add(d["t"]); s=int(d["t"]); drv+=d["cmd_cb"]
    w={k:cnt[k][s]+cnt[k][s-1] for k in cnt}
    for k in cnt: tot[k]+=w[k]
    if d["cmd_cb"]==0 and w["d10"]>5: lossd.append((s,w))
print("%s soak %.0f s: driver callbacks %d | probes (2 s windows, so /2): depth10 %d, depth1-reliable %d, depth1-best-effort %d"%(label,dur,drv,tot["d10"]//2,tot["d1rel"]//2,tot["d1be"]//2))
print("driver-loss seconds (cmd_cb==0 while depth10 probe saw >5): %d"%len(lossd))
if lossd:
    import statistics as st
    for k in ("d10","d1rel","d1be"): print("  in those seconds %-5s probe: median %.0f msgs per 2 s window, zero in %d of them"%(k,st.median(w[k] for s,w in lossd),sum(1 for s,w in lossd if w[k]==0)))
