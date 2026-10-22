"""Delivery soak: per second, messages an independent subscriber got on /cmd_vel vs the
driver's own cmd_cb (from /driver_diag).  A second where the watcher saw traffic but the
driver saw none is a delivery-loss event.  Usage: delivery_soak.py <seconds> [label]"""
import sys, time, json, collections, rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
dur=float(sys.argv[1]); label=sys.argv[2] if len(sys.argv)>2 else ""
rclpy.init(); n=Node("delivery_soak")
watch=collections.Counter(); diag=[]
n.create_subscription(Twist,"/cmd_vel",lambda m:watch.__setitem__(int(time.time()),watch[int(time.time())]+1),10)
n.create_subscription(String,"/driver_diag",lambda m:diag.append(json.loads(m.data)),10)
t0=time.time()
while time.time()-t0<dur: rclpy.spin_once(n,timeout_sec=0.1)
ev=[]; tot_w=tot_d=0; seen=set()
for d in diag:
    key=d["t"]
    if key in seen: continue
    seen.add(key); s=int(d["t"])
    w=watch[s]+watch[s-1]; tot_w+=w; tot_d+=d["cmd_cb"]
    if d["cmd_gap_max_ms"]>500 or (d["cmd_cb"]==0 and w>5): ev.append((s,w,d["cmd_cb"],d["cmd_gap_max_ms"],d["tick_gap_max_ms"],d["wd_trips"]))
print("%s soak %.0f s: %d driver records | watcher msgs %d (2 s windows) | driver callbacks %d"%(label,dur,len(seen),tot_w,tot_d))
print("loss events (driver gap>500 ms, or 0 callbacks while the watcher saw >5): %d"%len(ev))
for s,w,c,g,tg,wd in ev[:20]: print("  second %d: watcher %2d msgs in 2 s window | driver cmd_cb %2d | cmd_gap_max %4d ms | executor tick_gap_max %3d ms | wd_trips %d"%(s%100000,w,c,g,tg,wd))
