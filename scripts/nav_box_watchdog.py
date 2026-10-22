"""Run ON THE ROBOT: boxwatch.py <seconds> <limit_m> <out.json>.  Safety watcher for obstacle tests: cancels the Nav2 goal through the server if any
lethal local-costmap cell comes within LIMIT m of the robot centre.  Costmap is in the
odom frame, so the robot pose is taken in the costmap's own frame."""
import sys, time, math, json, asyncio, numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener
import websockets
LIMIT=float(sys.argv[2]); dur=float(sys.argv[1]); out=sys.argv[3]
rclpy.init(); n=Node("boxwatch"); lc=[]
n.create_subscription(OccupancyGrid,"/local_costmap/costmap",lambda m:lc.append(m),QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL))
buf=Buffer(); TransformListener(buf,n)
async def cancel():
    async with websockets.connect("ws://localhost:8081",max_size=None,ping_interval=None) as ws:
        await ws.send(json.dumps({"type":"cancel_nav"})); await asyncio.sleep(0.5)
t0=time.time(); mn=9.0; fired=False; seen=-1; rows=[]
while time.time()-t0<dur:
    rclpy.spin_once(n,timeout_sec=0.05)
    if not lc or len(lc)==seen: continue
    seen=len(lc); m=lc[-1]
    try: tf=buf.lookup_transform(m.header.frame_id,"base_link",rclpy.time.Time(),timeout=rclpy.duration.Duration(seconds=0.2))
    except Exception: continue
    px,py=tf.transform.translation.x,tf.transform.translation.y
    a=np.array(m.data,dtype=np.int16).reshape(m.info.height,m.info.width); res=m.info.resolution
    ys,xs=np.nonzero(a>=99)
    if not len(xs): continue
    d=float(np.hypot(m.info.origin.position.x+(xs+.5)*res-px,m.info.origin.position.y+(ys+.5)*res-py).min())
    mn=min(mn,d); rows.append((round(time.time()-t0,1),round(d,2)))
    if d<LIMIT and not fired:
        fired=True; asyncio.run(cancel()); print("SAFETY CANCEL at t=%.1f nearest lethal %.2f m"%(time.time()-t0,d),flush=True)
open(out,"w").write(json.dumps({"min_nearest_lethal_m":round(mn,3),"fired":fired,"limit":LIMIT,"samples":rows[-400:]}))
print("watcher done: min nearest lethal %.2f m, fired=%s"%(mn,fired),flush=True)
