import asyncio, json, time, threading, sys
from collections import Counter, OrderedDict
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud, PointCloud2, JointState, LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import tf2_ros
import websockets
sys.path.insert(0,"/home/jetson/x3_ws/src")
from sweep_load_bench import CpuSampler
from yahboomcar_bringup.lidar_deskew import rotate

speed=float(sys.argv[1]) if len(sys.argv)>1 else 20.
phase='warmup'; active_start=None; active_end=None; active_scans=0; active_counts=None; active_seconds=0.; active_stamp_start=0.; active_stamp_end=0.; end_latency_ms=[]
rclpy.init()
n = Node('deskew_physical_validation')
buf = tf2_ros.Buffer(); listener = tf2_ros.TransformListener(buf,n)
counts=Counter(); timed=OrderedDict(); pending=[]; joints=[]; positions=[]; worlds=[]; differences=[]; arrivals=[]; rigid_worlds=[]; matched_worlds=[]; cpu_result=None
current_pitch=0.; moving=False; bad_scan=0; cmd_max=0.; rawmeta=[]; dtmeta=[]
def key(m): return m.header.stamp.sec*1000000000+m.header.stamp.nanosec
def on_timed(m):
    counts['timed']+=1; timed[key(m)]=m
    while len(timed)>100: timed.popitem(last=False)
    c={c.name:c.values for c in m.channels}
    dtmeta.append([len(m.points),min(c['acquisition_time']),max(c['acquisition_time']),c['scan_duration'][0]])
def on_joint(m):
    global current_pitch,moving
    if 'lidar_tilt_joint' not in m.name:return
    i=m.name.index('lidar_tilt_joint'); current_pitch=m.position[i]; moving=bool(m.velocity[i]>.5)
    joints.append([key(m)*1e-9,current_pitch,float(moving)])
def on_cloud(m):
    counts['cloud']+=1; pending.append(m); arrivals.append(time.monotonic())
    if phase=='active' and key(m) in timed:
        duration=next(c.values[0] for c in timed[key(m)].channels if c.name=='scan_duration')
        end_latency_ms.append((n.get_clock().now().nanoseconds*1e-9-key(m)*1e-9-duration)*1000)
def on_scan(m):
    global bad_scan,active_scans
    counts['scan']+=1
    if phase=='active':active_scans+=1
    if abs(current_pitch)>.07:bad_scan+=1

def on_raw(m):
    counts['raw']+=1
    if len(rawmeta)<10:rawmeta.append([len(m.ranges),m.time_increment,m.scan_time])
def on_cmd(m):
    global cmd_max
    cmd_max=max(cmd_max,abs(m.linear.x),abs(m.linear.y),abs(m.angular.z))
def process():
    while pending:
        m=pending[0]
        try:t=buf.lookup_transform('base_footprint',m.header.frame_id,Time.from_msg(m.header.stamp))
        except tf2_ros.TransformException:
            if len(pending)>10:pending.pop(0);counts['tf_failed']+=1;continue
            return
        pending.pop(0)
        xyz=np.frombuffer(m.data,dtype='<f4').reshape(-1,3).copy()
        q=t.transform.rotation;v=t.transform.translation
        world=rotate(xyz,np.array([q.x,q.y,q.z,q.w]))+np.array([v.x,v.y,v.z]);worlds.append(world)
        raw=timed.get(key(m))
        if raw:
            pts=np.array([(p.x,p.y,p.z) for p in raw.points]);r=np.linalg.norm(pts,axis=1)
            # Accepted step clouds are tilted; production min range is .35.
            pts=pts[(r>=.35)&(r<8)&np.isfinite(pts).all(axis=1)]
            if pts.shape==xyz.shape:
                differences.extend(np.linalg.norm(xyz-pts,axis=1).tolist())
                rigid_worlds.append(rotate(pts,np.array([q.x,q.y,q.z,q.w]))+np.array([v.x,v.y,v.z]))
                matched_worlds.append(world)
        counts['transformed']+=1

for typ,topic,cb,qos in [(PointCloud,'/lidar/points_timed',on_timed,qos_profile_sensor_data),(PointCloud2,'/pointcloud_raw',on_cloud,10),(JointState,'/lidar_tilt/joint_states',on_joint,10),(LaserScan,'/scan',on_scan,qos_profile_sensor_data),(LaserScan,'/scan_raw',on_raw,qos_profile_sensor_data),(Twist,'/cmd_vel',on_cmd,10),(Odometry,'/odom',lambda m:positions.append([m.pose.pose.position.x,m.pose.pose.position.y]),10)]: n.create_subscription(typ,topic,cb,qos)
n.create_timer(.03,process)
stop=False
def spin():
    while not stop:rclpy.spin_once(n,timeout_sec=.05)
th=threading.Thread(target=spin);th.start()
async def wait(ws,seconds):
    end=time.monotonic()+seconds
    while time.monotonic()<end:
        try:
            m=json.loads(await asyncio.wait_for(ws.recv(),.5))
            if m.get('type')=='3d_scan_status':
                print('sweep_status',m,flush=True)
                if m.get('refused'):raise RuntimeError(m)
        except asyncio.TimeoutError:pass
        if cmd_max>.01:raise RuntimeError('Nonzero chassis command detected')
async def run():
    global phase,active_counts,active_seconds,cpu_result,active_stamp_start,active_stamp_end
    async with websockets.connect('ws://localhost:8081',max_size=None) as ws:
        await ws.send(json.dumps({'type':'toggle_motors','enabled':False}))
        await ws.send(json.dumps({'type':'stop'}))
        await ws.send(json.dumps({'type':'set_sweep_config','mode':'continuous','speed_deg_s':speed}))
        try:
            await wait(ws,5)
            print('preflight',dict(counts),flush=True)
            if counts['timed']<10 or len(joints)<20:raise RuntimeError('Live inputs missing')
            await ws.send(json.dumps({'type':'toggle_3d_scan','enabled':True}))
            await wait(ws,2)
            phase='active'; before=counts.copy(); t_active=time.monotonic(); cpu=CpuSampler();cpu.start();active_stamp_start=n.get_clock().now().nanoseconds*1e-9
            for i in range(3):
                await wait(ws,10);print('progress',10*(i+1),dict(counts),flush=True)
            active_stamp_end=n.get_clock().now().nanoseconds*1e-9;active_seconds=time.monotonic()-t_active; active_counts=dict(counts-before)
            rows,busy,elapsed=cpu.stop()
            cpu_result={'system_busy_percent':busy,'elapsed':elapsed,'processes':[]}
            for pct,pid,name in rows[:12]:
                try:cmd=open('/proc/%d/cmdline'%pid,'rb').read().replace(b'\0',b' ').decode()
                except OSError:cmd=name
                cpu_result['processes'].append({'percent_one_core':pct,'pid':pid,'cmd':cmd})
        finally:
            phase='homing'
            await ws.send(json.dumps({'type':'toggle_3d_scan','enabled':False}))
            await wait(ws,8)
            await ws.send(json.dumps({'type':'set_sweep_config','mode':'step'}))
            await wait(ws,2)
            await ws.send(json.dumps({'type':'stop'}))
            # Leave chassis motors disabled for inspection after the test.
try:asyncio.run(run())
finally:
    stop=True;th.join();n.destroy_node();rclpy.shutdown()
    w=np.concatenate(worlds) if worlds else np.empty((0,3));j=np.array(joints)
    result={'active_stamp_start':active_stamp_start,'active_stamp_end':active_stamp_end,'end_latency_ms_p50_p95_max':np.quantile(end_latency_ms,[.5,.95,1]).tolist() if end_latency_ms else [],'cpu':cpu_result,'speed_deg_s':speed,'active_scans':active_scans,'active_counts':active_counts,'active_seconds':active_seconds,'counts':dict(counts),'scan_while_tilted':bad_scan,'max_cmd':cmd_max,'raw_metadata':rawmeta,'timed_metadata':dtmeta[:10]}
    if len(j):result.update(tilt_deg_minmax=np.degrees([j[:,1].min(),j[:,1].max()]).tolist(),final_tilt_deg=float(np.degrees(j[-1,1])),joint_gap_ms_p50_p95_max=(1000*np.quantile(np.diff(j[:,0]),[.5,.95,1])).tolist())
    if len(positions):result['odom_max_displacement_m']=float(np.linalg.norm(np.array(positions)-positions[0],axis=1).max())
    if len(w):
        result['world_z_quantiles']=np.quantile(w[:,2],[0,.01,.05,.5,.95,.99,1]).tolist()
        floor=w[(np.abs(w[:,2])<.15)&(np.hypot(w[:,0],w[:,1])>.5)&(np.hypot(w[:,0],w[:,1])<3)]
        if len(floor):result['floor_candidate_z_p10_p50_p90']=np.quantile(floor[:,2],[.1,.5,.9]).tolist();result['floor_candidate_count']=len(floor)
    if differences:result['deskew_minus_rigid_m_p50_p95_p99_max']=np.quantile(differences,[.5,.95,.99,1]).tolist()
    np.savez_compressed('/home/jetson/x3_ws/artifacts/continuous-2026-09-05/continuous_%g_points.npz'%speed,points=w,joints=j,rigid=np.concatenate(rigid_worlds) if rigid_worlds else np.empty((0,3)),matched=np.concatenate(matched_worlds) if matched_worlds else np.empty((0,3)))
    open('/home/jetson/x3_ws/artifacts/continuous-2026-09-05/continuous_%g_result.json'%speed,'w').write(json.dumps(result,indent=2))
    print(json.dumps(result,indent=2),flush=True)
