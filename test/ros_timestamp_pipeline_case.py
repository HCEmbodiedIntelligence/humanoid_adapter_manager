"""Subprocess fixture: official-shaped ROS messages through adapter and recorder."""
import copy
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2, PointField, JointState
from realsense2_camera_msgs.msg import Metadata
from std_msgs.msg import Header

from humanoid_manager.recording.hub import CaptureHub
from humanoid_manager.recording.config import validate
from humanoid_manager.recording.catalog import connection, row_at, edits, save_edits
from humanoid_manager.recording.exporter import export, PointCloudReader

camera_script=Path(__file__).resolve().parents[2]/'humanoid_camera/scripts/timestamp_adapter.py'
spec=importlib.util.spec_from_file_location('camera_adapter',camera_script)
mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
root=Path(tempfile.mkdtemp(prefix='ros-midpoint-case-'))
rclpy.init(args=['--ros-args','-r','__ns:=/front'],domain_id=232)
node=Node('simulated_driver')
adapter=mod.TimestampAdapter()
executor=SingleThreadedExecutor();executor.add_node(node);executor.add_node(adapter)
hub=CaptureHub(root,{'enabled':True,'domain_id':232})
cfg=json.loads((Path(__file__).resolve().parents[1]/'config/alignment-strategy.example.json').read_text())
cfg['dataset']['duration_seconds']=.2
cfg['storage'].update(minimum_free_bytes=0,numeric_flush_seconds=.01,video_segment_seconds=.05)
cfg['sources']['front_cloud']['required']=True
hub.cfg=validate(cfg);hub.prepare_adapters()

def header(t,frame='optical'):
    h=Header(frame_id=frame);h.stamp.sec,h.stamp.nanosec=divmod(t,10**9);return h

pub={}
for key,typ,topic in [('rgb',Image,'/front/camera/color/image_raw'),('depth',Image,'/front/camera/depth/image_rect_raw'),
        ('rgb_meta',Metadata,'/front/camera/color/metadata'),('depth_meta',Metadata,'/front/camera/depth/metadata'),
        ('cloud',PointCloud2,'/front/camera/depth/color/points'),('joints',JointState,'/hc_teleop/joint_states'),('commands',JointState,'/hc_teleop/joint_cmd')]:
    pub[key]=node.create_publisher(typ,topic,qos_profile_sensor_data)
try:
    start=time.monotonic();base=time.time_ns();j=k=0;started=False
    while time.monotonic()-start<1.8:
        elapsed=time.monotonic()-start
        while j<=int(elapsed*100):
            capture=base+j*10_000_000
            message=JointState(header=header(capture,'joint'),name=['joint_1'],position=[j*.01])
            pub['joints'].publish(message);pub['commands'].publish(message);j+=1
        while k<=int(elapsed*30):
            capture=base+round(k*1e9/30)
            driver_stamp=capture+2_000_000
            sensor=1_000_000+round(k*1e6/30)
            for stream,encoding,step,values in [('rgb','rgb8',24,bytes([k%255])*144),('depth','16UC1',16,bytes([1,0])*48)]:
                msg=Image(header=header(driver_stamp,stream+'_optical'),height=6,width=8,encoding=encoding,step=step,data=values)
                raw={'clock_domain':'global_time','frame_timestamp':str(driver_stamp//1_000_000)+'.'+str(driver_stamp%1_000_000).zfill(6),
                    'hw_timestamp':sensor+2000,'sensor_timestamp':sensor,'frame_number':k,
                    'actual_exposure':8000,'gain_level':100} # Deliberately not checked per user instruction.
                pub[stream+'_meta'].publish(Metadata(header=msg.header,json_data=json.dumps(raw)))
                pub[stream].publish(msg)
            pub['cloud'].publish(PointCloud2(header=header(driver_stamp,'depth_optical'),height=1,width=1,
                fields=[PointField(name='x',offset=0,datatype=PointField.FLOAT32,count=1)],point_step=4,row_step=4,data=bytes([1,2,3,4])))
            k+=1
        executor.spin_once(timeout_sec=.001)
        if not started and elapsed>.5 and hub.state()['preflight']['ready']:
            hub.start();started=True
        if started and hub.session.closed.is_set():break
    assert started,hub.state()
    if not hub.session.closed.is_set():hub.stop()
    s=hub.session
    assert s.status()['state']=='completed',s.status()
    with connection(s.path) as db:
        rows=[json.loads(r[0]) for r in db.execute('SELECT body FROM rows ORDER BY k')]
        assert db.execute("SELECT count(*) FROM raw_samples WHERE source='joints'").fetchone()[0]>len(rows)
    good=[r for r in rows if r['valid']]
    assert len(good)>=2,rows
    row=good[0]
    assert row['images']['front']['rgb']['timestamp_semantics']=='exposure_midpoint'
    assert row['images']['front']['rgb']['raw_metadata']['actual_exposure']==8000
    assert row['pointclouds']['front_cloud']['capture_time_ns']==row['images']['front']['depth']['exposure_midpoint_ns']
    assert row['pointclouds']['front_cloud']['compute_complete_time_ns'] is None
    assert row['deadline_ns']<row['deadline_ros_ns']//100
    k=row['target_frame_index']
    value=edits(s.path)
    save_edits(s.path,{'episodes':[{'start_frame':k,'end_frame':k+1,'task':'synthetic ROS pipeline'}],
        'invalid_intervals':[],'notes':''},value['etag'])
    output=root/'exported'
    export(s.path,output)
    metadata,points=PointCloudReader(output).get('front_cloud',0,0)
    assert points==bytes([1,2,3,4])
    assert metadata['capture_time_ns']==row['pointclouds']['front_cloud']['capture_time_ns']
    print(json.dumps({'root':str(root),'rows':len(rows),'valid_rows':len(good),'native_pointcloud_bytes_preserved':True}))
finally:
    hub.stop();hub.close_adapters();executor.shutdown();adapter.destroy_node();node.destroy_node();rclpy.try_shutdown()
