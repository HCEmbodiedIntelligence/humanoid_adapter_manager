"""Read-only normalized ROS inputs. No SDK, device clock math or camera controls."""
import json
import threading
import time

import numpy as np

from .models import CONTRACT


def header_ns(header):
    return header.stamp.sec*1_000_000_000+header.stamp.nanosec


def decode_image(msg):
    if msg.encoding not in {'rgb8','16UC1','32FC1','mono16'}:
        raise ValueError('Unsupported raw image encoding: '+msg.encoding)
    dtype=np.dtype('u1' if msg.encoding=='rgb8' else ('f4' if msg.encoding=='32FC1' else 'u2'))
    dtype=dtype.newbyteorder('>' if msg.is_bigendian else '<')
    channels=3 if msg.encoding=='rgb8' else 1
    row_bytes=msg.width*channels*dtype.itemsize
    if msg.height<1 or msg.width<1 or msg.step<row_bytes or len(msg.data)!=msg.height*msg.step:
        raise ValueError('ROS image layout/length mismatch')
    shape=(msg.height,msg.width,channels) if channels==3 else (msg.height,msg.width)
    strides=(msg.step,channels*dtype.itemsize,dtype.itemsize) if channels==3 else (msg.step,dtype.itemsize)
    return np.ndarray(shape,dtype=dtype,buffer=msg.data,strides=strides)


def joint_document(ident,source,message,seq,receive_ros,receive_steady,epoch=0):
    capture=header_ns(message.header)
    if capture<=0:
        raise ValueError('JointState.header.stamp is uninitialized')
    payload={}
    for field,rule in source['fields'].items():
        names=rule.get('names')
        values=list(getattr(message,field,[]))
        if names:
            values=[values[message.name.index(name)] for name in names]
        payload[field]=values
    d={'input_contract':CONTRACT,'capture_clock_id':'ros','header_timestamp_ns':capture,
        'source_id':ident,'source_seq':seq,'source_timestamp_ns':capture,'capture_time_ns':capture,
        'receive_time_ns':receive_ros,'receive_clock_id':'ros','receive_steady_ns':receive_steady,
        'clock_id':'ros','clock_epoch':epoch,'ros_clock_epoch':epoch,'clock_model_id':'ros_header_direct',
        'timestamp_quality':'ros_stamped','clock_uncertainty_ns':source.get('timestamp_uncertainty_ns'),
        'clock_evidence':source.get('timestamp_evidence',''),'timestamp_semantics':source.get('timestamp_semantics','unavailable'),
        'known_delay_ns':source.get('known_delay_ns'),'payload':payload,
        'ros_header_timestamp_ns':capture,'ros_header_frame_id':message.header.frame_id,'joint_names':list(message.name)}
    if source['kind']=='action':
        if source.get('timestamp_semantics') not in {'send','effective'}:
            raise ValueError('Action header must represent send or effective time')
        d.update(action_stage='sent',control_mode=source.get('control_mode'))
        d['effective_time_ns' if source['timestamp_semantics']=='effective' else 'send_time_ns']=capture
    return d


class RosSources:
    def __init__(self,hub,ros_config):
        self.hub,self.config=hub,ros_config
        self.stop_event=threading.Event()
        self.thread=threading.Thread(target=self.run,name='normalized-ros-inputs',daemon=True)
        self.error=None
        self.rejected=0

    def start(self):
        self.thread.start()

    def stop(self):
        self.stop_event.set()
        self.thread.join(5)

    def active_session(self):
        s=self.hub.session
        return s if s and not s.closed.is_set() else None

    def run(self):
        context=node=executor=None
        try:
            from collections import OrderedDict
            import rclpy
            from rclpy.context import Context
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.qos import qos_profile_sensor_data
            from sensor_msgs.msg import JointState, Image, CameraInfo, PointCloud2
            from realsense2_camera_msgs.msg import RGBD, Metadata
            from .ros_images import HeaderPairs
            from humanoid_camera.transport import CaptureSubscription
            context=Context()
            rclpy.init(args=[],context=context,domain_id=self.config['domain_id'])
            node=rclpy.create_node('humanoid_normalized_sources',context=context)
            executor=SingleThreadedExecutor(context=context)
            executor.add_node(node)
            seq={};pending=OrderedDict();pending_bytes=0
            capture_subscriptions=[]
            budget=self.hub.cfg['buffers'].get('ros_assembly_bytes',64*1024*1024)
            def report(ident,data=None):
                self.hub.reports[ident]={**(data or {}),'ready':True,'capture_clock_id':'ros',
                    'registered_monotonic_ns':time.monotonic_ns()}
            def rejected(ident,error):
                self.rejected+=1
                session=self.active_session()
                if session:
                    session.counters['ros_contract_rejected:'+ident]+=1
                    session.storage.submit('gap',{'source_id':ident,'reason':'input_contract_rejected','detail':str(error)})
            def offer(ident,source,message,data):
                report(ident,data)
                session=self.active_session()
                if not session:return
                now=node.get_clock().now().nanoseconds;steady=time.monotonic_ns()
                session.observe_ros_clock(now,steady)
                if data.get('source_id')!=ident or data.get('input_contract')!=CONTRACT or header_ns(message.header)!=data.get('capture_time_ns'):
                    raise ValueError('Normalized envelope/header mismatch')
                data.update(receive_time_ns=now,receive_clock_id='ros',receive_steady_ns=steady)
                if source['kind']=='rgbd':
                    for name in ('rgb','depth'):
                        image=getattr(message,name)
                        if header_ns(image.header)!=data[name]['capture_time_ns']:
                            raise ValueError(name+': Header differs from capture time')
                        data[name]['header_timestamp_ns']=header_ns(image.header)
                    session.submit(data,decode_image(message.rgb),decode_image(message.depth))
                else:
                    data['pointcloud_layout']={'width':message.width,'height':message.height,'point_step':message.point_step,
                        'row_step':message.row_step,'is_bigendian':message.is_bigendian,'is_dense':message.is_dense,
                        'fields':[{'name':f.name,'offset':f.offset,'datatype':f.datatype,'count':f.count} for f in message.fields]}
                    session.submit(data,points=message.data)
            def combine(ident,source,kind,message):
                nonlocal pending_bytes
                try:
                    key=(ident,header_ns(message.header))
                    size=len(message.json_data.encode()) if kind=='metadata' else len(message.data) if source['kind']=='pointcloud' else len(message.rgb.data)+len(message.depth.data)
                    if size>budget:
                        raise ValueError('ROS assembly byte budget exceeded')
                    while pending and (pending_bytes+size>budget or (key not in pending and len(pending)>=256)):
                        oldkey,entry=pending.popitem(last=False);pending_bytes-=sum(x[1] for x in entry.values())
                        rejected(oldkey[0],'ROS paired message/metadata not completed before bounded eviction')
                    entry=pending.setdefault(key,{})
                    if kind in entry:return
                    entry[kind]=(message,size);pending_bytes+=size
                    if len(entry)==2:
                        del pending[key];pending_bytes-=sum(x[1] for x in entry.values())
                        d=json.loads(entry['metadata'][0].json_data)
                        offer(ident,source,entry['message'][0],d)
                except (ValueError,KeyError,TypeError) as error:
                    rejected(ident,error)
            sources=self.hub.cfg['sources']
            raw_count=max(1,sum(s['transport']=='ros_images' for s in sources.values()))
            for ident,source in sources.items():
                transport=source['transport']
                seq[ident]=0
                if transport=='ros_joint_state':
                    def joint(message,ident=ident,source=source):
                        session=self.active_session()
                        steady=time.monotonic_ns();now=node.get_clock().now().nanoseconds
                        if session:session.observe_ros_clock(now,steady)
                        try:
                            seq[ident]+=1
                            d=joint_document(ident,source,message,seq[ident],now,steady,session.scheduler.epoch if session else 0)
                            report(ident,{'timestamp_semantics':source.get('timestamp_semantics'),'timestamp_quality':'ros_stamped'})
                            if session:session.submit(d)
                        except (ValueError,KeyError,IndexError,TypeError) as error:rejected(ident,error)
                    node.create_subscription(JointState,source['topic'],joint,qos_profile_sensor_data)
                elif transport in {'ros_rgbd','ros_pointcloud'}:
                    if not source.get('metadata_topic'):
                        raise ValueError(ident+': metadata_topic is required for source lineage')
                    capture_subscriptions.append(CaptureSubscription(node,Metadata,source['metadata_topic'],lambda msg,ident=ident,source=source:combine(ident,source,'metadata',msg),depth=30))
                    capture_subscriptions.append(CaptureSubscription(node,RGBD if transport=='ros_rgbd' else PointCloud2,source['topic'],lambda msg,ident=ident,source=source:combine(ident,source,'message',msg),depth=2 if transport=='ros_pointcloud' else 10))
                elif transport=='ros_images':
                    pairs=HeaderPairs(byte_limit=budget//raw_count//2)
                    for name in ('rgb','depth'):
                        def image(message,ident=ident,source=source,pairs=pairs,name=name):
                            try:
                                pair,expired=pairs.add(name,message,node.get_clock().now().nanoseconds,time.monotonic_ns())
                                for item in expired:rejected(ident,'Unpaired image evicted from bounded input buffer')
                                if pair:
                                    report(ident)
                                    session=self.active_session()
                                    if session:
                                        d=pairs.document(ident,source,pair)
                                        session.submit(d,decode_image(pair['streams']['rgb'][0]),decode_image(pair['streams']['depth'][0]))
                            except (ValueError,KeyError,TypeError) as error:rejected(ident,error)
                        capture_subscriptions.append(CaptureSubscription(node,Image,source[name+'_topic'],image))
                        if source.get(name+'_info_topic'):
                            node.create_subscription(CameraInfo,source[name+'_info_topic'],lambda msg,pairs=pairs,name=name:pairs.info(name,msg),qos_profile_sensor_data)
            next_transport_refresh=time.monotonic()
            while not self.stop_event.is_set():
                executor.spin_once(timeout_sec=.02)
                now=time.monotonic()
                if now>=next_transport_refresh:
                    for subscription in capture_subscriptions:subscription.refresh()
                    next_transport_refresh=now+1.
        except Exception as error:
            self.error=str(error)
            session=self.active_session()
            if session:session.fail('ros_source_failure:'+self.error)
            for ident,source in self.hub.cfg['sources'].items():
                if source['transport'].startswith('ros_'):self.hub.reports[ident]={'ready':False,'error':self.error}
        finally:
            if executor:executor.shutdown(timeout_sec=1)
            if node:node.destroy_node()
            if context:context.try_shutdown()
