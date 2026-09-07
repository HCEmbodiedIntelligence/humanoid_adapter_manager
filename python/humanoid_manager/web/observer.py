"""Read-only ROS observation; it is independent of HC and motion execution."""
import copy
import json
import threading
import time
from collections import deque

from .protocol import envelope


class RosObserver:
    def __init__(self, enabled, domain_id, subscriptions=(), on_event=None, node_name='humanoid_manager_observer', button_topic='/hc_teleop_recv/buttons'):
        self.enabled, self.domain_id = enabled, domain_id
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.thread = None
        self.subscriptions=copy.deepcopy(subscriptions)
        self.on_event=on_event
        self.node_name=node_name
        self.button_topic=button_topic
        self.events = {}
        self.graph = {'state': 'starting' if enabled else 'disabled', 'error':None,
                      'discovered_nodes':[], 'discovered_topics':[], 'domain_id':domain_id,
                      'graph_age':100, 'graph_updated':0, 'messages':0, 'topic_health':{}}

    def start(self):
        if self.enabled:
            self.thread = threading.Thread(target=self.run, daemon=True, name='configuration-observer')
            self.thread.start()

    def stop(self):
        self.stop_event.set()
        if self.thread:
            self.thread.join(timeout=4)

    def status(self):
        with self.lock:
            state = copy.deepcopy(self.graph)
        state['graph_age'] = time.monotonic()-state.pop('graph_updated',0)
        return state

    def platform_status(self):
        with self.lock:
            events = copy.deepcopy(self.events)
        result = {'enabled':True,'configuration':None,'diagnostics':None,'teleop':None}
        for key, (stamp, payload) in events.items():
            result[key] = {'data':payload, 'age':round(time.monotonic()-stamp,2), 'fresh':time.monotonic()-stamp<4}
        return result

    def run(self):
        context = node = executor = None
        try:
            import rclpy
            from rclpy.context import Context
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.qos import qos_profile_sensor_data
            from diagnostic_msgs.msg import DiagnosticArray
            from std_msgs.msg import String
            from rosidl_runtime_py.convert import message_to_ordereddict
            from rosidl_runtime_py.utilities import get_message
            context = Context()
            rclpy.init(args=[], context=context, domain_id=self.domain_id)
            node = rclpy.create_node(self.node_name, context=context)
            executor = SingleThreadedExecutor(context=context)
            executor.add_node(node)
            def callback(message, key):
                try:
                    payload = json.loads(message.data) if hasattr(message,'data') else message_to_ordereddict(message)
                    if not isinstance(payload,dict):
                        return
                    with self.lock:
                        self.events[key] = (time.monotonic(),payload)
                except (ValueError,TypeError):
                    pass
            subscriptions = [node.create_subscription(kind, topic, lambda msg,key=key:callback(msg,key), qos_profile_sensor_data)
                for kind,topic,key in [(String,'/humanoid/configuration_state','configuration'),
                    (DiagnosticArray,'/diagnostics','diagnostics'),(String,'/hc_teleop_recv/status','teleop')]]
            samples={}
            last_sent={}
            for item in self.subscriptions:
                if not item.get('enabled',True) or 'websocket' not in item.get('outputs',[]):
                    continue
                topic=item['topic']
                samples[topic]=deque(maxlen=100)
                def monitor(msg, item=item):
                    topic=item['topic'];now=time.monotonic();samples[topic].append(now)
                    with self.lock:
                        self.graph['messages']+=1
                    limit=item.get('event_max_hz',10) or 20
                    if topic != self.button_topic and now-last_sent.get(topic,0)<1/limit:
                        return
                    last_sent[topic]=now
                    if self.on_event and item['type'] not in {'sensor_msgs/msg/Image','sensor_msgs/msg/CompressedImage'}:
                        self.on_event(envelope('ros_message','ros2',message_to_ordereddict(msg),topic=topic,msg_type=item['type']))
                subscriptions.append(node.create_subscription(get_message(item['type']),topic,monitor,qos_profile_sensor_data))
            while not self.stop_event.is_set():
                executor.spin_once(timeout_sec=.1)
                now = time.monotonic()
                if now-self.graph['graph_updated']>=1:
                    nodes = [{'name':name,'namespace':namespace} for name,namespace in node.get_node_names_and_namespaces()]
                    with self.lock:
                        health={}
                        for topic,stamps in samples.items():
                            fresh=bool(stamps) and now-stamps[-1]<2
                            hz=(len(stamps)-1)/(stamps[-1]-stamps[0]) if fresh and len(stamps)>1 and stamps[-1]>stamps[0] else 0
                            health[topic]={'topic':topic,'has_data':fresh,'state':'ok' if fresh else 'no_data','hz':round(hz,1),'messages':len(stamps),'min_hz':1,'target_hz':10}
                        self.graph.update(state='running', discovered_nodes=nodes, graph_updated=now,
                            discovered_topics=[{'topic':name,'types':types} for name,types in node.get_topic_names_and_types()],topic_health=health)
        except Exception as error:
            with self.lock:
                self.graph.update(state='error',error=str(error))
        finally:
            if executor:
                executor.shutdown()
            if node:
                node.destroy_node()
            if context and context.ok():
                context.shutdown()
