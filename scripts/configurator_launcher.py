#!/usr/bin/python3
"""Start the standalone humanoid configuration and data manager."""
import argparse
import asyncio
import copy
import fcntl
import json
from pathlib import Path
import sys
import signal


def main():
    source=Path(__file__).resolve().parents[1]
    if (source/'python').is_dir():
        sys.path.insert(0,str(source/'python'))
    parser=argparse.ArgumentParser(description=__doc__)
    from humanoid_manager.deployment import DEFAULT_PLUGIN_ROOT
    default_root=DEFAULT_PLUGIN_ROOT
    parser.add_argument('--plugin-root',type=Path)
    parser.add_argument('--state-root',type=Path,default=Path.home()/'.local/share/humanoid-manager')
    parser.add_argument('--host')
    parser.add_argument('--port',type=int)
    parser.add_argument('--domain-id',type=int)
    parser.add_argument('--run-robot',action='store_true',help='同时启动网页中明确选定的机器人；未选择时只启动网页')
    parser.add_argument('--bringup-json',default='{"package":"","launch_file":"","arguments":{}}',help='由整机 launch 提供的底层启动项')
    parser.add_argument('--robot-id')
    parser.add_argument('--start-teleop',choices=('true','false'))
    parser.add_argument('--start-cameras',choices=('true','false'))
    parser.add_argument('--offline',action=argparse.BooleanOptionalAction,default=None,help='Edit and validate without ROS')
    args, _ = parser.parse_known_args()
    try:
        from aiohttp import web
        import mcap
        from humanoid_manager.web.config import ConfigStore
        from humanoid_manager.web.server import create_app
    except ImportError as error:
        parser.error(f'缺少依赖 {error.name}，请安装 requirements-web.txt')
    state_root=args.state_root.expanduser().resolve()
    state_root.mkdir(parents=True,exist_ok=True)
    lease=(state_root/'.web.lock').open('a+')
    try:
        fcntl.flock(lease,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:
        parser.error('该配置目录已有网页服务运行')
    path=state_root/'configurator.yaml'
    store=ConfigStore(path)
    if not path.exists():
        store.save({'server':{'host':args.host or ('0.0.0.0' if args.run_robot else '127.0.0.1'),'port':args.port if args.port is not None else 7876},
            'adapter_manager':{'enabled':True,'cli':str(Path(__file__).resolve().parent/'humanoid_pluginctl.py'),
                'plugin_root':str((args.plugin_root or default_root).expanduser().resolve()),'state_root':str(state_root/'configuration')},
            'ros':{'enabled':not args.offline,'domain_id':args.domain_id if args.domain_id is not None else 14,'node_name':'humanoid_manager_observer',
                'recording':{'directory':str(state_root/'recordings')},'subscriptions':[
                    {'topic':'/hc_teleop/joint_states','type':'sensor_msgs/msg/JointState','enabled':True,'outputs':['record','websocket'],'max_hz':0,'event_max_hz':20},
                    {'topic':'/diagnostics','type':'diagnostic_msgs/msg/DiagnosticArray','enabled':True,'outputs':['record'],'max_hz':0,'event_max_hz':0},
                    {'topic':'/hc_teleop_recv/status','type':'std_msgs/msg/String','enabled':False,'outputs':['record'],'max_hz':0,'event_max_hz':0},
                ]}})
    config=store.load()
    if args.plugin_root and str(args.plugin_root.expanduser().resolve())!=config['adapter_manager']['plugin_root']:
        parser.error('此 state-root 已绑定其他插件目录，请传原 --plugin-root 或另选 --state-root')
    previous=copy.deepcopy(config)
    for section,key,value in [('server','host',args.host),('server','port',args.port),('ros','domain_id',args.domain_id),
                              ('ros','enabled',not args.offline if args.offline is not None else None)]:
        if value is not None:
            config[section][key]=value
    if config!=previous:
        from humanoid_manager.web.settings_workspace import atomic_json, fingerprint
        config=store.save(config)
        draft_path=path.parent/f'.{path.name}.history'/'draft.json'
        if draft_path.exists():
            draft=json.loads(draft_path.read_text())
            if draft.get('config')==previous:
                atomic_json(draft_path,{'config':config,'base':fingerprint(config)})
    print(f'配置文件：{path}',flush=True)
    print(f'插件目录：{config["adapter_manager"]["plugin_root"]}',flush=True)
    print(f'网页：http://{config["server"]["host"]}:{config["server"]["port"]}/dashboard/#robots',flush=True)
    if not args.run_robot:
        web.run_app(create_app(store),host=config['server']['host'],port=config['server']['port'])
        return

    async def serve():
        from humanoid_manager.startup import default_plan, validate_plan
        plan=default_plan('launch_validation')
        plan['bringup']=json.loads(args.bringup_json)
        validate_plan(plan)
        initial={key: value for key,value in (
            ('robot_id',args.robot_id),
            ('start_teleop',args.start_teleop=='true' if args.start_teleop is not None else None),
            ('start_cameras',args.start_cameras=='true' if args.start_cameras is not None else None)) if value is not None}
        app=create_app(store,run_robot=True,bringup=plan['bringup'],initial_robot=initial)
        runner=web.AppRunner(app)
        stopped=asyncio.Event()
        loop=asyncio.get_running_loop()
        for sig in (signal.SIGINT,signal.SIGTERM):
            loop.add_signal_handler(sig,stopped.set)
        try:
            await runner.setup()
            await web.TCPSite(runner,config['server']['host'],config['server']['port']).start()
            # Start hardware only after the HTTP port is successfully bound.
            app['runtime'].launcher.begin_autostart()
            print('统一启动已就绪。关闭浏览器不影响运行；Ctrl+C 停止本入口启动的整套服务。',flush=True)
            await stopped.wait()
        finally:
            await runner.cleanup()
            for sig in (signal.SIGINT,signal.SIGTERM):
                loop.remove_signal_handler(sig)

    asyncio.run(serve())


if __name__=='__main__':
    main()
