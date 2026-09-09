"""Standalone robot configuration, ROS status and recording HTTP application."""
import asyncio
from pathlib import Path
import tempfile

from aiohttp import web, WSMsgType
from mcap.exceptions import McapError

from .adapter_api import register_adapter_routes
from .runtime import PlatformRuntime
from .settings_workspace import register_settings_routes
from .datasets import register_dataset_routes
from .capture_api import register_capture_routes
from .robot_launcher import RobotLauncher, register_launcher_routes


def create_app(store, *, run_robot=False, bringup=None, initial_robot=None):
    runtime=PlatformRuntime(store.load(),store.path.parent)
    mutation_lock=asyncio.Lock()
    dataset_lock=asyncio.Lock()
    runtime.dataset_operations=0

    @web.middleware
    async def guard(request,handler):
        try:
            if request.method in {'POST','PUT','DELETE'}:
                origin=request.headers.get('Origin')
                if origin and origin!=f'{request.scheme}://{request.host}':
                    raise web.HTTPForbidden(text='请从本服务网页提交修改')
                if request.path=='/api/capture/sample':
                    return await handler(request)
                is_dataset = request.path.startswith('/api/datasets/') or request.path.startswith('/api/recordings/')
                async with dataset_lock if is_dataset else mutation_lock:
                    if is_dataset:
                        runtime.dataset_operations+=1
                    try:
                        return await handler(request)
                    finally:
                        if is_dataset:
                            runtime.dataset_operations-=1
            return await handler(request)
        except (ValueError,KeyError,TypeError,OSError,McapError,EOFError) as error:
            raise web.HTTPBadRequest(text=str(error)) from error

    app=web.Application(client_max_size=100*1024*1024,middlewares=[guard])
    app['runtime']=runtime
    register_settings_routes(app,store,runtime)
    client=register_adapter_routes(app,store,runtime)
    runtime.launcher=RobotLauncher(runtime,client,enabled=run_robot,bringup=bringup,initial_robot=initial_robot)
    register_launcher_routes(app,runtime.launcher)
    register_dataset_routes(app,runtime)
    register_capture_routes(app,runtime)
    # In a colcon symlink-install the installed static files are individual
    # symlinks outside that directory. Resolve the module to its source root,
    # keeping aiohttp's protection against following arbitrary symlinks intact.
    static=Path(__file__).resolve().parent/'static'

    async def index(_request):
        return web.FileResponse(static/'index.html')

    async def status(_request):
        return web.json_response(runtime.status())

    async def config_get(_request):
        return web.json_response(store.load())

    async def profiles(_request):
        return web.json_response({'active':'','profiles':[], 'standard_topics':{
            'joint_state':'/hc_teleop/joint_states','joint_command':'/hc_teleop/joint_cmd'}})

    async def recordings(_request):
        files=await asyncio.to_thread(runtime.recorder.list_recordings)
        total=sum(x['size_bytes'] for x in files)
        return web.json_response({'files':files,'directory':str(runtime.recorder.directory),
            'total_count':len(files),'total_size_bytes':total,'total_size_human':f'{total/1048576:.2f} MB'})

    async def recording_action(request):
        action=request.match_info['action']
        data=await request.json() if request.can_read_body else {}
        if not isinstance(data,dict):
            raise web.HTTPBadRequest(text='录制参数必须为对象')
        if action=='start':
            return await runtime.start_recording(data)
        if action=='stop':
            if runtime.quality.active_segment and runtime.recorder.is_recording():
                runtime.quality.mark('segment',source='recording_stop')
            result=await asyncio.to_thread(runtime.recorder.stop)
            runtime.quality.persist()
            return web.json_response({'ok':True,'recording':False,'status':result})
        if action=='mark':
            if runtime.capture.session and not runtime.capture.session.closed.is_set():
                return web.json_response(runtime.capture.session.mark(data.get('scope','segment'),data.get('note','')))
            return web.json_response(runtime.quality.mark(data.get('scope'),note=data.get('note','')))
        if action=='precheck':
            return web.json_response(runtime.precheck())
        raise web.HTTPNotFound()

    async def recording_get(request):
        if request.match_info['action']=='status':
            return web.json_response(runtime.recorder.status())
        if request.match_info['action']=='precheck':
            return web.json_response(runtime.precheck())
        raise web.HTTPNotFound()

    async def topics(_request):
        return web.json_response(runtime.ros.status()['discovered_topics'])

    async def file_action(request):
        filename=request.match_info['filename']
        if Path(filename).name!=filename or '\\' in filename:
            raise web.HTTPBadRequest(text='无效文件名')
        if request.method=='DELETE':
            if runtime.player.status()['is_active'] and runtime.player.status()['filename']==filename:
                raise web.HTTPConflict(text='请停止回放后再删除文件')
            await asyncio.to_thread(runtime.recorder.delete_recording,filename)
            for suffix in ('.mcap.edit.json','.mcap.quality.json','.mcap.session.json'):
                (runtime.recorder.directory/filename).with_suffix(suffix).unlink(missing_ok=True)
            return web.json_response({'ok':True})
        path=runtime.recorder.directory/filename
        if not path.is_file() or path.is_symlink():
            raise web.HTTPNotFound(text='文件不存在')
        if runtime.recorder.is_recording() and path==runtime.recorder.path:
            raise web.HTTPConflict(text='请停止录制后再下载完整 MCAP')
        return web.FileResponse(path,headers={'Content-Disposition':'attachment; filename="recording.mcap"'})

    async def batch_delete(request):
        data=await request.json()
        deleted,errors=[],[]
        for filename in data.get('filenames',[]):
            try:
                await asyncio.to_thread(runtime.recorder.delete_recording,filename)
                deleted.append(filename)
            except (ValueError,OSError) as error:
                errors.append({'filename':filename,'error':str(error)})
        return web.json_response({'ok':not errors,'deleted':deleted,'errors':errors})

    async def upload_mcap(request):
        from mcap.reader import make_reader
        reader=await request.multipart()
        directory=runtime.recorder.directory
        directory.mkdir(parents=True,exist_ok=True)
        async for part in reader:
            if part.name not in {'file','archive'} or not part.filename:
                continue
            name=Path(part.filename).name
            if name!=part.filename or not name.endswith('.mcap'):
                raise web.HTTPBadRequest(text='请选择 MCAP 文件')
            destination=directory/name
            if destination.exists():
                raise web.HTTPConflict(text='同名文件已存在')
            with tempfile.TemporaryDirectory(dir=directory,prefix='.upload-') as temporary:
                path=Path(temporary)/name
                total=0
                with path.open('wb') as stream:
                    while chunk:=await part.read_chunk():
                        total+=len(chunk)
                        if total>100*1024*1024:
                            raise web.HTTPRequestEntityTooLarge(max_size=100*1024*1024,actual_size=total)
                        stream.write(chunk)
                with path.open('rb') as stream:
                    make_reader(stream).get_summary()
                path.rename(destination)
            return web.json_response({'ok':True,'filename':name})
        raise web.HTTPBadRequest(text='请选择 MCAP 文件')

    async def websocket(request):
        ws=web.WebSocketResponse(heartbeat=25)
        await ws.prepare(request)
        runtime.websockets.add(ws)
        try:
            async for message in ws:
                if message.type==WSMsgType.ERROR:
                    break
        finally:
            runtime.websockets.discard(ws)
        return ws

    async def startup(_app):
        await runtime.start()

    async def shutdown(_app):
        await runtime.launcher.stop()
        await asyncio.gather(*(ws.close() for ws in tuple(runtime.websockets)), return_exceptions=True)
        await runtime.stop()

    app.router.add_get('/',index)
    app.router.add_get('/dashboard/',index)
    app.router.add_static('/static/',static)
    app.router.add_get('/api/status',status)
    app.router.add_get('/health',status)
    app.router.add_get('/api/config',config_get)
    app.router.add_get('/api/robot-profiles',profiles)
    app.router.add_get('/api/ros/topics',topics)
    app.router.add_get('/api/recordings',recordings)
    app.router.add_get('/api/recording/{action}',recording_get)
    app.router.add_post('/api/recording/{action}',recording_action)
    app.router.add_get('/api/recordings/{filename}/download',file_action)
    app.router.add_delete('/api/recordings/{filename}',file_action)
    app.router.add_post('/api/recordings/batch-delete',batch_delete)
    app.router.add_post('/api/recordings/upload',upload_mcap)
    app.router.add_get('/ws',websocket)
    app.on_startup.append(startup)
    app.on_shutdown.append(shutdown)
    return app
