"""HTTP contracts for versioned acquisition and session data management."""
import asyncio
import base64
import json
import math
from pathlib import Path
from collections import OrderedDict
import io
import threading

import numpy as np
from aiohttp import web

from ..recording.catalog import session_path, summary, row_at, save_edits, read_json


def register_capture_routes(app,runtime):
    preview_readers=OrderedDict()
    preview_lock=threading.Lock()
    def hub():
        return runtime.capture

    async def state(_request):
        return web.json_response(hub().state())

    async def operation(request):
        data=await request.json() if request.can_read_body else {}
        op=request.match_info['operation']
        if op=='config':
            result=await asyncio.to_thread(hub().save_config,data['config'],data.get('etag'))
        elif op=='prepare':
            result=await asyncio.to_thread(hub().prepare_adapters)
        elif op=='clock':
            result=hub().register_clock(data)
        elif op=='capability':
            result=hub().register_report(data['source_id'],data['report'])
        elif op=='start':
            if runtime.recorder.is_recording() or runtime.player.status()['is_active']:
                raise web.HTTPConflict(text='请先结束当前 ROS 录制或数据回放')
            result=await asyncio.to_thread(hub().start,runtime.recording_metadata())
        elif op=='stop':
            result=await asyncio.to_thread(hub().stop)
        elif op=='mark':
            if not hub().session:
                raise ValueError('当前没有对齐采集')
            result=hub().session.mark(data.get('scope','segment'),data.get('note',''))
        else:
            raise web.HTTPNotFound()
        return web.json_response(result)

    async def sample(request):
        if not hub().session or hub().session.closed.is_set():
            raise web.HTTPConflict(text='未启动采集 session')
        data=await request.json()
        metadata=data['metadata']
        source=hub().cfg['sources'].get(metadata.get('source_id'),{})
        if source.get('transport')!='envelope':
            raise ValueError('仅 envelope 来源接受 HTTP 投递')
        arrays={}
        for key in ('rgb','depth'):
            if key not in data:
                continue
            document=data[key]
            dtype=np.dtype(document['dtype'])
            shape=document['shape']
            if not isinstance(shape,list) or not 2<=len(shape)<=3 or any(type(x) is not int or not 1<=x<=8192 for x in shape) or dtype.kind not in 'uif' or dtype.itemsize>8:
                raise ValueError('图像尺寸或数据类型无效')
            size=math.prod(shape)*dtype.itemsize
            if size>hub().cfg['buffers']['ingress_bytes']:
                raise ValueError('图像超过入口字节预算')
            raw=base64.b64decode(document['base64'],validate=True)
            if len(raw)!=size:
                raise ValueError('图像字节数与 dtype/shape 不匹配')
            arrays[key]=np.frombuffer(raw,dtype).reshape(shape)
        result=hub().session.submit(metadata,arrays.get('rgb'),arrays.get('depth'))
        return web.json_response(result,status=200 if result['accepted'] else 429)

    async def sessions(_request):
        return web.json_response({'sessions':await asyncio.to_thread(hub().sessions)})

    async def detail(request):
        path=session_path(hub().directory,request.match_info['ident'])
        return web.json_response(await asyncio.to_thread(summary,path))

    async def row(request):
        path=session_path(hub().directory,request.match_info['ident'])
        k=int(request.query.get('k','0'))
        if k<0:
            raise ValueError('目标帧不能为负数')
        return web.json_response(await asyncio.to_thread(row_at,path,k,request.query.get('version','online')))

    async def mutate_session(request):
        ident=request.match_info['ident']
        path=session_path(hub().directory,ident)
        data=await request.json()
        operation=request.match_info['operation']
        if operation=='edits':
            result=await asyncio.to_thread(save_edits,path,data['data'],data.get('etag'))
        elif operation in {'export','realign'}:
            result=hub().start_job(operation,ident,data)
        else:
            raise web.HTTPNotFound()
        return web.json_response(result)

    async def file(request):
        root=session_path(hub().directory,request.match_info['ident'])
        relative=request.query.get('path','')
        path=(root/relative).resolve()
        if not relative or not path.is_relative_to(root.resolve()) or not path.is_file() or any((root/Path(*Path(relative).parts[:i])).is_symlink() for i in range(1,len(Path(relative).parts)+1)):
            raise web.HTTPNotFound()
        manifest=read_json(root/'manifest.json',{})
        committed={x['path'] for x in manifest.get('committed',[])}
        if relative not in committed and relative not in {'session.json','manifest.json','quality.json','edits.json'}:
            raise web.HTTPConflict(text='文件尚未提交或不在可下载清单中')
        return web.FileResponse(path)

    async def preview(request):
        path=session_path(hub().directory,request.match_info['ident'])
        k=int(request.query.get('k','0'))
        source=request.query.get('source','')
        version=request.query.get('version','online')
        def decode():
            from PIL import Image
            from ..recording.exporter import FrameReader
            row=row_at(path,k,version)
            binding=row.get('video_bindings',{}).get(source)
            committed={x['path'] for x in read_json(path/'manifest.json',{}).get('committed',[])}
            if binding is None or binding['video_segment_path'] not in committed:
                raise ValueError('目标图像尚未提交，无法预览')
            with preview_lock:
                key=(str(path),source)
                if key not in preview_readers:
                    preview_readers[key]=FrameReader(path)
                preview_readers.move_to_end(key)
                while len(preview_readers)>4:
                    _,reader=preview_readers.popitem(last=False)
                    reader.close()
                pixels=preview_readers[key].read(binding,seek=True)
                image=Image.fromarray(pixels)
                image.thumbnail((1280,960))
                output=io.BytesIO()
                image.save(output,format='JPEG',quality=92)
                return output.getvalue()
        return web.Response(body=await asyncio.to_thread(decode),content_type='image/jpeg',headers={'Cache-Control':'private, max-age=3600'})

    async def cleanup(_app):
        with preview_lock:
            for reader in preview_readers.values():
                reader.close()
            preview_readers.clear()

    app.router.add_get('/api/capture',state)
    app.router.add_post('/api/capture/sample',sample)
    app.router.add_post('/api/capture/{operation}',operation)
    app.router.add_get('/api/sessions',sessions)
    app.router.add_get('/api/sessions/{ident}',detail)
    app.router.add_get('/api/sessions/{ident}/row',row)
    app.router.add_get('/api/sessions/{ident}/file',file)
    app.router.add_get('/api/sessions/{ident}/preview',preview)
    app.router.add_post('/api/sessions/{ident}/{operation}',mutate_session)
    app.on_cleanup.append(cleanup)
