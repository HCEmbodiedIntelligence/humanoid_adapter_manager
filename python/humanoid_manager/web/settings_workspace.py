"""Separate editing, durable configuration and the running HC configuration."""
from __future__ import annotations

import copy
from functools import wraps
import fcntl
import hashlib
import json
import os
from pathlib import Path
import tempfile
import time
import uuid

from aiohttp import web

from .config import ConfigError, validate_config


def fingerprint(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, allow_nan=False).encode()).hexdigest()


def atomic_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, name = tempfile.mkstemp(dir=path.parent, prefix=".write-")
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            json.dump(value, stream, ensure_ascii=False, allow_nan=False)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(name, path)
    finally:
        if os.path.exists(name):
            os.unlink(name)


def locked(method):
    @wraps(method)
    def call(self, *args, **kwargs):
        self.root.mkdir(parents=True, exist_ok=True)
        with (self.root / '.lock').open('a+') as stream:
            fcntl.flock(stream, fcntl.LOCK_EX)
            return method(self, *args, **kwargs)
    return call


class SettingsWorkspace:
    def __init__(self, store, runtime):
        self.store, self.runtime = store, runtime
        self.root = store.path.parent / f".{store.path.name}.history"
        self.draft_path = self.root / "draft.json"

    def _saved(self):
        # Detect a CLI/editor change instead of overwriting its stale cache.
        return self.store.load()

    def state(self):
        saved = self._saved()
        try:
            draft = json.loads(self.draft_path.read_text())
        except FileNotFoundError:
            draft = {"config": saved, "base": fingerprint(saved)}
        history = []
        for path in sorted(self.root.glob("v-*.json"), reverse=True):
            item = json.loads(path.read_text())
            history.append({"revision": path.stem, "created_at": item["created_at"]})
        return {"saved": saved, "draft": draft["config"], "active": self.runtime.config,
                "etag": fingerprint({"saved": saved, "draft": draft}),
                "external_change": draft["base"] != fingerprint(saved),
                "pending": fingerprint(saved) != fingerprint(self.runtime.config),
                "history": history}

    def check(self, etag):
        state = self.state()
        if etag != state["etag"]:
            raise web.HTTPConflict(text="配置已被其他窗口或程序修改，请刷新后重新编辑")
        return state

    @locked
    def draft(self, config, etag):
        state = self.check(etag)
        atomic_json(self.draft_path, {"config": validate_config(config), "base": fingerprint(state["saved"])})
        return self.state()

    @locked
    def save(self, etag):
        state = self.check(etag)
        if state["external_change"]:
            raise web.HTTPConflict(text="磁盘配置已变化，请重新读取并合并修改后保存")
        self.root.mkdir(parents=True, exist_ok=True)
        if not list(self.root.glob("v-*.json")):
            atomic_json(self.root / f"v-{time.time_ns()}-initial.json", {"config": state["saved"], "created_at": time.time()})
        config = self.store.save(state["draft"])
        atomic_json(self.root / f"v-{time.time_ns()}-{uuid.uuid4().hex[:8]}.json", {"config": config, "created_at": time.time()})
        atomic_json(self.draft_path, {"config": config, "base": fingerprint(config)})
        return self.state()

    def busy(self):
        if getattr(self.runtime, 'dataset_operations', 0):
            raise web.HTTPConflict(text="正在处理数据文件，处理完成后再应用配置；仍可保存修改")
        if getattr(self.runtime, 'capture', None) and self.runtime.capture.busy():
            raise web.HTTPConflict(text='对齐采集或数据处理正在运行，请完成后再应用配置')
        if self.runtime.recorder and self.runtime.recorder.is_recording():
            raise web.HTTPConflict(text="正在录制；配置可以保存，请停止录制后再应用")
        if self.runtime.player and self.runtime.player.status().get("is_active"):
            raise web.HTTPConflict(text="正在回放，请停止回放后再应用配置")

    async def apply(self, etag):
        state = self.check(etag)
        self.busy()
        if getattr(self.runtime, 'launcher', None) and self.runtime.launcher.busy:
            raise web.HTTPConflict(text='请先停止机器人，再应用网页运行设置')
        config, active = state["saved"], copy.deepcopy(self.runtime.config)
        restart = (config["server"] != active["server"] or
                   config["ros"]["domain_id"] != active["ros"]["domain_id"] or
                   config.get("adapter_manager") != active.get("adapter_manager"))
        if not restart and fingerprint(config) != fingerprint(active):
            try:
                await self.runtime.restart(copy.deepcopy(config))
            except Exception as error:
                await self.runtime.restart(active)
                raise web.HTTPInternalServerError(text=f"应用失败，已恢复原运行配置: {error}") from error
        return {**self.state(), "server_restart_required": restart}


def register_settings_routes(app, store, runtime):
    workspace = SettingsWorkspace(store, runtime)

    async def state(_request):
        return web.json_response(workspace.state())

    async def action(request):
        try:
            data = await request.json()
            operation = request.match_info["operation"]
            if operation == "draft":
                result = workspace.draft(data["config"], data.get("etag"))
            elif operation == "save":
                result = workspace.save(data.get("etag"))
            elif operation == "apply":
                result = await workspace.apply(data.get("etag"))
            elif operation == "restore":
                revision = data.get("revision", "")
                if not isinstance(revision, str) or not revision.startswith("v-") or any(c not in "abcdefghijklmnopqrstuvwxyz0123456789-" for c in revision):
                    raise ConfigError("无效的历史版本")
                config = json.loads((workspace.root / f"{revision}.json").read_text())["config"]
                result = workspace.draft(config, data.get("etag"))
            else:
                raise web.HTTPNotFound()
            return web.json_response(result)
        except (ConfigError, ValueError, KeyError, TypeError, OSError) as error:
            raise web.HTTPBadRequest(text=str(error)) from error

    app.router.add_get("/api/settings", state)
    app.router.add_post("/api/settings/{operation}", action)
    return workspace
