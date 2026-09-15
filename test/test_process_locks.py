"""Real kernel locks and parent death signals; children never load ROS or hardware."""
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import pytest

from humanoid_manager.deployment import DeploymentError
from humanoid_manager.runtime_state import (
    acquire_manager_run_lock, acquire_robot_run_lock, bind_to_parent,
)
from humanoid_manager.web.robot_launcher import _group_running


def wait_until(predicate, timeout=10):
    deadline = time.monotonic() + timeout
    while not predicate():
        if time.monotonic() >= deadline:
            pytest.fail('Timed out waiting for harmless test process')
        time.sleep(.02)


@pytest.mark.parametrize('acquire,filename', [
    (acquire_manager_run_lock, '.manager-run.lock'),
    (acquire_robot_run_lock, '.robot-run.lock'),
])
def test_process_lock_survives_duplicate_and_releases_after_crash(tmp_path, acquire, filename):
    root = tmp_path / 'plugins'
    alias = tmp_path / 'alias'
    root.mkdir()
    alias.symlink_to(root, target_is_directory=True)
    code = f'''import sys,time
from humanoid_manager.runtime_state import {acquire.__name__}
with {acquire.__name__}(sys.argv[1]):
    print('locked',flush=True)
    time.sleep(60)
'''
    holder = subprocess.Popen([sys.executable, '-c', code, str(root)],
                              stdout=subprocess.PIPE, text=True)
    try:
        wait_until(lambda: (root / filename).exists() and (root / filename).read_text().isdigit())
        assert (root / filename).read_text() == str(holder.pid)
        with pytest.raises(DeploymentError, match=f'PID {holder.pid}'):
            acquire(alias)
        # Rejection must not truncate the lock owner's identity.
        assert (root / filename).read_text() == str(holder.pid)
        with acquire(tmp_path / 'another_robot'):
            pass
        holder.kill()
        holder.wait(timeout=5)
        # The file stays, but flock releases automatically; no stale-file deletion.
        with acquire(root):
            assert (root / filename).read_text() == str(os.getpid())
    finally:
        if holder.poll() is None:
            holder.kill()
        holder.wait(timeout=5)
        holder.stdout.close()


def test_manager_and_robot_have_separate_locks(tmp_path):
    with acquire_manager_run_lock(tmp_path), acquire_robot_run_lock(tmp_path):
        with pytest.raises(DeploymentError, match='机器人管理进程'):
            acquire_manager_run_lock(tmp_path)
        with pytest.raises(DeploymentError, match='机器人启动进程'):
            acquire_robot_run_lock(tmp_path)


def test_dead_owner_is_rejected_before_starting():
    with pytest.raises(DeploymentError, match='启动进程已退出'):
        bind_to_parent(1)
    with pytest.raises(DeploymentError, match='启动进程已退出'):
        bind_to_parent(os.getpid())


def test_killed_manager_signals_robot_and_keeps_lock_until_cleanup(tmp_path):
    child_code = '''import os,signal,sys,threading,time
from pathlib import Path
from humanoid_manager.runtime_state import acquire_robot_run_lock,bind_to_parent
root=Path(sys.argv[1]);stopping=threading.Event()
signal.signal(signal.SIGINT,lambda *args:stopping.set())
bind_to_parent(int(sys.argv[2]))
with acquire_robot_run_lock(root):
    (root/'ready').write_text(str(os.getpid()))
    if not stopping.wait(30):raise RuntimeError('owner death signal missing')
    (root/'stopping').touch()
    deadline=time.monotonic()+10
    while not (root/'finish').exists() and time.monotonic()<deadline:time.sleep(.02)
'''
    owner_code = '''import os,subprocess,sys,time
from humanoid_manager.runtime_state import acquire_manager_run_lock
with acquire_manager_run_lock(sys.argv[1]):
    subprocess.Popen([sys.executable,'-c',sys.argv[2],sys.argv[1],str(os.getpid())],start_new_session=True)
    time.sleep(60)
'''
    owner = subprocess.Popen([sys.executable, '-c', owner_code, str(tmp_path), child_code])
    child_pid = None
    try:
        wait_until(lambda: (tmp_path / 'ready').exists() and (tmp_path / 'ready').read_text().isdigit())
        child_pid = int((tmp_path / 'ready').read_text())
        owner.kill()  # No Python finally or normal shutdown can run in the owner.
        owner.wait(timeout=5)
        wait_until(lambda: (tmp_path / 'stopping').exists())
        with pytest.raises(DeploymentError, match='重复启动'):
            acquire_robot_run_lock(tmp_path)
        (tmp_path / 'finish').touch()
        wait_until(lambda: not _group_running(child_pid))
        with acquire_manager_run_lock(tmp_path), acquire_robot_run_lock(tmp_path):
            pass
    finally:
        (tmp_path / 'finish').touch()
        if owner.poll() is None:
            owner.kill()
        owner.wait(timeout=5)
        if child_pid and _group_running(child_pid):
            os.killpg(child_pid, signal.SIGKILL)
