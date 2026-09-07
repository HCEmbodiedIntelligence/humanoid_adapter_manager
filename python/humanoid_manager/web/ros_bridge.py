"""Frequency expectations used by the dedicated recording executor."""
DEFAULT_TOPIC_STANDARDS = {
    '/hc_teleop/joint_states': {'target_hz':100.0,'min_hz':50.0},
    '/hc_teleop/joint_cmd': {'target_hz':100.0,'min_hz':50.0},
    '/diagnostics': {'target_hz':10.0,'min_hz':1.0},
    '/hc_teleop_recv/status': {'target_hz':5.0,'min_hz':1.0},
    '/humanoid/configuration_state': {'target_hz':1.0,'min_hz':0.5},
    '/vrdata': {'target_hz':60.0,'min_hz':20.0},
}
