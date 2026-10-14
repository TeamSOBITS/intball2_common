"""spawn後のモデルの動かし方(Motion)。spawn_modelの-dの値で切り替える。"""
from intball2_programs.spawn.motion.drift import DRIFT_MODES, DRIFT_WORLD, DriftMotion
from intball2_programs.spawn.motion.iss_fixed import IssFixedMotion


def create_motion(drift_mode, node, iss_follower, model_name, relative_pos, orientation):
    """drift_mode(-dの値。未指定ならNone)に対応するMotionを返す。"""
    if drift_mode is None:
        return IssFixedMotion(node, iss_follower, model_name, relative_pos, orientation)
    return DriftMotion(node, iss_follower, model_name, relative_pos, orientation, drift_mode)


__all__ = ['DRIFT_MODES', 'DRIFT_WORLD', 'create_motion']
