"""Gazebo上のISSモデル(Noetic側シム)の物理定数。

値はシム側の設定ファイルから転記したもの。シム側の設定を変えたらここも合わせること
(docs/drift_spawn_facts.md参照)。
"""
import math

# ISS重心の位置(ISSモデル原点=iss_body基準)[m]。
# 出典: urdf/iss.urdf の iss_body <inertial><origin>。
# Gazebo上で原点が回転しても、この点はworld上で静止していることを実測で確認済み。
ISS_COG_OFFSET = (-6.03, 0.31, 3.59)

# issdynプラグインがISSに与える角速度のうち、一定成分(slope)[rad/s]。world系、3軸同値。
# 出典: Int-Ball2_platform_simulator/.../ib2_gazebo/sim/sim.yaml の issdyn_parameter/slope
#       (0.001 deg/s)。式は plugins/issdyn/src/issdyn.cpp の setIssAttitude()。
# 周期50秒の振動成分(gain 0.1 deg, freq 0.02 Hz)は含めない。
ISS_SLOPE_ANGULAR_VEL = (math.radians(0.001),) * 3
