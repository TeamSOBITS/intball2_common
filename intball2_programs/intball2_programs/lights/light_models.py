"""ISS室内照明(iss_light1〜16)のspawn用メタ情報とSDF生成処理。

方式は docs/gazebo_light_sources.md の「方式B」(モデルの<link>内に<light>タグを
埋め込んでspawn_sdf_modelでspawnする)を採用。pose/diffuse/attenuation/direction/
cast_shadows等すべてSDF文字列に直接書き込むため、ROS2側gazebo_msgsのGetLightProperties/
SetLightPropertiesがpose等のフィールドを持たない、というROS2側の制約を受けない。

typeはjpm_light1〜3と同じPOINT(全方向発光)。sunのようなDIRECTIONAL(位置無視・平行光線)
ではなく、TFフレームに配置する意味を持たせるためPOINTを使う。

光源は物理的な衝突判定対象ではないため、コリジョンは常になし(-cフラグ相当の概念を持たない)。
"""

# iss_light1〜16は全て共通値(pose以外全同一値だったjpm_light1〜3と同じ設計思想)。
# 明るさ = diffuse / (constant + linear*距離 + quadratic*距離^2) で近似されるため、
# constantを1.0未満にすると距離0(光源直近)で1.0を超え白飛びする
# (constant:0.3で試した際に実機目視確認で発生。docs/gazebo_light_spawn_plan.md参照)。
# jpm_light/sunともconstantはほぼ1.0付近だったのはこれが理由。
# constantは1.0に固定し、距離減衰はlinear/quadraticのみで調整する。
#
# 複数同時spawnの実機確認で、1個だけの目視確認では気付かなかった問題が発覚:
# 各光源の明るさへの寄与はGazebo上で加算されるため、複数のiss_lightが近接して並ぶと
# 重なった領域で明るさが積み重なり、1個だけなら適正だった値でも全体としては白飛びする。
# attenuationの強化だけでは重なり分の積み上げを打ち消しきれなかったため、
# diffuse自体を大きく下げて全体の底上げ量そのものを抑える方向にも調整した。
LIGHT_DIFFUSE = (0.35, 0.35, 0.35, 1.0)
LIGHT_SPECULAR = (0.15, 0.15, 0.15, 1.0)
LIGHT_ATTENUATION_CONSTANT = 1.0
LIGHT_ATTENUATION_LINEAR = 2.0
LIGHT_ATTENUATION_QUADRATIC = 0.8
LIGHT_RANGE = 5.0
LIGHT_CAST_SHADOWS = False

# 個別の位置(TFフレーム)以外は全て共通値のため、メタ情報は空dictで十分
# (将来、特定のlightだけ値を変えたくなった場合はここに上書き値を追加する)。
LIGHT_MODELS = {f'iss_light{i}': {} for i in range(1, 17)}


def build_light_model_xml(model_name, meta):
    """LIGHT_MODELSのメタ情報から<light>を1つ持つSDF文字列を組み立てる。

    metaは現状未使用(全て共通値)だが、将来の個別上書きに備えて引数として受け取る。
    """
    diffuse_str = ' '.join(str(v) for v in meta.get('diffuse', LIGHT_DIFFUSE))
    specular_str = ' '.join(str(v) for v in meta.get('specular', LIGHT_SPECULAR))
    attenuation_constant = meta.get('attenuation_constant', LIGHT_ATTENUATION_CONSTANT)
    attenuation_linear = meta.get('attenuation_linear', LIGHT_ATTENUATION_LINEAR)
    attenuation_quadratic = meta.get('attenuation_quadratic', LIGHT_ATTENUATION_QUADRATIC)
    light_range = meta.get('range', LIGHT_RANGE)
    cast_shadows = meta.get('cast_shadows', LIGHT_CAST_SHADOWS)
    cast_shadows_str = 'true' if cast_shadows else 'false'

    return (
        '<?xml version="1.0"?>\n'
        '<sdf version="1.6">\n'
        f'  <model name="{model_name}">\n'
        '    <static>true</static>\n'
        '    <link name="link">\n'
        f'      <light name="{model_name}" type="point">\n'
        '        <pose>0 0 0 0 0 0</pose>\n'
        f'        <diffuse>{diffuse_str}</diffuse>\n'
        f'        <specular>{specular_str}</specular>\n'
        '        <attenuation>\n'
        f'          <range>{light_range}</range>\n'
        f'          <constant>{attenuation_constant}</constant>\n'
        f'          <linear>{attenuation_linear}</linear>\n'
        f'          <quadratic>{attenuation_quadratic}</quadratic>\n'
        '        </attenuation>\n'
        f'        <cast_shadows>{cast_shadows_str}</cast_shadows>\n'
        '      </light>\n'
        '    </link>\n'
        '  </model>\n'
        '</sdf>\n'
    )
