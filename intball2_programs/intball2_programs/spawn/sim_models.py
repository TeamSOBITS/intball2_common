"""Noetic側score_managerのローカルモデル(human_obstacles/portable_objects)の
spawn用メタ情報とSDF生成処理。

Noetic側の model:// 自己参照URIは GAZEBO_MODEL_PATH からの単純なパス連結で
解決されるため(docs/floating_human_spawn.md 参照)、ここでは実際のディレクトリ
配置(GAZEBO_MODEL_PATH/<CATEGORY_BASE_PATHS[category]>/[folder/]<mesh>)に
一致するURIを組み立てる。Noetic側の配置が変わった場合はこのファイルのみを直せばよい。
"""

# astrobee_freeflyer は複数visual/複数collisionという特殊構成のため、
# SIM_MODELS辞書には含めず専用の astrobee_freeflyer.sdf テンプレートを使う(例外扱い)。
# コリジョンは本家(NASA提供)モデルの構成通り、bodyとpmc左右のboxプリミティブ3個を使う
# (メッシュ全体は複数パーツに分かれているためmesh collisionでの近似は行わない)。
# -c フラグ指定時のみ astrobee_freeflyer.sdf の {collision_block} に差し込まれる。
ASTROBEE_COLLISION_BLOCK = '''\
      <collision name="body_collision">
        <pose>-0.000794 0.000229 -0.003907 0 0 0</pose>
        <geometry>
          <box><size>0.290513 0.151942 0.281129</size></box>
        </geometry>
      </collision>
      <collision name="pmc_port_collision">
        <pose>0 -0.117546 0 0 0 0</pose>
        <geometry>
          <box><size>0.319199 0.083962 0.319588</size></box>
        </geometry>
      </collision>
      <collision name="pmc_stbd_collision">
        <pose>0 0.117546 0 0 0 0</pose>
        <geometry>
          <box><size>0.319199 0.083962 0.319588</size></box>
        </geometry>
      </collision>
'''

CATEGORY_BASE_PATHS = {
    'human_obstacles': 'human_obstacles/meshes',
    'portable_objects': 'portable_objects/meshes',
}

# category: CATEGORY_BASE_PATHS のキー
# folder: portable_objects のようにアイテムごとのサブフォルダを持つ場合に指定
#         (human_obstacles はフォルダ無しでメッシュが直下に並ぶ)
# mesh: フォルダ(あれば)からの相対パス
# static: True の場合 <static>true</static> とし <inertial>/<gravity> を省略
# scale: メッシュのスケール(未指定なら1)
# pose_rpy: (roll, pitch, yaw)[rad]。メッシュ原点姿勢の補正用
SIM_MODELS = {
    # human_obstacles(旧 HUMAN_FLOAT_MESHES)
    'float_blue':    {'category': 'human_obstacles', 'mesh': 'float_blue.dae', 'pose_rpy': (3.14159, 0, 0)},
    'float_orange':  {'category': 'human_obstacles', 'mesh': 'float_orange.dae', 'pose_rpy': (3.14159, 0, 0)},
    'float_purple':  {'category': 'human_obstacles', 'mesh': 'float_purple.dae', 'pose_rpy': (3.14159, 0, 0)},
    'float2_blue':   {'category': 'human_obstacles', 'mesh': 'float2_blue.dae', 'pose_rpy': (3.14159, 0, 0)},
    'float2_green':  {'category': 'human_obstacles', 'mesh': 'float2_green.dae', 'pose_rpy': (3.14159, 0, 0)},
    'float2_orange': {'category': 'human_obstacles', 'mesh': 'float2_orange.dae', 'pose_rpy': (3.14159, 0, 0)},
    'float2_purple': {'category': 'human_obstacles', 'mesh': 'float2_purple.dae', 'pose_rpy': (3.14159, 0, 0)},

    # portable_objects
    'tape': {
        'category': 'portable_objects',
        'folder': 'Shurtape_Gaffers_Tape_Silver_2_x_60_yd',
        'mesh': 'meshes/model.obj',
        'static': True,
        'pose_rpy': (3.14, 0, 0),
    },
    'ctb_1023': {
        'category': 'portable_objects', 'folder': 'CTB_1023', 'mesh': 'CTB_1023.dae',
        'static': True, 'scale': 0.3, 'pose_rpy': (3.14, 0, 0),
    },
    'ctb_1456': {
        'category': 'portable_objects', 'folder': 'CTB_1456', 'mesh': 'CTB_1456.dae',
        'static': True, 'scale': 0.3, 'pose_rpy': (3.14, 0, 0),
    },  # 目視確認済み(collision on/off両方)
    # 以下3種は CTB_1023/ctb_1456 と同一構造(meshes/サブフォルダ無し、直下にdae)と推定しているが
    # 個別未確認。使用前に一度spawnしてGazebo画面で目視確認すること。
    'ctb_2305': {
        'category': 'portable_objects', 'folder': 'CTB_2305', 'mesh': 'CTB_2305.dae',
        'static': True, 'scale': 0.3, 'pose_rpy': (3.14, 0, 0),
    },
    'ctb_2789': {
        'category': 'portable_objects', 'folder': 'CTB_2789', 'mesh': 'CTB_2789.dae',
        'static': True, 'scale': 0.3, 'pose_rpy': (3.14, 0, 0),
    },
    'ctb_4678': {
        'category': 'portable_objects', 'folder': 'CTB_4678', 'mesh': 'CTB_4678.dae',
        'static': True, 'scale': 0.3, 'pose_rpy': (3.14, 0, 0),
    },
}


def _mesh_uri(meta):
    base = CATEGORY_BASE_PATHS[meta['category']]
    folder = meta.get('folder')
    prefix = f'{base}/{folder}/' if folder else f'{base}/'
    return f'model://{prefix}{meta["mesh"]}'


# rviz(ROS2側)でMarker表示するためのメッシュ配置。Noetic側のGAZEBO_MODEL_PATH配置
# (CATEGORY_BASE_PATHS、末尾に/meshesが付く)とは異なり、intball2_programsパッケージ内では
# media/meshes/<category>/ 直下にそのまま置いている(docs/portable_objects_spawn.md参照)。
LOCAL_MESH_CATEGORY_PATHS = {
    'human_obstacles': 'human_obstacles',
    'portable_objects': 'portable_objects',
}


def local_mesh_uri(meta):
    """rvizのMESH_RESOURCE Marker用に package://intball2_programs/media/meshes/... を返す。"""
    base = LOCAL_MESH_CATEGORY_PATHS[meta['category']]
    folder = meta.get('folder')
    prefix = f'{base}/{folder}/' if folder else f'{base}/'
    return f'package://intball2_programs/media/meshes/{prefix}{meta["mesh"]}'


# astrobee_freeflyer.sdf の各<visual>と対応させたパーツ一覧(Marker表示用)。
# rpy はメッシュ自身のローカル回転補正(ラジアン)。pmc_stbd系はsdf側と同じ(0, 3.14, 3.14)を反映。
ASTROBEE_MESH_FOLDER = 'astrobee_freeflyer'
ASTROBEE_VISUAL_PARTS = [
    {'mesh': 'meshes/body.dae', 'rpy': (0.0, 0.0, 0.0)},
    {'mesh': 'meshes/pmc.dae', 'rpy': (0.0, 0.0, 0.0)},
    {'mesh': 'meshes/pmc_bumper.dae', 'rpy': (0.0, 0.0, 0.0)},
    {'mesh': 'meshes/pmc_skin_.dae', 'rpy': (0.0, 0.0, 0.0)},
    {'mesh': 'meshes/pmc.dae', 'rpy': (0.0, 3.14, 3.14)},
    {'mesh': 'meshes/pmc_bumper.dae', 'rpy': (0.0, 3.14, 3.14)},
    {'mesh': 'meshes/pmc_skin_.dae', 'rpy': (0.0, 3.14, 3.14)},
]


def astrobee_mesh_uri(part):
    return f'package://intball2_programs/media/meshes/portable_objects/{ASTROBEE_MESH_FOLDER}/{part["mesh"]}'


def build_sim_model_xml(model_name, meta, collision_enabled):
    """SIM_MODELS のメタ情報から 1 visual (+ 任意で1 collision) のSDF文字列を組み立てる。

    collision_enabled が True のときだけ <collision> を追加する(-c フラグ経由)。
    コリジョンは常にビジュアルと同じメッシュを使う(箱等での近似は行わない)。
    """
    uri = _mesh_uri(meta)
    scale = meta.get('scale', 1)
    scale_str = f'{scale} {scale} {scale}'
    roll, pitch, yaw = meta.get('pose_rpy', (0, 0, 0))
    pose_str = f'{roll} {pitch} {yaw}'
    static = meta.get('static', False)

    static_block = '<static>true</static>' if static else '<static>false</static>'

    gravity_block = '' if static else '      <gravity>0</gravity>\n'
    inertial_block = '' if static else (
        '      <inertial>\n'
        '        <mass>1.0</mass>\n'
        '        <inertia>\n'
        '          <ixx>0.1</ixx><iyy>0.1</iyy><izz>0.1</izz>\n'
        '          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>\n'
        '        </inertia>\n'
        '      </inertial>\n'
    )

    collision_block = ''
    if collision_enabled:
        collision_block = (
            '      <collision name="collision">\n'
            f'        <pose>0 0 0 {pose_str}</pose>\n'
            '        <geometry>\n'
            '          <mesh>\n'
            f'            <uri>{uri}</uri>\n'
            f'            <scale>{scale_str}</scale>\n'
            '          </mesh>\n'
            '        </geometry>\n'
            '      </collision>\n'
        )

    return (
        '<?xml version="1.0"?>\n'
        '<sdf version="1.6">\n'
        f'  <model name="{model_name}">\n'
        f'    {static_block}\n'
        '    <pose>0 0 0 0 0 0</pose>\n'
        '    <link name="link">\n'
        f'{gravity_block}'
        '      <self_collide>false</self_collide>\n'
        f'{inertial_block}'
        f'{collision_block}'
        '      <visual name="visual">\n'
        f'        <pose>0 0 0 {pose_str}</pose>\n'
        '        <geometry>\n'
        '          <mesh>\n'
        f'            <uri>{uri}</uri>\n'
        f'            <scale>{scale_str}</scale>\n'
        '          </mesh>\n'
        '        </geometry>\n'
        '      </visual>\n'
        '    </link>\n'
        '  </model>\n'
        '</sdf>\n'
    )
