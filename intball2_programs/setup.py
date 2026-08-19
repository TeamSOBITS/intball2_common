import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'intball2_programs'


def mesh_data_files(root_dir, install_prefix):
    """root_dir以下をディレクトリ構造を保ったままdata_filesエントリ化する。

    human_obstacles/portable_objectsのメッシュは自身のディレクトリ配置に対する
    相対URI(meshes/...等)を前提にしているため、glob('*.sdf')のような単純な
    フラットインストールでは構造が崩れて解決できなくなる。
    """
    entries = []
    for dirpath, _dirnames, filenames in os.walk(root_dir):
        if not filenames:
            continue
        rel = os.path.relpath(dirpath, root_dir)
        install_dir = install_prefix if rel == '.' else os.path.join(install_prefix, rel)
        entries.append((install_dir, [os.path.join(dirpath, f) for f in filenames]))
    return entries


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/models', glob('models/*.sdf')),
        ('share/' + package_name + '/models', glob('models/*.ply')),
        ('share/' + package_name + '/locations', glob('locations/*.yaml')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
        ('share/' + package_name + '/urdf', glob('urdf/*.urdf')),
        ('share/' + package_name + '/media/meshes/iss', glob('media/meshes/iss/*.dae')),
        ('share/' + package_name + '/media/meshes/int-ball2', glob('media/meshes/int-ball2/*.dae')),
        ('share/' + package_name + '/media/materials/textures',
            glob('media/materials/textures/*.png') + glob('media/materials/textures/*.jpg')),
        ]
        + mesh_data_files('media/meshes/human_obstacles', 'share/' + package_name + '/media/meshes/human_obstacles')
        + mesh_data_files('media/meshes/portable_objects', 'share/' + package_name + '/media/meshes/portable_objects'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rg-msi-03',
    maintainer_email='f22hakuti@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'spawn_model = intball2_programs.spawn:main',
            'spawn_lights = intball2_programs.lights.spawn_lights:main',
            'delete_lights = intball2_programs.lights.delete_lights:main',
            'spawn_location_broadcaster = intball2_programs.spawn_location_broadcaster:main',
            'move_relative = intball2_programs.move_relative:main',
            'cmd_vel = intball2_programs.cmd_vel:main',
            'servise_picture = intball2_programs.servise_picture:main',
            'crop_pointcloud = intball2_programs.stereo_points.crop_pointcloud:main',
            'fix_camera_info = intball2_programs.stereo_points.fix_camera_info:main',
            'person_distance = intball2_programs.stereo_points.person_distance:main',
            'pointcloud_distance = intball2_programs.stereo_points.pointcloud_distance:main',
            'bbox_center_distance = intball2_programs.stereo_points.bbox_center_distance:main',
            'ply_publisher = intball2_programs.gaussian_target.ply_publisher:main',
            'ply_target_extractor = intball2_programs.gaussian_target.ply_target_extractor:main',
            'ply_candidate_generator = intball2_programs.gaussian_target.ply_candidate_generator:main',
        ],
    },
)
