from glob import glob
from setuptools import find_packages, setup

package_name = 'intball2_programs'

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
    ],
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
            'move_relative = intball2_programs.move_relative:main',
            'cmd_vel = intball2_programs.cmd_vel:main',
            'servise_picture = intball2_programs.servise_picture:main',
        ],
    },
)
