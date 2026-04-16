from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'atlas_hand'

def _collect_data_files(src_dirs):
    result = []
    for src_dir in src_dirs:
        for root, _, files in os.walk(src_dir):
            file_list = [os.path.join(root, f) for f in files if not f.endswith('.py')]
            if file_list:
                result.append((os.path.join('share', package_name, root), file_list))
    return result

urdf_files = _collect_data_files(['urdf', 'rviz'])

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test', 'scripts', 'todo']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.py'))),

        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.json'))),
    ] + urdf_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='whatslab',
    maintainer_email='sdh@whatslab.co.kr',
    description='Atlas Hand Retargeting - ROS 2 Python Package',
    license='CC BY-NC-ND 4.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # OSC 수신 노드
            'osc_receiver = atlas_hand.nodes.osc_receiver:main',
            # Position 기반 리타겟팅 노드
            'retarget     = atlas_hand.nodes.retargeting:main',
            # 3D 시각화 노드 
            'visualizer   = atlas_hand.nodes.visualizer:main',
        ],
    },
)
