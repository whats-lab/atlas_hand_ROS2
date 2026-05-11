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

urdf_files = _collect_data_files(['models'])

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
            'osc_receiver = atlas_hand.osc_receiver_node:main',
            'retarget     = atlas_hand.retargeting_node:main',
            'visualizer   = atlas_hand.visualizer_node:main',
        ],
    },
)
