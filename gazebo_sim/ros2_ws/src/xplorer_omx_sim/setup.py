from setuptools import setup
import os
from glob import glob

package_name = 'xplorer_omx_sim'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'),   glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'maps'),   glob('maps/*')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml') + glob('config/*.json') + glob('config/*.rviz')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*.stl')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nitu George-Catalin',
    maintainer_email='nitu.george15@gmail.com',
    description='Simulare Gazebo Xplorer-A + OMX-AI, scenariu orchestrare end-to-end',
    license='MIT',
    entry_points={
        'console_scripts': [
            'manipulation_infer_node_sim = xplorer_omx_sim.manipulation_infer_node_sim:main',
            'mission_orchestrator_sim = xplorer_omx_sim.mission_orchestrator_sim:main',
            'mission_multi_poi = xplorer_omx_sim.mission_multi_poi:main',
            'go_collect = xplorer_omx_sim.go_collect:main',
            'reset_objects = xplorer_omx_sim.reset_objects:main',
            'sim_doctor = xplorer_omx_sim.sim_doctor:main',
            'ground_truth_mux = xplorer_omx_sim.ground_truth_mux:main',
        ],
    },
)
