from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'SCARA_tray_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name,"csv"), glob("csv/*.csv")),
        (os.path.join("share", package_name,'models'), glob("models/*.*")),
        (os.path.join("share", package_name,'launch'), glob("launch/*.launch.py*"))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='martin',
    maintainer_email='martin@email.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "inverse_kinematics= SCARA_tray_planner.inverse_kinematics:main",
            "direct_kinematics= SCARA_tray_planner.direct_kinematics:main",
            "trajectory_planner_node = SCARA_tray_planner.trajectory_planner_node:main",
            "dxf_parser_node2 = SCARA_tray_planner.dxf_parser_node2:main",
        ],
    },
)
