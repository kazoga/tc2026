from glob import glob

from setuptools import setup

package_name = 'geo_pose_converter'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/params', glob('params/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kazuki Ogata',
    maintainer_email='kaz.ogata1988@gmail.com',
    description='LLH/ENU pose conversion utilities and ROS 2 nodes.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'geo_pose_converter_node = geo_pose_converter.geo_pose_converter_node:main',
            'route_geo_projector_node = geo_pose_converter.route_geo_projector_node:main',
            'llh_osm_viewer_node = geo_pose_converter.llh_osm_viewer_node:main',
        ],
    },
)
