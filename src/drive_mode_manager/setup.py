import os
from glob import glob
from typing import List, Tuple

from setuptools import find_packages, setup


package_name = 'drive_mode_manager'


def list_data_files(target_dir: str) -> List[Tuple[str, List[str]]]:
    entries = []
    for root, _, files in os.walk(target_dir):
        if not files:
            continue
        install_dir = os.path.join('share', package_name, os.path.relpath(root, '.'))
        src_files = [os.path.join(root, file_name) for file_name in files]
        entries.append((install_dir, src_files))
    return entries


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(include=[package_name, f'{package_name}.*']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (f'share/{package_name}/docs', glob('docs/*.md')),
        (f'share/{package_name}', ['package.xml']),
    ] + list_data_files('launch') + list_data_files('params'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kazuki',
    maintainer_email='kaz.ogata1988@gmail.com',
    description='Drive mode command mux and dedicated status GUI for tc2026.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'manual_teleop_node = drive_mode_manager.manual_teleop_node:main',
            'drive_cmd_mux_node = drive_mode_manager.drive_cmd_mux_node:main',
            'drive_status_gui_node = drive_mode_manager.drive_status_gui_node:main',
            'ps3_joy_sim_node = drive_mode_manager.ps3_joy_sim_node:main',
        ],
    },
)
