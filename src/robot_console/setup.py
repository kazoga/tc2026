from glob import glob
from pathlib import Path
from typing import List

from setuptools import find_packages, setup

package_name = 'robot_console'


def collect_data_files(directory: str) -> List[tuple[str, List[str]]]:
    """相対配置を保ち、キャッシュを除外して共有ファイルを収集する。"""

    base_path = Path(directory)
    if not base_path.exists():
        return []
    grouped = {}
    for path in sorted(base_path.rglob('*')):
        if (not path.is_file() or '__pycache__' in path.parts
                or '.pytest_cache' in path.parts or path.suffix in ('.pyc', '.pyo')):
            continue
        destination = str(Path('share') / package_name / path.parent)
        grouped.setdefault(destination, []).append(str(path))
    return list(grouped.items())


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(include=[package_name, f'{package_name}.*']),
    package_data={package_name: ['web/static/*']},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (f'share/{package_name}/launch', glob('launch/*.launch.py')),
        (f'share/{package_name}/config', glob('config/*.yaml')),
        (f'share/{package_name}/config/node_params',
         glob('config/node_params/**/*.yaml', recursive=True)),
        (f'share/{package_name}/rviz', glob('rviz/*.rviz')),
        *collect_data_files('docs'),
        *collect_data_files('tools'),
        (f'share/{package_name}', ['package.xml']),
    ],
    install_requires=['setuptools', 'Pillow>=9.0', 'opencv-python>=4.5'],
    zip_safe=True,
    maintainer='robot_console maintainers',
    maintainer_email='maintainer@example.com',
    description='Route monitoring console with tkinter GUI and ROS2 integrations.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'record_bag_with_map = robot_console.map_bag_record:main',
            'robot_console_qt = robot_console.ui_qt_main:main',
            'robot_console_web = robot_console.web_main:main',
        ],
    },
)
