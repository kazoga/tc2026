from glob import glob

from setuptools import find_packages, setup


package_name = 'drive_mode_manager'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(include=[package_name, f'{package_name}.*']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (f'share/{package_name}/docs', glob('docs/*.md')),
        (f'share/{package_name}', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kazuki',
    maintainer_email='kaz.ogata1988@gmail.com',
    description='Drive mode command mux and dedicated status GUI for tc2026.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [],
    },
)
