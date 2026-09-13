from glob import glob

from setuptools import setup

package_name = 'gnss_lio_fusion'
setup(
    name=package_name, version='0.1.0', packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/'+package_name]),
        ('share/'+package_name, ['package.xml', 'README.md']),
        ('share/'+package_name+'/params', glob('params/*.yaml')),
        ('share/'+package_name+'/launch', glob('launch/*.launch.py')),
        ('share/'+package_name+'/docs', glob('docs/*.md')),
    ],
    scripts=glob('tools/*.py'),
    install_requires=['setuptools', 'numpy'], zip_safe=True,
    maintainer='Kazuki Ogata', maintainer_email='kaz.ogata1988@gmail.com',
    description='適応baseline品質判定とGNSS/LIO平面融合', license='Apache-2.0',
    entry_points={'console_scripts': [
        'fusion_node = gnss_lio_fusion.fusion_node:main',
    ]},
)
