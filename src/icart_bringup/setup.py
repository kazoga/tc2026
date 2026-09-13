from glob import glob
from os.path import isfile
from setuptools import setup

setup(name='icart_bringup', version='0.1.0', packages=['icart_bringup'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/icart_bringup']),
                  ('share/icart_bringup', ['package.xml', 'README.md']),
                  *[('share/icart_bringup/'+folder, [p for p in glob(folder+'/*') if isfile(p)])
                    for folder in ['launch', 'params', 'docs']]],
      scripts=glob('tools/*.py'), install_requires=['setuptools', 'PyYAML'], zip_safe=True,
      maintainer='Kazuki Ogata', maintainer_email='kaz.ogata1988@gmail.com',
      description='実機とデジタルツインの共通起動', license='Apache-2.0',
      entry_points={'console_scripts': ['prepare_session = icart_bringup.session_core:main',
                                          'run_digital_twin = icart_bringup.digital_twin_core:main',
                                          'run_session = icart_bringup.session_core:run_main']})
