from glob import glob

from setuptools import setup

setup(
    name='hexapod_controller',
    version='0.1.0',
    packages=['hexapod_controller'],
    data_files=[('share/ament_index/resource_index/packages', ['resource/hexapod_controller']),
                ('share/hexapod_controller', ['package.xml']),
                ('share/hexapod_controller/config', glob('config/*.yaml')),
                ('share/hexapod_controller/launch', glob('launch/*.py'))],
    install_requires=['setuptools'],
    zip_safe=True,
    license='GPL-3.0-only',
    entry_points={'console_scripts': ['controller_node = hexapod_controller.controller_node:main']},
)
