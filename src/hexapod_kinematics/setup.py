from setuptools import setup

# Lets both pip and colcon install the library. The library itself has no ROS imports.
setup(
    name='hexapod_kinematics',
    version='0.1.0',
    packages=['hexapod_kinematics'],
    data_files=[('share/ament_index/resource_index/packages', ['resource/hexapod_kinematics']),
                ('share/hexapod_kinematics', ['package.xml'])],
    install_requires=['numpy', 'pyyaml'],
    python_requires='>=3.10',
    zip_safe=True,
    license='GPL-3.0-only',
    description='Pure-Python kinematics, gaits and stability for the 18-DOF hexapod',
)
