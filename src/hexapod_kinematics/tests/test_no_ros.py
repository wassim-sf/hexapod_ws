"""The library must stay usable outside ROS."""
import ast
import pathlib
import subprocess
import sys

import hexapod_kinematics

ALLOWED = {'numpy', 'yaml', 'hexapod_kinematics'}


def test_only_numpy_yaml_and_the_standard_library_are_imported():
    root = pathlib.Path(hexapod_kinematics.__file__).parent
    for source in root.glob('*.py'):
        for node in ast.walk(ast.parse(source.read_text())):
            names = []
            if isinstance(node, ast.Import):
                names = [a.name.split('.')[0] for a in node.names]
            elif isinstance(node, ast.ImportFrom) and node.level == 0:
                names = [node.module.split('.')[0]]
            for name in names:
                assert name in ALLOWED or name in sys.stdlib_module_names, f'{source.name} imports {name}'


def test_import_does_not_pull_in_ros():
    code = ("import sys, hexapod_kinematics;"
            "bad = [m for m in sys.modules if m.split('.')[0] in ('rclpy', 'ament_index_python', 'rosidl_runtime_py')];"
            "sys.exit(1 if bad else 0)")
    assert subprocess.run([sys.executable, '-c', code], check=False).returncode == 0
