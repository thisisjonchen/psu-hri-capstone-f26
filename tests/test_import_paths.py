"""Check local imports without starting ROS nodes or accessing drone hardware.

Run from the repository root: python3 -m unittest discover -s tests
"""
import ast
import importlib.machinery
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


SCRIPTS = Path(__file__).resolve().parents[1] / 'src' / 'scripts'


def resolve_module(name, search_path):
    """Resolve every component without executing package initializers."""
    spec = None
    components = name.split('.')
    for index in range(len(components)):
        spec = importlib.machinery.PathFinder.find_spec(
            '.'.join(components[:index + 1]), search_path)
        if spec is None:
            return None
        search_path = spec.submodule_search_locations
        if index < len(components) - 1 and search_path is None:
            return None
    return spec


class ImportPathsTest(unittest.TestCase):
    def test_local_imports_resolve_in_drone_layout(self):
        local_names = {path.stem for path in SCRIPTS.glob('*.py')}
        local_names.update(('filterpy', 'dt_vl53l0x', 'drone_controller'))
        for path in SCRIPTS.rglob('*.py'):
            tree = ast.parse(path.read_text(), filename=str(path))
            search_path = [str(path.parent), str(SCRIPTS)]
            # FilterPy's tests/examples run with its parent on PYTHONPATH.
            # Archived UKF experiments use the same library.
            if 'filterpy' in path.parts or 'archive' in path.parts:
                search_path.insert(0, str(SCRIPTS / 'StateEstimators'))
            if path.name == 'basic_routine.py':
                search_path.insert(0, str(SCRIPTS / 'PALS'))
            for node in ast.walk(tree):
                if isinstance(node, ast.Import):
                    names = [alias.name for alias in node.names]
                elif isinstance(node, ast.ImportFrom):
                    if node.level:
                        package_dir = path.parent
                        for _ in range(node.level - 1):
                            package_dir = package_dir.parent
                        self.assertTrue(package_dir.is_relative_to(SCRIPTS),
                                        '{}:{} escapes scripts'.format(path, node.lineno))
                        continue
                    names = [node.module]
                else:
                    continue
                for name in names:
                    with self.subTest(file=str(path), line=node.lineno, module=name):
                        self.assertNotIn(name.split('.')[0], ('src', 'packages'))
                        if name.split('.')[0] in local_names:
                            self.assertIsNotNone(resolve_module(name, search_path))

    def test_pid_helpers_import_outside_repository(self):
        # Only rospy is stubbed; the PID and vector modules are imported normally.
        code = '''
import sys
import types
sys.path.insert(0, sys.argv[1])
sys.modules['rospy'] = types.ModuleType('rospy')
import pid_class
from pid_class import PID, PIDaxis
import command_values
from three_dim_vec import Position, Velocity, Error, RPY
assert PID is pid_class.PID
'''
        with tempfile.TemporaryDirectory() as directory:
            result = subprocess.run(
                [sys.executable, '-I', '-B', '-c', code, str(SCRIPTS)],
                cwd=directory, capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == '__main__':
    unittest.main()
