"""No-world startup opt-in: before native imports, never decoder-import side effects."""
import ast
import os
from pathlib import Path
import unittest
from unittest.mock import patch

SOURCE = Path(__file__).resolve().parents[1]/'scripts/rgbd_odometry_node.py'
TREE = ast.parse(SOURCE.read_text())
BOOT = TREE.body[:next(i for i, node in enumerate(TREE.body)
    if isinstance(node, ast.ImportFrom) and node.module == 'collections')]


def boot(name):
    scope = {'__name__': name}
    exec(compile(ast.Module(body=BOOT, type_ignores=[]), str(SOURCE), 'exec'), scope)


class StartupAffinityTests(unittest.TestCase):
    def test_missing_optin_keeps_original_affinity_and_import_is_always_inert(self):
        with patch.dict(os.environ, {}, clear=True), patch.object(os, 'sched_setaffinity') as change:
            boot('__main__')
            change.assert_not_called()
        with patch.dict(os.environ, {'MOONCAR_RGBD_CPUSET': 'invalid'}), patch.object(os, 'sched_setaffinity') as change:
            boot('rgbd_odometry_node')
            change.assert_not_called()

    def test_explicit_affinity_is_applied_and_verified_before_native_imports(self):
        with patch.dict(os.environ, {'MOONCAR_RGBD_CPUSET': '0,1'}), \
                patch.object(os, 'sched_getaffinity', side_effect=[set(range(6)), {0, 1}]), \
                patch.object(os, 'sched_setaffinity') as change:
            boot('__main__')
            change.assert_called_once_with(0, {0, 1})

    def test_invalid_or_unavailable_optin_fails_before_any_write(self):
        for value in ('', '0,', '0,0', '-1', '0-1', '0,6', 'a'):
            with self.subTest(value=value), patch.dict(os.environ, {'MOONCAR_RGBD_CPUSET': value}), \
                    patch.object(os, 'sched_getaffinity', return_value=set(range(6))), \
                    patch.object(os, 'sched_setaffinity') as change:
                with self.assertRaises(ValueError):
                    boot('__main__')
                change.assert_not_called()


if __name__ == '__main__':
    unittest.main()
