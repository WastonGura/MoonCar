import json
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import runtime_baseline as baseline


class RuntimeBaselineTests(unittest.TestCase):
    def test_modified_and_missing_source_are_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            source = root / 'module.py'
            source.write_text('original')
            manifest = {'source_sha256': {'module.py': baseline.sha(source)}}
            baseline.verify_sources(root, manifest)
            source.write_text('modified')
            with self.assertRaisesRegex(RuntimeError, 'Source changed'):
                baseline.verify_sources(root, manifest)
            source.unlink()
            with self.assertRaisesRegex(RuntimeError, 'Source changed'):
                baseline.verify_sources(root, manifest)

    def test_old_or_missing_install_is_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            source, target = root/'source', root/'installed'
            source.write_text('new')
            with self.assertRaisesRegex(RuntimeError, 'Installed file differs'):
                baseline.verify_pair(source, target)
            target.write_text('old')
            with self.assertRaisesRegex(RuntimeError, 'Installed file differs'):
                baseline.verify_pair(source, target)
            target.write_text('new')
            self.assertEqual(baseline.verify_pair(source, target)['sha256'], baseline.sha(source))

    def test_unverified_receipt_cannot_claim_verified(self):
        self.assertFalse(baseline.load_identity(None)['verified'])
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'receipt.json'
            path.write_text(json.dumps({'verified': False}))
            with self.assertRaisesRegex(RuntimeError, 'not verified'):
                baseline.load_identity(path)
            path.write_text(json.dumps({'verified': True, 'identity': {'git_commit': 'abc'}}))
            self.assertEqual(baseline.load_identity(path)['git_commit'], 'abc')


if __name__ == '__main__':
    unittest.main()
