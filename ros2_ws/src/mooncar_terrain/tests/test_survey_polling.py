"""Polling must tolerate a manifest that is absent or being replaced."""
from pathlib import Path
import sys
import tempfile
import errno
from unittest.mock import Mock
sys.path.insert(0,str(Path(__file__).resolve().parents[4]/'scripts'))
from compare_survey_strategies import read_pending_json


def test_manifest_replacement():
    with tempfile.TemporaryDirectory() as directory:
        path=Path(directory)/'manifest.json'
        assert read_pending_json(path) is None
        path.write_text('{',encoding='utf-8')
        assert read_pending_json(path) is None
        path.write_text('{"status":"recording"}',encoding='utf-8')
        assert read_pending_json(path)=={'status':'recording'}
    source=Mock();source.read_text.side_effect=[OSError(getattr(errno,'ENODATA',61),'No data available'),'{"status":"recording"}']
    assert read_pending_json(source) is None and read_pending_json(source)=={'status':'recording'}
    source.read_text.side_effect=PermissionError('Denied')
    try:read_pending_json(source)
    except PermissionError:pass
    else:raise AssertionError('Non-transient read failure was hidden')


if __name__=='__main__':
    test_manifest_replacement()
    print('Manifest polling replacement check passed')
