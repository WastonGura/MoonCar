"""Test-side interpretation only; never a vehicle readiness or permit source."""
import json
from pathlib import Path
import time


def mission_outcome(phase):
    return {'completed': 'passed', 'budget_reached': 'budget_completed'}.get(phase, 'failed')


def lifecycle_snapshot(path, errors):
    try:
        return json.loads(Path(path).read_text())
    except (FileNotFoundError, json.JSONDecodeError):
        return {}
    except OSError as error:
        errors.append(dict(wall=time.monotonic(), path=str(path), error=repr(error)))
        return dict(all_active=False, read_error=repr(error))
