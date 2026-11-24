"""Run the ROS-independent survey planner and mission regression suites."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import unittest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report', type=Path, help='Optional JSON verification report')
    args = parser.parse_args()
    tests = Path(__file__).resolve().parents[1] / 'ros2_ws/src/mooncar_terrain/tests'
    patterns = ('test_autonomous_survey.py', 'test_survey_recovery.py',
                'test_planner_v3_features.py', 'test_raycasting_visibility.py',
                'test_localization_revisit.py', 'test_localization_revisit_integration.py',
                'test_localization_revisit_retirement.py', 'test_localization_revisit_retirement_integration.py',
                'test_localization_revisit_nearby.py', 'test_localization_revisit_nearby_integration.py',
                'test_localization_revisit_displacement.py', 'test_localization_revisit_old_anchor.py', 'test_localization_revisit_capture.py', 'test_localization_revisit_candidates.py', 'test_localization_revisit_precision.py')
    suite = unittest.TestSuite()
    for pattern in patterns:
        suite.addTests(unittest.defaultTestLoader.discover(str(tests), pattern=pattern))
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    passed = result.wasSuccessful() and not result.skipped and result.testsRun > 0
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(dict(
            scope='offline survey planner and mission unit tests; no ROS/Gazebo or hardware acceptance',
            created_utc=datetime.now(timezone.utc).isoformat(), suites=list(patterns),
            tests_run=result.testsRun, failures=len(result.failures), errors=len(result.errors),
            skipped=len(result.skipped), passed=passed), indent=2), encoding='utf-8')
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
