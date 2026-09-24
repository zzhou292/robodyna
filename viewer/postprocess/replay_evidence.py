"""Require an executed exact Chrono replay test before starting capture."""

from pathlib import Path
import re
import xml.etree.ElementTree as ET


EXACT_REPLAY_TEST = (
    'PhysicalSceneArchive.'
    'ExactOriginalArchiveWallActivityAndFailedSeekPreserveDisplay'
)
_TEXT_CAP = 2 * 1024 * 1024


def _require_count(element, name, expected, optional=False):
    value = element.get(name)
    if optional and value is None:
        return
    if value is None or re.fullmatch(r'[0-9]+', value) is None or int(value) != expected:
        raise ValueError(f'Exact archive replay has invalid {element.tag} {name} count')


def require_exact_replay(report):
    """Accept one named, completed GoogleTest case; aggregate counts alone fail.

    The caller still owns archive authentication and bounded process execution.
    This checks their replay test evidence without running a checker or renderer.
    """
    with Path(report).open('rb') as stream:
        data = stream.read(_TEXT_CAP + 1)
    if len(data) > _TEXT_CAP:
        raise ValueError('Exact archive replay XML exceeds its text limit')
    try:
        root = ET.fromstring(data)
    except ET.ParseError as error:
        raise ValueError('Exact archive replay XML is malformed') from error

    if root.tag != 'testsuites' or len(root) != 1 or root[0].tag != 'testsuite':
        raise ValueError('Exact archive replay requires exactly one test suite')
    suite = root[0]
    if len(suite) != 1 or suite[0].tag != 'testcase':
        raise ValueError('Exact archive replay requires exactly one test case')
    case = suite[0]
    if len(list(root.iter('testcase'))) != 1 or len(list(root.iter('testsuite'))) != 1:
        raise ValueError('Exact archive replay contains duplicate test records')
    expected_suite, expected_case = EXACT_REPLAY_TEST.split('.', 1)
    if (suite.get('name') != expected_suite or case.get('classname') != expected_suite
            or case.get('name') != expected_case):
        raise ValueError('Exact archive replay ran a different test')
    if case.get('status') != 'run' or case.get('result') != 'completed':
        raise ValueError('Exact archive replay test did not execute and complete')
    if any(element.tag in ('skipped', 'failure', 'error') for element in root.iter()):
        raise ValueError('Exact archive replay test was skipped or failed')
    for element in (root, suite):
        _require_count(element, 'tests', 1)
        for name in ('failures', 'disabled', 'errors'):
            _require_count(element, name, 0)
        # GoogleTest emits skipped on the suite, but some versions omit it
        # on the root. A present counter must agree with the completed case.
        _require_count(element, 'skipped', 0, optional=True)
