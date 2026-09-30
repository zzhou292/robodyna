"""Bounded completed-test evidence; no scientific or solver authority."""
from pathlib import Path
import re
import xml.etree.ElementTree as ET

_TEXT_CAP = 2 * 1024 * 1024

def _require_count(element, name, expected, optional=False):
    value = element.get(name)
    if optional and value is None:
        return
    if value is None or re.fullmatch(r'[0-9]+', value) is None or int(value) != expected:
        raise ValueError(f'Named test evidence has invalid {element.tag} {name} count')


def require_completed_test(report, expected_test, required_properties=None):
    """Accept one named, completed GoogleTest case; aggregate counts alone fail.

    The caller still owns archive authentication and bounded process execution.
    This checks their replay test evidence without running a checker or renderer.
    """
    with Path(report).open('rb') as stream:
        data = stream.read(_TEXT_CAP + 1)
    if len(data) > _TEXT_CAP:
        raise ValueError('Named test evidence XML exceeds its text limit')
    try:
        root = ET.fromstring(data)
    except ET.ParseError as error:
        raise ValueError('Named test evidence XML is malformed') from error

    if root.tag != 'testsuites' or len(root) != 1 or root[0].tag != 'testsuite':
        raise ValueError('Named test evidence requires exactly one test suite')
    suite = root[0]
    if len(suite) != 1 or suite[0].tag != 'testcase':
        raise ValueError('Named test evidence requires exactly one test case')
    case = suite[0]
    if len(list(root.iter('testcase'))) != 1 or len(list(root.iter('testsuite'))) != 1:
        raise ValueError('Named test evidence contains duplicate test records')
    expected_suite, expected_case = expected_test.split(".", 1)
    if (suite.get('name') != expected_suite or case.get('classname') != expected_suite
            or case.get('name') != expected_case):
        raise ValueError('Named test evidence ran a different test')
    if case.get('status') != 'run' or case.get('result') != 'completed':
        raise ValueError('Named test evidence test did not execute and complete')
    if any(element.tag in ('skipped', 'failure', 'error') for element in root.iter()):
        raise ValueError('Named test evidence test was skipped or failed')
    for element in (root, suite):
        _require_count(element, 'tests', 1)
        for name in ('failures', 'disabled', 'errors'):
            _require_count(element, name, 0)
        # GoogleTest emits skipped on the suite, but some versions omit it
        # on the root. A present counter must agree with the completed case.
        _require_count(element, 'skipped', 0, optional=True)

    if required_properties is None:
        return {}
    properties = case.findall("properties")
    if len(properties) != 1:
        raise ValueError("Named numerical test needs one property set")
    values = {}
    for entry in properties[0]:
        name = entry.get("name")
        if entry.tag != "property" or name is None or name in values:
            raise ValueError("Invalid or duplicate numerical evidence property")
        values[name] = entry.get("value")
    if any(values.get(name) != value for name, value in required_properties.items()):
        raise ValueError("Named numerical evidence properties differ")
    return values
