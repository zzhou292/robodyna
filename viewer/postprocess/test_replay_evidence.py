"""Host-only replay receipt tests; no scene checker or renderer is launched."""

from copy import deepcopy
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

from .replay_evidence import require_exact_replay


# Shape and field names from an actual successful GoogleTest replay report.
# In particular, the root lacks a skipped counter while the suite includes it.
VALID_REPORT = '''<?xml version="1.0" encoding="UTF-8"?>
<testsuites tests="1" failures="0" disabled="0" errors="0" name="AllTests">
  <testsuite name="PhysicalSceneArchive" tests="1" failures="0" disabled="0" skipped="0" errors="0">
    <testcase name="ExactOriginalArchiveWallActivityAndFailedSeekPreserveDisplay"
      status="run" result="completed" classname="PhysicalSceneArchive" />
  </testsuite>
</testsuites>
'''


class ReplayEvidenceTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.report = Path(temporary.name) / 'replay.xml'

    def validate(self, root):
        self.report.write_bytes(ET.tostring(root))
        require_exact_replay(self.report)

    def test_accepts_executed_case_with_actual_google_test_shape(self):
        self.report.write_text(VALID_REPORT)
        require_exact_replay(self.report)

    def test_accepts_zero_root_skip_count_and_case_properties(self):
        root = ET.fromstring(VALID_REPORT)
        root.set('skipped', '0')
        properties = ET.SubElement(root[0][0], 'properties')
        ET.SubElement(properties, 'property', name='archive', value='checked')
        self.validate(root)

    def test_rejects_runtime_skip_even_when_aggregate_failures_are_zero(self):
        root = ET.fromstring(VALID_REPORT)
        root[0].set('skipped', '1')
        root[0][0].set('result', 'skipped')
        ET.SubElement(root[0][0], 'skipped', message='No archive supplied')
        with self.assertRaises(ValueError):
            self.validate(root)

    def test_rejects_wrong_identity_or_incomplete_execution(self):
        changes = ((0, 'name', 'OtherSuite'), (1, 'classname', 'OtherSuite'),
                   (1, 'name', 'DifferentTest'), (1, 'status', 'notrun'),
                   (1, 'result', 'skipped'), (1, 'status', None),
                   (1, 'result', None))
        for location, field, value in changes:
            with self.subTest(location=location, field=field, value=value):
                root = ET.fromstring(VALID_REPORT)
                element = root[0] if location == 0 else root[0][0]
                if value is None:
                    del element.attrib[field]
                else:
                    element.set(field, value)
                with self.assertRaises(ValueError):
                    self.validate(root)

    def test_rejects_missing_or_duplicate_records_and_wrong_root(self):
        for mutation in ('no_suite', 'no_case', 'two_suites', 'two_cases',
                         'nested_case', 'nested_suite', 'root'):
            with self.subTest(mutation=mutation):
                root = ET.fromstring(VALID_REPORT)
                if mutation == 'no_suite':
                    root.remove(root[0])
                elif mutation == 'no_case':
                    root[0].remove(root[0][0])
                elif mutation == 'two_suites':
                    root.append(deepcopy(root[0]))
                elif mutation == 'two_cases':
                    root[0].append(deepcopy(root[0][0]))
                elif mutation == 'nested_case':
                    root[0][0].append(deepcopy(root[0][0]))
                elif mutation == 'nested_suite':
                    root[0][0].append(deepcopy(root[0]))
                else:
                    root.tag = 'other'
                with self.assertRaises(ValueError):
                    self.validate(root)

    def test_rejects_inconsistent_or_malformed_counts(self):
        for location in (0, 1):
            for field in ('tests', 'failures', 'disabled', 'errors', 'skipped'):
                for value in ('2', '-1', '1.0', 'invalid', '', None):
                    if field == 'skipped' and value is None:
                        continue  # GoogleTest versions may omit this counter.
                    with self.subTest(location=location, field=field, value=value):
                        root = ET.fromstring(VALID_REPORT)
                        element = root if location == 0 else root[0]
                        if value is None:
                            del element.attrib[field]
                        else:
                            element.set(field, value)
                        with self.assertRaises(ValueError):
                            self.validate(root)

    def test_rejects_failure_or_skip_records_even_with_zero_counters(self):
        for tag in ('skipped', 'failure', 'error'):
            with self.subTest(tag=tag):
                root = ET.fromstring(VALID_REPORT)
                ET.SubElement(root[0][0], tag, message='Contradicts completed status')
                with self.assertRaises(ValueError):
                    self.validate(root)

    def test_rejects_malformed_or_oversized_xml(self):
        for data in (b'', b'<testsuites>', b'x' * (2 * 1024 * 1024 + 1)):
            with self.subTest(bytes=len(data)):
                self.report.write_bytes(data)
                with self.assertRaises(ValueError):
                    require_exact_replay(self.report)


if __name__ == '__main__':
    unittest.main()
