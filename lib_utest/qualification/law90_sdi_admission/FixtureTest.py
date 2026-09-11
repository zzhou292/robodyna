#!/usr/bin/env python3
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

import fixture
import prepare_sources

RECEIPT = Path('/home/jsonzhou/Desktop/chrono-work/crash-work/reports/yaris-radiator-geometry-1/manifest.json')


class FixtureTests(unittest.TestCase):
    def test_original_bytes_and_named_controls(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            m = fixture.fixture(RECEIPT, root)
            text = (root / 'original.key').read_text()
            for block in m['source_blocks']:
                self.assertIn(block['raw_text'], text)
            card = next(b for b in m['source_blocks'] if b['family'] == 'material')['cards'][0]['text']
            self.assertEqual(card.ljust(80)[50:60], ' ' * 10)
            self.assertEqual(card.ljust(80)[70:80], ' ' * 10)
            explicit = (root / 'explicit_hu_one.key').read_text()
            changed = next(line for line in explicit.splitlines() if line.startswith('   20000637.'))
            self.assertEqual(float(changed[50:60]), 1)
            self.assertEqual(changed[:50], card[:50])
            damp = (root / 'source_damp_control.key').read_text()
            changed = next(line for line in damp.splitlines() if line.startswith('   20000637.'))
            self.assertEqual(float(changed[70:80]), .2)
            self.assertEqual(changed[:70], card.ljust(80)[:70])
            self.assertEqual(m['curve_working_xy'][-2:], [.92, 20000])

    def test_mutated_source_receipt_is_rejected_before_output(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            data = json.loads(RECEIPT.read_text())
            data['source_mass_unit'] = 'kg'
            altered = root / 'altered.json'; altered.write_text(json.dumps(data))
            with self.assertRaisesRegex(ValueError, 'receipt identity'):
                fixture.fixture(altered, root / 'out')
            self.assertFalse((root / 'out').exists())

    def test_existing_different_fixture_is_preserved(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            destination = root / 'original.key'
            destination.write_text('prior incompatible evidence\n')
            before = destination.read_bytes()
            with self.assertRaisesRegex(ValueError, 'existing fixture differs'):
                fixture.fixture(RECEIPT, root)
            self.assertEqual(destination.read_bytes(), before)

    def test_forged_reader_manifest_cannot_stage_sources(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            (root / 'reader-source-manifest.json').write_text('{"revision":"wrong","files":[]}')
            with self.assertRaisesRegex(ValueError, 'identity mismatch'):
                list(prepare_sources.records(root, 'reader-source-manifest.json',
                                             prepare_sources.READER_SHA, 'original'))


if __name__ == '__main__':
    unittest.main()
