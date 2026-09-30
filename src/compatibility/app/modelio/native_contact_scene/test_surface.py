"""Explicit moving-primary declaration and exact fixed-profile compatibility."""
from dataclasses import replace
import hashlib
import json
from pathlib import Path
import tempfile
import unittest
from .definition import load
from .export import export
from .mesh import build
from .native import starter

ROOT = Path(__file__).resolve().parents[2]
SCENES = ROOT / 'benchmarks/native_contact_scene'


class SurfaceDeclarationTests(unittest.TestCase):
    def test_fixed_v1_export_and_decks_remain_byte_identical_to_qualified_source(self):
        hashes = json.loads(Path(__file__).with_name('v1-export-hashes.json').read_text())['fixtures']
        with tempfile.TemporaryDirectory() as temporary:
            for fixture, expected in hashes.items():
                destination = Path(temporary) / fixture
                record = export(SCENES / fixture, destination)
                self.assertEqual(record['schema'], 'robo_dyna.native_contact_scene_export.v1')
                self.assertNotIn('contact_surface', record['scene'])
                self.assertNotIn('definition_version', record['scene'])
                self.assertEqual({p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                                  for p in destination.iterdir()}, expected)

    def test_v2_adds_every_moving_shell_to_one_genuine_surface(self):
        scene = load(SCENES / 'moving_shell_surface.json')
        self.assertEqual(scene.definition_version, 2)
        self.assertEqual(scene.contact_surface, 'all_shells')
        mesh = build(scene)
        deck = starter(scene, mesh)
        body = deck.split('/SURF/SEG/1\n', 1)[1].split('/INTER/TYPE25/1\n', 1)[0].splitlines()[1:]
        records = [tuple(int(line[i:i+10]) for i in range(0, len(line), 10)) for line in body]
        expected = [(e.id, *e.nodes, 0) for e in mesh.wall] + [(e.id, *e.nodes) for e in mesh.patch]
        self.assertEqual(records, expected)
        self.assertEqual(len(records), len(mesh.wall)+len(mesh.patch))
        self.assertEqual(len({row[0] for row in records}), len(records))
        with tempfile.TemporaryDirectory() as temporary:
            record = export(SCENES / 'moving_shell_surface.json', Path(temporary)/'export')
            self.assertEqual(record['schema'], 'robo_dyna.native_contact_scene_export.v2')
            self.assertEqual(record['scene']['contact_surface'], 'all_shells')
            self.assertEqual(record['scene']['velocity_mm_s'], (1000., 0., -10000.))
            self.assertEqual(record['scene']['time_step_cap_s'], 3e-7)
            self.assertNotIn('definition_version', record['scene'])

    def test_surface_changes_do_not_modify_structure_material_or_clock(self):
        wall = load(SCENES / 'fixed_wall_patch_capped.json')
        moving = load(SCENES / 'moving_shell_surface.json')
        self.assertEqual(replace(moving, contact_surface='fixed_wall', definition_version=1), wall)
        self.assertEqual(build(wall), build(moving))
        old = starter(wall, build(wall))
        new = starter(moving, build(moving))
        self.assertEqual(old.split('/SURF/SEG/1')[0], new.split('/SURF/SEG/1')[0])
        self.assertEqual(old.split('/INTER/TYPE25/1')[1], new.split('/INTER/TYPE25/1')[1])

    def test_unknown_implicit_or_mismatched_profile_cannot_export(self):
        original = json.loads((SCENES/'moving_shell_surface.json').read_text())
        with tempfile.TemporaryDirectory() as temporary:
            source = Path(temporary)/'scene.json'
            for fault in range(4):
                data = dict(original)
                if fault == 0: data['contact_surface'] = 'nearest_shells'
                elif fault == 1: data.pop('contact_surface')
                elif fault == 2: data['schema'] = 'robo_dyna.native_contact_scene.v1'
                else: data['schema'] = 'robo_dyna.native_contact_scene.v99'
                source.write_text(json.dumps(data))
                destination = Path(temporary)/f'bad-{fault}'
                with self.assertRaises(ValueError): export(source, destination)
                self.assertFalse(destination.exists())
        scene = load(SCENES/'moving_shell_surface.json')
        with self.assertRaises(ValueError): starter(replace(scene, definition_version=1), build(scene))


if __name__ == '__main__':
    unittest.main()
