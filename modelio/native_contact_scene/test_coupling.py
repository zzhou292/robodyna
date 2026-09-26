"""Declared rigid source inputs only; no Starter or trajectory claim."""
from dataclasses import replace
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from .coupling import reference_rigid_body
from .definition import load
from .export import export
from .mesh import build
from .native import starter

ROOT = Path(__file__).resolve().parents[2]
SCENES = ROOT/'benchmarks/native_contact_scene'


class CoupledDeclarationTests(unittest.TestCase):
    def test_qualified_v2_export_is_byte_identical(self):
        expected = json.loads(Path(__file__).with_name('v2-export-hashes.json').read_text())['files']
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary)/'export'
            export(SCENES/'moving_shell_surface.json', path)
            self.assertEqual({f.name: hashlib.sha256(f.read_bytes()).hexdigest()
                              for f in path.iterdir()}, expected)

    def test_rigid_primary_is_separate_from_complete_physical_mesh(self):
        scene = load(SCENES/'rigid_patch_capped.json')
        mesh = build(scene)
        body = reference_rigid_body(scene, mesh)
        self.assertEqual(len(mesh.nodes), 18)
        self.assertEqual((len(mesh.wall), len(mesh.patch)), (8, 4))
        self.assertEqual(body['primary']['id'], 19)
        self.assertNotIn(19, mesh.wall_nodes+mesh.patch_nodes)
        self.assertEqual(body['member_node_ids'], mesh.patch_nodes)
        # Original GetCentroid(PartRead) first-encounter order, not NID sort.
        self.assertEqual(body['centroid_node_order'], (10,11,14,13,12,15,17,16,18))
        points = {n.id: n.xyz_mm for n in mesh.nodes}
        center = [0., 0., 0.]
        for node in body['centroid_node_order']:
            for axis in range(3): center[axis] += points[node][axis]
        self.assertEqual(body['primary']['xyz_mm'], tuple(x/9 for x in center))
        self.assertEqual(body['converter_mass_tonne'], 1e-20)
        self.assertEqual(body['converter_inertia_tonne_mm2'], (1e-20,)*3)
        self.assertEqual(body['primary_velocity_mm_s'], scene.velocity_mm_s)

    def test_deck_and_manifest_retain_exact_declared_group_controls(self):
        scene = load(SCENES/'rigid_patch_capped.json')
        mesh = build(scene)
        deck = starter(scene, mesh)
        block = deck.split('/RBODY/1\n', 1)[1].split('/BCS/1', 1)[0].splitlines()
        card = block[1]
        self.assertEqual(tuple(int(card[i:i+10]) for i in (0,10,20,30)), (19,0,0,2))
        self.assertEqual(float(card[40:60]), 1e-20)
        self.assertEqual(tuple(int(card[i:i+10]) for i in (60,70,80,90)), (2,0,1,0))
        self.assertEqual(tuple(float(block[2][i:i+20]) for i in (0,20,40)), (1e-20,)*3)
        self.assertEqual(tuple(float(block[3][i:i+20]) for i in (0,20,40)), (0.,)*3)
        self.assertEqual(tuple(int(block[4][i:i+10]) for i in (0,10,20)), (0,2,0))
        surface = deck.split('/SURF/SEG/1\n', 1)[1].split('/INTER/TYPE25/1', 1)[0]
        self.assertEqual(len(surface.splitlines())-1, 12)
        for row in surface.splitlines()[1:]:
            self.assertNotIn(19, [int(row[i:i+10]) for i in range(10,len(row),10)])
        with tempfile.TemporaryDirectory() as temporary:
            record = export(SCENES/'rigid_patch_capped.json', Path(temporary)/'export')
            self.assertEqual(record['schema'], 'robo_dyna.native_contact_scene_export.v3')
            self.assertEqual(record['scene']['coupling']['kind'], 'rigid_patch')
            self.assertEqual(record['reference_rigid_body']['primary']['id'], 19)
            self.assertEqual(len(record['mesh']['nodes']), 18)

    def test_unknown_or_implicit_coupling_rejects_before_destination_creation(self):
        original = json.loads((SCENES/'rigid_patch_capped.json').read_text())
        with tempfile.TemporaryDirectory() as temporary:
            source = Path(temporary)/'source.json'
            for fault in range(6):
                data = json.loads(json.dumps(original))
                if fault == 0: data.pop('coupling')
                if fault == 1: data['schema'] = 'robo_dyna.native_contact_scene.v2'
                if fault == 2: data['coupling']['kind'] = 'cin_patch'
                if fault == 3: data['coupling']['tied_secondary_removal'] = 3
                if fault == 4: data['coupling']['center_of_gravity'] = True
                if fault == 5: data['coupling']['primary_velocity'] = 'unspecified'
                source.write_text(json.dumps(data))
                destination = Path(temporary)/f'bad-{fault}'
                with self.assertRaises(ValueError): export(source, destination)
                self.assertFalse(destination.exists())
        scene = load(SCENES/'rigid_patch_capped.json')
        with self.assertRaises(ValueError): starter(replace(scene, coupling=None), build(scene))


if __name__ == '__main__':
    unittest.main()
