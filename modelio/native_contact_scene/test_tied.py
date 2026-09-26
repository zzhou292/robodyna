"""Source/card tests only. No reader classification, CIN operator or trajectory."""
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
from .tied import reference_tied_interface

ROOT = Path(__file__).resolve().parents[2]
SCENES = ROOT/'benchmarks/native_contact_scene'


class TiedDeclarationTests(unittest.TestCase):
    def test_qualified_rigid_v3_bytes_remain_identical(self):
        expected = json.loads(Path(__file__).with_name('v3-export-hashes.json').read_text())['files']
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary)/'export'
            export(SCENES/'rigid_patch_capped.json', path)
            self.assertEqual({f.name: hashlib.sha256(f.read_bytes()).hexdigest() for f in path.iterdir()}, expected)

    def test_complete_seventeen_node_geometry_has_real_disjoint_shell_patches(self):
        scene = load(SCENES/'tied_patch_capped.json')
        mesh = build(scene)
        self.assertEqual(tuple(n.id for n in mesh.nodes), tuple(range(1,18)))
        self.assertEqual((len(mesh.wall),len(mesh.patch)),(8,2))
        self.assertEqual(tuple((e.id,e.part,e.nodes) for e in mesh.patch),
                         ((9,2,(10,11,13,12)),(10,3,(14,15,17,16))))
        self.assertEqual(mesh.patch_nodes,tuple(range(10,18)))
        self.assertFalse(set(mesh.wall_nodes)&set(mesh.patch_nodes))
        record = reference_tied_interface(scene,mesh)
        self.assertEqual(record['master_node_ids'],(10,11,13,12))
        self.assertEqual(record['secondary_node_ids'],(14,15,16,17))
        points = {n.id:n.xyz_mm for n in mesh.nodes}
        self.assertEqual(tuple(sum(points[n][k] for n in record['secondary_node_ids'])/4 for k in range(3)),
                         (-1.25,1.25,2.))
        self.assertNotIn('weights',record)
        self.assertNotIn('irupt',record)

    def test_exact_2024_type2_card_and_all_shell_type25_scope(self):
        scene = load(SCENES/'tied_patch_capped.json');mesh = build(scene);deck = starter(scene,mesh)
        self.assertNotIn('/RBODY',deck)
        block = deck.split('/INTER/TYPE2/2\n',1)[1].split('/END',1)[0].splitlines()
        self.assertEqual(len(block),3)
        self.assertEqual(tuple(int(block[1][i:i+10]) for i in range(0,70,10)),(3,2,2,28,0,0,1))
        self.assertEqual(block[1][70:80],' '*10)
        self.assertEqual(float(block[1][80:100]),0.)
        self.assertEqual(tuple(float(block[2][i:i+20]) for i in (0,20)),(1.,.05))
        self.assertEqual(block[2][40:60],' '*20)
        self.assertEqual(int(block[2][60:70]),2)
        main = deck.split('/SURF/SEG/2\n',1)[1].split('/INTER/TYPE2/2',1)[0].splitlines()
        self.assertEqual(len(main),2)
        self.assertEqual(tuple(int(main[1][i:i+10]) for i in range(0,50,10)),(9,10,11,13,12))
        surface = deck.split('/SURF/SEG/1\n',1)[1].split('/INTER/TYPE25/1',1)[0].splitlines()
        self.assertEqual(len(surface)-1,10)
        type25 = deck.split('/INTER/TYPE25/1\n',1)[1].splitlines()[1]
        self.assertEqual(int(type25[50:60]),1) # Actual Irem_i2 input is retained.
        self.assertIn('/SHELL/3\n',deck)

    def test_export_has_no_auxiliary_or_prescribed_contact_or_cin_state(self):
        with tempfile.TemporaryDirectory() as temporary:
            destination = Path(temporary)/'export'
            record = export(SCENES/'tied_patch_capped.json',destination)
            self.assertEqual(record['schema'],'robo_dyna.native_contact_scene_export.v4')
            self.assertEqual(len(record['mesh']['nodes']),17)
            self.assertEqual(len(record['mesh']['wall'])+len(record['mesh']['patch']),10)
            self.assertNotIn('reference_rigid_body',record)
            self.assertEqual(record['reference_tied_interface']['tied_secondary_removal'],1)
            self.assertNotIn('mass',record['reference_tied_interface'])
            self.assertNotIn('weights',record['reference_tied_interface'])
            for entry in record['files']:
                data=(destination/entry['path']).read_bytes()
                self.assertEqual(len(data),entry['bytes'])
                self.assertEqual(hashlib.sha256(data).hexdigest(),entry['sha256'])
            with self.assertRaises(FileExistsError):export(SCENES/'tied_patch_capped.json',destination)

    def test_unsupported_controls_or_geometry_reject_before_creation(self):
        original=json.loads((SCENES/'tied_patch_capped.json').read_text())
        with tempfile.TemporaryDirectory() as temporary:
            source=Path(temporary)/'source.json'
            for fault in range(9):
                data=json.loads(json.dumps(original))
                if fault==0:data.pop('coupling')
                if fault==1:data['coupling']['tied_secondary_removal']=3
                if fault==2:data['coupling']['spotflag']=27
                if fault==3:data['coupling']['ignore']=True
                if fault==4:data['coupling']['dependent']['x_mm']=[-5,1]
                if fault==5:data['coupling']['dependent']['z_mm']=2.1
                if fault==6:data['patch']['x_mm']=[-5,0,5]
                if fault==7:data['coupling']['dependent']['dz_dx']=.01
                if fault==8:data['coupling']['weights']=[.25]*4
                source.write_text(json.dumps(data));destination=Path(temporary)/f'bad-{fault}'
                with self.assertRaises(ValueError):export(source,destination)
                self.assertFalse(destination.exists())
        scene=load(SCENES/'tied_patch_capped.json')
        with self.assertRaises(ValueError):starter(replace(scene,coupling=None),build(scene))


if __name__=='__main__':unittest.main()
