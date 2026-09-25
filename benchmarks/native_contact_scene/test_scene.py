"""Owning definition/export checks; these do not qualify native solver behavior."""
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from . import cards, definition, mesh, native
from .__main__ import export

SOURCE = Path(__file__).with_name('fixed_wall_patch.json')


class NativeContactScene(unittest.TestCase):
    def test_complete_oriented_mesh_and_physical_initial_state(self):
        scene = definition.load(SOURCE)
        model = mesh.build(scene)
        self.assertEqual((len(model.nodes),len(model.wall),len(model.patch)),(18,8,4))
        self.assertEqual(len(set(n.id for n in model.nodes)),18)
        self.assertFalse(set(model.wall_nodes) & set(model.patch_nodes))
        positions = {n.id:n.xyz_mm for n in model.nodes}
        area = 0.
        for face in model.wall:
            a,b,c = (positions[n] for n in face.nodes)
            twice = (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])
            self.assertGreater(twice,0)
            area += twice/2
        self.assertEqual(area,1600.)
        self.assertAlmostEqual(min(positions[n][2] for n in model.patch_nodes),1.9)
        self.assertEqual(scene.velocity_mm_s,(1000.,0.,-10000.))

    def test_native_fields_preserve_intended_source_branch_and_numbers(self):
        scene = definition.load(SOURCE)
        lines = native.starter(scene,mesh.build(scene)).splitlines()
        self.assertEqual(lines[0],'#RADIOSS STARTER')
        self.assertEqual(lines[-1],'/END')
        law = lines.index('/MAT/LAW44/1')
        self.assertEqual([float(lines[law+4][i:i+20]) for i in range(0,100,20)],
                         [250.,1000.,1.,0.,1e30])
        self.assertEqual(float(lines[law+5][60:80]),10000.)
        prop = lines.index('/PROP/TYPE1/1')
        self.assertEqual([int(lines[prop+2][i:i+10]) for i in range(0,50,10)],[24,2,1,2,0])
        self.assertEqual(int(lines[prop+4][:10]),3)
        self.assertEqual(float(lines[prop+4][20:40]),scene.thickness_mm)
        self.assertEqual(int(lines[prop+4][70:80]),1)
        contact = lines.index('/INTER/TYPE25/1')
        row = lines[contact+2]
        self.assertEqual([int(row[i:i+10]) for i in (0,10,20,30,40,50,70,80,90)],
                         [1,0,4,0,2,1,1,1000,0])
        self.assertEqual(int(lines[contact+3][:10]),2)
        self.assertEqual(float(lines[contact+9]),-.001)
        self.assertIn('/DT/NODA/STOP',native.engine(scene))
        self.assertNotIn('/CST',native.engine(scene))

    def test_field_writers_reject_loss_and_nonfinite(self):
        for value in (True, float('nan'),float('inf'),1.2345678901234568e-120):
            with self.assertRaises(ValueError): cards.real(value)
        with self.assertRaises(ValueError): cards.integer(10000000000)
        for value in (7.85e-9,210000.,.3,-.001,-0.,5./6.):
            text = cards.real(value)
            self.assertEqual(len(text),20)
            self.assertEqual(float(text).hex(),value.hex())

    def test_invalid_or_ambiguous_declarations_reject(self):
        base = json.loads(SOURCE.read_text())
        with tempfile.TemporaryDirectory() as name:
            path = Path(name)/'case.json'
            variants=[]
            for modify in (lambda x:x['run'].update(nodal_scale=True),
                           lambda x:x['patch'].update(z_mm=.5),
                           lambda x:x['wall'].update(x_mm=[0,0]),
                           lambda x:x['material'].update(young_n_mm2=float('inf')),
                           lambda x:x.update(extra='not supported')):
                value=json.loads(json.dumps(base));modify(value);variants.append(json.dumps(value))
            variants.append(SOURCE.read_text().replace('"thickness_mm": 1','"thickness_mm": 1, "thickness_mm": 2'))
            for text in variants:
                path.write_text(text)
                with self.assertRaises(ValueError): definition.load(path)

    def test_create_only_export_pins_actual_emitted_bytes(self):
        with tempfile.TemporaryDirectory() as name:
            out=Path(name)/'export'
            record=export(SOURCE,out)
            self.assertEqual(record['source_sha256'],hashlib.sha256(SOURCE.read_bytes()).hexdigest())
            for pin in record['files']:
                data=(out/pin['path']).read_bytes()
                self.assertEqual(len(data),pin['bytes'])
                self.assertEqual(hashlib.sha256(data).hexdigest(),pin['sha256'])
            original=(out/'contact_scene_0000.rad').read_bytes()
            with self.assertRaises(FileExistsError): export(SOURCE,out)
            self.assertEqual((out/'contact_scene_0000.rad').read_bytes(),original)

    def test_invalid_export_creates_no_partial_destination(self):
        with tempfile.TemporaryDirectory() as name:
            path=Path(name)/'invalid.json';path.write_text('{}')
            out=Path(name)/'absent'
            with self.assertRaises(ValueError): export(path,out)
            self.assertFalse(out.exists())


if __name__ == '__main__':
    unittest.main()
