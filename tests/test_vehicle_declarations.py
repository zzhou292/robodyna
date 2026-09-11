from dataclasses import asdict
from io import BytesIO
from pathlib import Path
import copy
import tempfile
import unittest
from unittest.mock import patch
from test_source_assembly import AssemblyFixture
from test_canonical_part_geometry import row
from modelio.source_blocks import scan_declarations
from modelio.declarations import UnitSystem
from modelio.vehicle_declarations import compile_vehicle_declarations
from modelio.vehicle_declaration_io import write_vehicle_declarations

class VehicleDeclarations(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.f=AssemblyFixture(self.temp.name)
    def tearDown(self):self.temp.cleanup()
    def inputs(self):
        index=scan_declarations(BytesIO(self.f.raw),'yaris-coarse-v1l.key')
        tables={k:[] for k in ('material','section','curve')}
        for (kind,identity),blocks in index.entries.items():
            if kind in tables:tables[kind].append(dict(identity=identity,sha256=blocks[0].sha256))
        parts=[dict(source_part_id=p,source_material_id=p+20,source_section_id=p+10,
                    source_part_sha256=index.one('part',p).sha256,retained_shell_part=True,counts=dict(shells=2)) for p in (17,18)]
        scope=dict(schema='robo-dyna.full-shell-scope.v1',source=dict(member_sha256=index.sha256),
                   selected_shell_part_ids=[17,18],declarations=dict(parts=parts,tables=tables),coverage=dict(retained=dict(shells=4)))
        return index,scope
    def compile(self):
        i,s=self.inputs();return compile_vehicle_declarations(i,s,UnitSystem('t','mm','s',1000,.001,1),{})
    def test_same_typed_declarations_without_geometry_or_owner(self):
        actual=self.compile();v3=self.f.compile_assembly(material_policy='layered_law1_or_law44')
        self.assertEqual(actual['supported_declarations']['declarations'],v3['declarations'])
        self.assertEqual(actual['counts'],dict(parts=2,shells=4,supported_parts=2,supported_shells=4))
        self.assertNotIn('geometry',actual);self.assertFalse(actual['native_startup_qualified'])
    def test_unresolved_material_keeps_complete_original_blocks_and_shells(self):
        old=row(38,'7.8900E-9','2.0000E+5',.3,180).encode()
        new=row(38,'7.8900E-9','2.0000E+5',.3,180,None,1).encode()
        self.f.rebuild(self.f.base_raw.replace(old,new));result=self.compile()
        self.assertEqual(result['counts']['shells'],4);self.assertEqual(result['counts']['supported_shells'],2)
        blocked=result['parts'][1];self.assertEqual(blocked['status'],'unresolved')
        self.assertEqual(set(blocked['source_blocks']),{'part','section','material'})
        self.assertIn('fail',blocked['obligations'][0]['reason'])
    def test_late_source_association_missing_material_and_coverage_reject_then_retry(self):
        index,scope=self.inputs();saved=copy.deepcopy(scope)
        scope['declarations']['parts'][-1]['source_part_sha256']='0'*64
        with self.assertRaisesRegex(ValueError,'block changed'):compile_vehicle_declarations(index,scope,UnitSystem('t','mm','s',1000,.001,1),{})
        scope=saved;index.entries.pop(('material',38))
        with self.assertRaisesRegex(ValueError,'unresolved material'):compile_vehicle_declarations(index,scope,UnitSystem('t','mm','s',1000,.001,1),{})
        self.assertEqual(self.compile()['counts']['supported_parts'],2)
    def test_create_only_and_byte_cap_precede_publication(self):
        p=Path(self.temp.name)/'declarations.json';report=self.compile()
        with patch('modelio.vehicle_declaration_io.MAX_BYTES',16):
            with self.assertRaises(ValueError):write_vehicle_declarations(p,report)
        self.assertFalse(p.exists());write_vehicle_declarations(p,report);before=p.read_bytes()
        with self.assertRaises(FileExistsError):write_vehicle_declarations(p,report)
        self.assertEqual(p.read_bytes(),before)

if __name__=='__main__':unittest.main()
