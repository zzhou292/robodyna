"""Complete synthetic source coverage and explicit unsupported-mechanics gates."""
from dataclasses import replace
from io import BytesIO
from pathlib import Path
from types import SimpleNamespace
import sys
import struct
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from modelio._legacy import VehicleGeometry,scan_vehicle,FIELDS,sha256
from modelio.source_blocks import scan_metadata,DECLARATION_FAMILIES,ATTACHMENT_FAMILIES
from modelio.spotweld_cards import scan_spotwelds
from modelio.full_shell_source import verify_canonical
from modelio.full_shell_coverage import coverage,tire_exclusions,TIRE_SHELL_PARTS
from modelio.full_shell_declarations import declaration_coverage
from modelio.full_shell_connections import connection_coverage
from modelio.full_shell_materials import material_fields
from modelio.yaris_full_shell import compile_full_shell_scope,write_full_shell_scope
from test_source_assembly import AssemblyFixture
from test_canonical_part_geometry import row


class FullShellScope(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.f=AssemblyFixture(self.temp.name)
        g=VehicleGeometry();summary=scan_vehicle(BytesIO(self.f.raw),'yaris-coarse-v1l.key',g);g.validate()
        index=scan_metadata(BytesIO(self.f.raw),'yaris-coarse-v1l.key',families=DECLARATION_FAMILIES|ATTACHMENT_FAMILIES)
        self.source=SimpleNamespace(geometry=g,index=index,units=self.f.declarations.units,
            welds=scan_spotwelds(BytesIO(self.f.raw),'yaris-coarse-v1l.key'),
            source_files={'yaris-coarse-v1l.key':summary})

    def tearDown(self):self.temp.cleanup()

    def test_every_shell_and_nonshell_interface_is_accounted(self):
        g=self.source.geometry;c,rows,parts,nodes=coverage(g,set())
        self.assertEqual(c['retained'],dict(shells=4,q4=1,t3=3,parts=2,nodes=9))
        self.assertEqual(c['excluded'],dict(shells=0,q4=0,t3=0,parts=0))
        self.assertEqual(parts,{17,18});self.assertEqual(c['non_shell']['solids']['touching_retained_shell_nodes'],1)
        self.assertEqual(c['non_shell']['beams']['touching_retained_shell_nodes'],1)
        self.assertIsNone(c['non_shell']['solids']['mass_removed_kg'])
        a=connection_coverage(self.source,parts,nodes,rows)
        self.assertEqual(a['classifications']['nodal_rigid'],{'internal':1,'cross_boundary':1})
        self.assertEqual(a['classifications']['spotweld'],{'cross_boundary':1,'internal':1})
        incidence={r['source_node_id']:r for r in a['interface_node_part_incidence']}
        self.assertEqual(incidence[6]['parts_by_family'],dict(beams=[19],shells=[18],solids=[20]))
        self.assertFalse(a['tied_contacts'][0]['pairing_qualified']);self.assertFalse(a['full_load_path_closure_qualified'])
        self.assertEqual(a['tied_contacts'][0]['slave_part_geometry'][0]['counts']['shells'],2)
        d=declaration_coverage(self.source,rows,parts,set())
        self.assertEqual(sum(r['shells'] for r in d['retained_shell_families']),4)

    def test_unsupported_late_material_remains_selected_and_missing_reference_fails(self):
        c,rows,parts,nodes=coverage(self.source.geometry,set());index=self.source.index
        old=index.one('material',38)
        index.entries[('material',38)]=[replace(old,keyword='*MAT_RIGID')]
        d=declaration_coverage(self.source,rows,parts,set());last=next(p for p in d['parts'] if p['source_part_id']==18)
        self.assertTrue(last['retained_shell_part']);self.assertEqual(last['counts']['shells'],2)
        self.assertEqual(last['disposition'],'unsupported_declaration_retained')
        self.assertTrue(last['existing_adapter_rejections'])
        del index.entries[('material',38)]
        with self.assertRaisesRegex(ValueError,'unresolved material'):declaration_coverage(self.source,rows,parts,set())

    def test_exclusion_is_exact_and_rim_names_never_select_it(self):
        g=self.source.geometry
        self.assertEqual(tire_exclusions(g,'retain_all'),set())
        with self.assertRaisesRegex(ValueError,'tire-shell identity'):tire_exclusions(g,'omit_original_tire_shells')
        for pid,title in TIRE_SHELL_PARTS.items():
            g.parts[pid]=dict(title=title,source_section_id=pid,source_material_id=pid)
            g.sections[pid]=dict(keyword='*SECTION_SHELL');g.materials[pid]=dict(keyword='*MAT_ELASTIC')
        self.assertEqual(tire_exclusions(g,'omit_original_tire_shells'),set(TIRE_SHELL_PARTS))
        self.assertEqual(len(TIRE_SHELL_PARTS),8)
        g.parts[2000211]['title']='410_tirefrontrim'
        with self.assertRaisesRegex(ValueError,'tire-shell identity'):tire_exclusions(g,'omit_original_tire_shells')
        c,_,_,nodes=coverage(g,{18})
        self.assertEqual(c['retained']['shells'],2);self.assertEqual(c['excluded']['shells'],2)
        self.assertNotIn(16,nodes)

    def test_canonical_late_rehashed_nonshell_array_cannot_change(self):
        g=self.source.geometry;m=self.f.manifest
        counts=dict(nodes=len(g.node_ids),parts=len(g.parts),**{n:len(v)//len(FIELDS[n]) for n,v in g.elements.items()})
        m['counts']=counts
        for name in ('parts','sections','materials'):m[name]=[getattr(g,name)[k] for k in sorted(getattr(g,name))]
        reference=dict(self.f.reference,expected_vehicle_geometry=counts)
        verify_canonical(self.f.directory,m,g,self.source.source_files,reference)
        name='solids_blank_masks';info=m['arrays'][name];p=self.f.directory/info['file'];raw=p.read_bytes()
        changed=raw[:-1]+bytes([raw[-1]^1]);p.write_bytes(changed);info['sha256']=sha256(changed)
        with self.assertRaisesRegex(ValueError,'differs from original source'):
            verify_canonical(self.f.directory,m,g,self.source.source_files,reference)

    def test_coordinate_signed_zero_bit_is_preserved_after_rehash(self):
        g=self.source.geometry;m=self.f.manifest
        counts=dict(nodes=len(g.node_ids),parts=len(g.parts),**{n:len(v)//len(FIELDS[n]) for n,v in g.elements.items()})
        m['counts']=counts
        for name in ('parts','sections','materials'):m[name]=[getattr(g,name)[k] for k in sorted(getattr(g,name))]
        reference=dict(self.f.reference,expected_vehicle_geometry=counts)
        p=self.f.directory/m['arrays']['node_positions']['file'];raw=bytearray(p.read_bytes())
        index=max(i for i,v in enumerate(g.positions) if v==0)
        raw[8*index+7]^=128
        self.assertEqual(struct.unpack_from('<d',raw,8*index)[0],g.positions[index])
        p.write_bytes(raw);m['arrays']['node_positions']['sha256']=sha256(raw)
        with self.assertRaisesRegex(ValueError,'node_positions'):
            verify_canonical(self.f.directory,m,g,self.source.source_files,reference)

    def test_material_census_distinguishes_table_rate_and_bilinear_fields(self):
        block=self.source.index.one('material',38)
        fields=material_fields(block)
        self.assertEqual(fields['hardening_declaration'],'positive_curve_reference')
        self.assertEqual(fields['strict_positive_admission_failures'],[])
        cards=list(block.cards);cards[1]=replace(cards[1],text=row(0,0,48,None,'0.0'))
        altered=replace(block,cards=tuple(cards));fields=material_fields(altered)
        self.assertEqual(fields['strict_positive_admission_failures'],['c','p'])
        self.assertEqual(fields['hardening_declaration'],'positive_curve_reference')
        self.source.index.entries[('material',38)]=[altered]
        _,rows,parts,_=coverage(self.source.geometry,set())
        census=declaration_coverage(self.source,rows,parts,set())['material_dispositions']
        failed=[r for r in census['mat024_source_modes'] if r['strict_positive_admission_failures']]
        self.assertEqual((failed[0]['parts'],failed[0]['shells']),(1,2))
        cards[1]=replace(cards[1],text=row(0,0,0,None,'0.0'))
        fields=material_fields(replace(block,cards=tuple(cards)))
        self.assertEqual(fields['hardening_declaration'],'SIGY_ETAN_fields_without_positive_LCSS')
        self.assertEqual(fields['source_values']['sigy'],180)
        cards[0]=replace(cards[0],text=row(38,'7.8900E-9','2.0000E+5',.3,180,10,2.5))
        self.source.index.entries[('material',38)]=[replace(block,cards=tuple(cards))]
        census=declaration_coverage(self.source,rows,parts,set())['material_dispositions']
        failure=next(r for r in census['mat024_supplied_optional_fields'] if r['field']=='fail')
        self.assertEqual((failure['source_value'],failure['shells']),(2.5,2))

    def test_policy_preflight_and_create_only_bounded_publication(self):
        with patch('modelio.yaris_full_shell.load_full_shell_source') as load:
            with self.assertRaisesRegex(ValueError,'tire policy'):compile_full_shell_scope('missing','missing','guess_from_title')
            load.assert_not_called()
        path=Path(self.temp.name)/'scope.json'
        report=dict(schema='robo-dyna.full-shell-scope.v1',simulation_ready=False)
        write_full_shell_scope(path,report);before=path.read_bytes()
        with self.assertRaises(FileExistsError):write_full_shell_scope(path,report)
        self.assertEqual(path.read_bytes(),before)
        with patch('modelio.yaris_full_shell.MAX_SCOPE_BYTES',4):
            with self.assertRaisesRegex(ValueError,'byte cap'):write_full_shell_scope(Path(self.temp.name)/'missing.json',report)
        self.assertFalse((Path(self.temp.name)/'missing.json').exists())


if __name__=='__main__':unittest.main()
