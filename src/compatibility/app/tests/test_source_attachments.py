"""Authored fixed-width namespace/one-hop oracles; no solver or original asset."""
import argparse
from dataclasses import asdict, FrozenInstanceError
from io import BytesIO
import json
import os
from pathlib import Path
import struct
import sys
import tempfile
import unittest
from unittest.mock import patch
import zipfile

APP = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(APP))
from modelio._legacy import VehicleGeometry, scan_vehicle, FIELDS, file_sha256, sha256
from modelio.source_blocks import scan_metadata, scan_declarations, DECLARATION_FAMILIES, ATTACHMENT_FAMILIES
from modelio.attachment_cards import parse_list_set, parse_nodal_rigid
from modelio.attachments import compile_attachment_scope, AttachmentLimits
from modelio.canonical_geometry import load_array
from modelio import yaris_part
from test_canonical_part_geometry import TinyCanonical, row
import import_yaris_vehicle as vehicle

QUADRATURE_PATH = os.environ.get('ROBO_DYNA_SURFACE_QUADRATURE')
FAMILIES = DECLARATION_FAMILIES | ATTACHMENT_FAMILIES
README = '\n'.join(('Mass **t**: metric ton (1,000 kg)', 'Length **mm**: millimeter',
                    'Force **N**: newton', 'Time **s**: second'))


class AttachedCanonical(TinyCanonical):
    def __init__(self, directory):
        super().__init__(directory)
        extra = []
        for pid in (18, 19, 20):
            extra += ['*PART', f'neighbor {pid}', row(pid, 27, 37)]
        extra += ['*NODE']
        for nid in range(6, 17):
            extra.append(row(nid, width=8) + row(nid*1000, (nid % 3)*1000, (nid % 2)*1000, width=16))
        extra += ['*ELEMENT_SHELL', row(201, 18, 6, 7, 8, 8, width=8),
                  row(202, 18, 7, 8, 16, 16, width=8),
                  '*ELEMENT_SOLID', row(301, 20, 6, 11, 12, 13, 14, 15, 15, 15, width=8),
                  '*ELEMENT_BEAM', row(401, 19, 9, 10, 6, width=8), row(402, 19, 6, 9, 10, width=8),
                  '*CONSTRAINED_NODAL_RIGID_BODY', row(17, None, 17),
                  '*SET_NODE_LIST_TITLE', 'literal group with source title', row(17), row(2, 6),
                  '*CONSTRAINED_NODAL_RIGID_BODY', row(18, None, 18),
                  '*SET_NODE_LIST', row(18), row(7, 9),
                  '*SET_PART_LIST', row(17), row(18),
                  '*SET_PART_LIST_TITLE', 'candidate master, no generated pairs', row(18), row(17),
                  '*CONTACT_TIED_SHELL_EDGE_TO_SURFACE', row(17, 18, 2, 2), '', '', '*END']
        self.base_raw = self.raw.rsplit(b'*END', 1)[0] + ('\n'.join(extra)+'\n').encode('ascii')
        self.rebuild(self.base_raw)

    def rebuild(self, raw):
        self.raw = raw
        parsed = VehicleGeometry()
        source = scan_vehicle(BytesIO(raw), 'yaris-coarse-v1l.key', parsed)
        parsed.validate()
        with zipfile.ZipFile(self.archive, 'w') as archive:
            archive.writestr('fixture/yaris-coarse-v1l.key', raw)
            archive.writestr('fixture/README.md', README)
        self.reference = dict(model='independently authored attachment fixture', archive_member_prefix='fixture/',
                              archive_sha256=file_sha256(self.archive),
                              files={'yaris-coarse-v1l.key': {'sha256': sha256(raw)}})
        self.manifest['source_archive']['sha256'] = self.reference['archive_sha256']
        self.manifest['source_files']['yaris-coarse-v1l.key'] = source
        self.manifest['parts'] = list(parsed.parts.values())
        arrays = {'node_ids': (parsed.node_ids, 1), 'node_positions': (parsed.positions, 3),
                  'node_codes': (parsed.node_codes, 2), 'node_blank_masks': (parsed.node_masks, 1),
                  'node_source_lines': (parsed.node_lines, 1)}
        for family in FIELDS:
            arrays.update({family+'_records': (parsed.elements[family], len(FIELDS[family])),
                           family+'_node_indices': (parsed.connectivity[family], 2 if family=='beams' else len(FIELDS[family])-2),
                           family+'_source_lines': (parsed.element_lines[family], 1),
                           family+'_blank_masks': (parsed.element_masks[family], 1)})
        for name, (values, columns) in arrays.items():
            filename = name+'.bin'
            self.manifest['arrays'][name] = dict(file=filename, **vehicle.write_array(self.directory/filename, values, columns, None))
        self.save()

    def index(self, raw=None):
        return scan_metadata(BytesIO(self.raw if raw is None else raw), 'yaris-coarse-v1l.key', families=FAMILIES)

    def compile(self, **kwargs):
        return compile_attachment_scope(self.index(), self.load(), self.directory, self.archive, self.reference, **kwargs)


class SourceAttachments(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.f = AttachedCanonical(self.temp.name)

    def tearDown(self):
        self.temp.cleanup()

    def test_literal_namespace_links_exact_one_hop_and_immutable_scope(self):
        result = self.f.compile()
        self.assertEqual(result.source_instance, (self.f.reference['archive_sha256'], 'fixture/yaris-coarse-v1l.key'))
        self.assertEqual(len(result.rigid_groups), 1)
        g = result.rigid_groups[0]
        self.assertEqual((g.rigid.identity, g.node_set.identity), (17, 17))
        self.assertEqual(g.selected_node_ids, (2,)); self.assertEqual(g.external_node_ids, (6,))
        self.assertEqual(result.neighboring_part_ids, (18, 19, 20))
        self.assertEqual([n.source_id for n in result.incidence.nodes], [1, 2, 3, 4, 5, 6])
        self.assertEqual({e.source_id for e in result.incidence.elements}, {101, 102, 201, 301, 402})
        self.assertNotIn(401, {e.source_id for e in result.incidence.elements})  # N3 is orientation, not endpoint.
        self.assertNotIn(202, {e.source_id for e in result.incidence.elements})  # No neighbor/transitive expansion.
        self.assertEqual(result.incidence.elements[-1].raw_record, (402, 19, 6, 9, 10, 0, 0, 0, 0, 0))
        self.assertTrue(result.literal_one_hop_references_validated)
        for flag in ('simulation_ready', 'full_attachment_closure_qualified', 'attachment_mechanics_implemented',
                     'tied_pairing_qualified', 'cut_boundary_supplied'):
            self.assertIs(getattr(result, flag), False)
        with self.assertRaises(FrozenInstanceError):
            result.selected_part_id = 18
        with self.assertRaises(FrozenInstanceError):
            g.rigid.identity = 18

    def test_tied_master_membership_is_separate_and_contacts_have_location_identity(self):
        raw = self.f.raw.replace(b'*END', ('*CONTACT_TIED_SHELL_EDGE_TO_SURFACE\n'+row(17,18,2,2)+'\n\n\n*END').encode())
        self.f.rebuild(raw)
        result = self.f.compile()
        self.assertEqual(len(result.tied_candidates), 2)
        self.assertEqual(len({c.contact.source_identity for c in result.tied_candidates}), 2)
        for c in result.tied_candidates:
            self.assertFalse(c.selected_part_in_slave_set)
            self.assertTrue(c.selected_part_in_master_set)
            self.assertFalse(c.actual_pairing_qualified)
            self.assertEqual([r.raw_text for r in c.contact.cards[1:]], ['', ''])
        self.assertEqual(len(result.rigid_groups), 1)

    def test_optional_rigid_cards_headers_blanks_and_title_are_retained_unsupported(self):
        old = '*CONSTRAINED_NODAL_RIGID_BODY\n'+row(17,None,17)
        new = '*CONSTRAINED_NODAL_RIGID_BODY_TITLE\noptional source title\n'+row(17,5,17,6,1,2,3)+'\n'+row('1.25','unparsed')+'\n'
        self.f.rebuild(self.f.raw.replace(old.encode(), new.encode()))
        g = self.f.compile().rigid_groups[0]
        self.assertEqual(g.rigid.title, 'optional source title')
        self.assertEqual(len(g.rigid.cards), 3)
        self.assertEqual(g.rigid.cards[1].fields[1].strip(), 'unparsed')
        self.assertEqual(g.rigid.cards[2].blank_mask, 255)
        self.assertEqual(g.rigid.cards[0].fields[1].strip(), '5')
        self.assertEqual(len(g.rigid.unsupported), 3)
        self.assertIn(new, g.rigid.source.raw_text)
        self.assertEqual(g.node_set.title, 'literal group with source title')
        self.assertEqual(g.node_set.cards[0].blank_mask, 254)

    def test_late_duplicate_namespace_entries_and_unsupported_variants_fail(self):
        for extra, family, sid in [('*SET_NODE_LIST\n'+row(17)+'\n'+row(3), 'node_set',17),
                                   ('*SET_NODE_GENERAL\n'+row(17)+'\nALL', 'node_set',17),
                                   ('*CONSTRAINED_NODAL_RIGID_BODY\n'+row(17,None,18), 'nodal_rigid',17)]:
            raw = self.f.raw.replace(b'*END', (extra+'\n*END').encode())
            with self.assertRaisesRegex(ValueError, 'duplicate'):
                self.f.index(raw).one(family, sid)
        with self.assertRaisesRegex(ValueError, 'keyword'):
            parse_nodal_rigid(self.f.index(self.f.raw.replace(b'*CONSTRAINED_NODAL_RIGID_BODY\n',
                                                              b'*CONSTRAINED_NODAL_RIGID_BODY_INERTIA\n',1)).one('nodal_rigid',17))

    def test_referenced_general_additive_cycles_and_missing_set_fail(self):
        for operator in ('*SET_NODE_GENERAL', '*SET_NODE_ADD'):
            raw = self.f.raw.replace(b'*SET_NODE_LIST_TITLE\nliteral group with source title', operator.encode())
            if operator == '*SET_NODE_ADD':
                raw = raw.replace(row(2,6).encode(), row(17).encode())  # Explicit self-cycle; never expanded.
            self.f.rebuild(raw)
            with self.assertRaisesRegex(ValueError, 'unsupported set operator'):
                self.f.compile()
            self.f.rebuild(self.f.base_raw)
        self.f.rebuild(self.f.raw.replace(row(17,None,17).encode(),row(17,None,999).encode()))
        with self.assertRaisesRegex(ValueError, 'unresolved node_set'):
            self.f.compile()

    def test_duplicate_missing_and_invalid_members_fail_without_output(self):
        for value, message in ((row(2,2),'duplicate set member'), (row(2,999),'incident node'),
                               (row(2,0),'set member')):
            self.f.rebuild(self.f.base_raw.replace(row(2,6).encode(), value.encode()))
            with self.assertRaisesRegex(ValueError, message):
                self.f.compile()

    def test_caps_fail_and_original_compilation_is_reusable(self):
        expected = self.f.compile()
        for limits in (AttachmentLimits(nodes=5), AttachmentLimits(incident_elements=4)):
            with self.assertRaisesRegex(ValueError, 'cap'):
                self.f.compile(limits=limits)
        self.assertEqual(expected, self.f.compile())
        for kwargs in ({'nodes':513}, {'groups':65}, {'incident_elements':4097}, {'nodes':True}):
            with self.assertRaises(ValueError): AttachmentLimits(**kwargs)

    def test_external_rehashed_geometry_or_incidence_lie_fails_source_check(self):
        for name, offset, fmt, value, message in [('node_positions', 5*3*8, '<d', 123., 'node differs'),
                                                ('beams_records', 10*8+4*8, '<Q', 11, 'element differs')]:
            original = (self.f.directory/(name+'.bin')).read_bytes()
            data = bytearray(original); struct.pack_into(fmt,data,offset,value)
            self.f.update_bytes(name,data)
            with self.assertRaisesRegex(ValueError, message): self.f.compile()
            self.f.update_bytes(name,original)

    def test_omitted_neighbor_row_cannot_hide_behind_valid_array_hashes(self):
        # Remove the incident beam while retaining the unrelated orientation-only beam.
        for name,width in [('beams_records',80),('beams_node_indices',8),('beams_source_lines',4),('beams_blank_masks',2)]:
            self.f.manifest['arrays'][name]['shape'][0] = 1
            self.f.update_bytes(name,(self.f.directory/(name+'.bin')).read_bytes()[:width])
        with self.assertRaisesRegex(ValueError,'omitted a source element'): self.f.compile()

    def test_index_disagreement_and_duplicate_structural_identity_fail(self):
        original=(self.f.directory/'solids_node_indices.bin').read_bytes()
        data=bytearray(original)
        struct.pack_into('<I',data,0,0)
        self.f.update_bytes('solids_node_indices',data)
        with self.assertRaisesRegex(ValueError,'index/source ID'):self.f.compile()
        self.f.update_bytes('solids_node_indices',original)
        records=bytearray((self.f.directory/'beams_records.bin').read_bytes())
        struct.pack_into('<Q',records,80,101)
        self.f.update_bytes('beams_records',records)
        with self.assertRaisesRegex(ValueError,'duplicate/invalid incident element'):self.f.compile()

    def test_unreferenced_operators_and_added_mass_remain_outside_closure(self):
        extra='*SET_NODE_ADD\n'+row(99)+'\n'+row(99)+'\n*ELEMENT_MASS_PART\n'+row(17,.01)+'\n'
        self.f.rebuild(self.f.raw.replace(b'*END',(extra+'*END').encode()))
        result=self.f.compile()
        self.assertEqual(len(result.rigid_groups),1)
        self.assertEqual(len(result.unsupported_set_blocks),1)
        self.assertIn(row(99),result.unsupported_set_blocks[0].raw_text)
        self.assertFalse(result.full_attachment_closure_qualified)
        self.assertFalse(result.attachment_mechanics_implemented)
        self.assertTrue(any('added masses' in reason for reason in result.unresolved))

    def test_shared_selected_node_neighbor_is_retained_without_rigid_membership(self):
        extra='*PART\nshared-node neighbor\n'+row(21,27,37)+'\n*ELEMENT_SHELL\n'+row(501,21,1,7,8,8,width=8)+'\n'
        self.f.rebuild(self.f.raw.replace(b'*END',(extra+'*END').encode()))
        result=self.f.compile()
        self.assertEqual(result.neighboring_part_ids,(18,19,20,21))
        self.assertEqual(result.selected_group_node_ids,(2,))
        node=next(n for n in result.node_incidence if n.node_id==1)
        self.assertEqual(node.incident_part_ids,(17,21))

    def test_empty_optional_family_is_explicit_and_hash_checked(self):
        self.f.rebuild(self.f.base_raw.replace(('*ELEMENT_BEAM\n'+row(401,19,9,10,6,width=8)+'\n'+row(402,19,6,9,10,width=8)+'\n').encode(), b''))
        result=self.f.compile()
        self.assertFalse(any(e.family=='beams' for e in result.incidence.elements))
        with self.assertRaisesRegex(ValueError,'row count'):
            load_array(self.f.directory,self.f.manifest,'beams_records','Q','<u8',10)
        self.assertEqual(len(load_array(self.f.directory,self.f.manifest,'beams_records','Q','<u8',10,allow_empty=True)),0)
        (self.f.directory/'beams_records.bin').write_bytes(b'x')
        with self.assertRaisesRegex(ValueError,'checksum/size'):self.f.compile()

    def test_e1_family_selection_and_explicit_missing_family_contract(self):
        old=scan_declarations(BytesIO(self.f.raw),'yaris-coarse-v1l.key')
        self.assertEqual(old.retained_families,DECLARATION_FAMILIES)
        self.assertTrue(all(f in DECLARATION_FAMILIES for f,_ in old.entries))
        explicit=scan_metadata(BytesIO(self.f.raw),'yaris-coarse-v1l.key',families=DECLARATION_FAMILIES)
        self.assertEqual(asdict(old),asdict(explicit))
        with self.assertRaisesRegex(ValueError,'family coverage'):
            compile_attachment_scope(old,self.f.load(),self.f.directory,self.f.archive,self.f.reference)
        with self.assertRaisesRegex(ValueError,'family selection'):
            scan_metadata(BytesIO(self.f.raw),'yaris-coarse-v1l.key',families={'guessed'})

    def test_explicit_v2_composer_preserves_v1_inventory_and_create_only_publication(self):
        self.assertTrue(QUADRATURE_PATH,'actual quadrature artifact is required, never skipped')
        ref=self.f.directory/'reference.json';ref.write_text(json.dumps(self.f.reference))
        output=self.f.directory/'typed.json'
        with patch.object(yaris_part,'REFERENCE',ref):
            old=yaris_part.compile_archive_readiness(self.f.archive,self.f.directory,QUADRATURE_PATH,17)
            result=yaris_part.compile_archive_attachment_readiness(self.f.archive,self.f.directory,QUADRATURE_PATH,17)
            self.assertEqual(result['schema'],'robo-dyna.source-part-readiness.v2')
            self.assertEqual(result['e2a_readiness'],old)
            self.assertFalse(result['e2a_readiness']['attachments']['typed_closure_qualified'])
            self.assertTrue(result['typed_attachment_scope']['literal_one_hop_references_validated'])
            self.assertFalse(result['full_attachment_closure_qualified'])
            for name,digest in result['attachment_generator_sources'].items():
                self.assertEqual(file_sha256(APP/name),digest)
            yaris_part.write_report(output,result);before=output.read_bytes()
            with self.assertRaises(FileExistsError):yaris_part.write_report(output,result)
            self.assertEqual(output.read_bytes(),before)
            self.f.rebuild(self.f.raw.replace(row(2,6).encode(),row(2,999).encode()))
            ref.write_text(json.dumps(self.f.reference))
            rejected=self.f.directory/'never-created.json'
            with self.assertRaises(ValueError):
                yaris_part.write_report(rejected,yaris_part.compile_archive_attachment_readiness(
                    self.f.archive,self.f.directory,QUADRATURE_PATH,17))
            self.assertFalse(rejected.exists());self.assertEqual(output.read_bytes(),before)


if __name__=='__main__':
    parser=argparse.ArgumentParser(add_help=False);parser.add_argument('--quadrature')
    args,remaining=parser.parse_known_args();QUADRATURE_PATH=args.quadrature or QUADRATURE_PATH
    unittest.main(argv=[sys.argv[0]]+remaining)
