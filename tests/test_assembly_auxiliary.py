"""External joint/mass nodes retain literal evidence without becoming owners."""
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from test_source_assembly import AssemblyFixture
from test_canonical_part_geometry import row


class AssemblyAuxiliary(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.f=AssemblyFixture(self.temp.name)

    def tearDown(self):
        self.temp.cleanup()

    def source(self,mass=True,joint=True,kind='*CONSTRAINED_JOINT_SPHERICAL_ID'):
        # NID10 remains a genuine source node and a beam orientation ID, but
        # deleting beam401 removes its physical shell/solid/beam incidence.
        raw=self.f.base_raw.replace((row(401,19,9,10,6,width=8)+'\n').encode(),b'')
        extra=[]
        if mass:extra+=['*ELEMENT_MASS',row(941,10,width=8)+row('1.000100e-05',width=16)]
        if joint:extra+=[kind,row(942),row(10,9)]
        raw=raw.replace(b'*END',('\n'.join(extra+['*END'])).encode());self.f.rebuild(raw)

    def compile(self):
        return self.f.compile_assembly(material_policy='layered_law1_or_law44',boundary_policy='released_external_connections')

    def test_literal_auxiliary_frontier_keeps_geometry_and_released_interfaces(self):
        original=self.f.compile_assembly();self.source();result=self.compile();a=result['attachments']['auxiliary_frontier']
        self.assertEqual(result['counts'],original['counts'])
        self.assertEqual(result['geometry']['source_node_ids'],original['geometry']['source_node_ids'])
        self.assertEqual(result['boundary'],original['boundary'] | {'policy':'released_external_connections',
            'interpretation':result['boundary']['interpretation']})
        self.assertEqual(result['attachments']['external_node_ids'],[9,10])
        self.assertEqual(a['source_node_ids'],[10]);self.assertEqual(len(a['source_blocks']),2)
        self.assertEqual(a['point_masses'][0]['source_element_id'],941)
        self.assertEqual(a['point_masses'][0]['supplied_mass_source'],1.000100e-5)
        self.assertEqual(a['point_masses'][0]['supplied_mass_kg'],1.000100e-5*1000)
        self.assertEqual(a['spherical_joints'][0]['source_node_ids'],[10,9])
        self.assertFalse(a['selected_owner_nodes_added']);self.assertFalse(a['selected_owner_mass_added'])
        self.assertFalse(a['mechanics_qualified'])
        for policy in ('tabulated','law44_tabulated_or_linear'):
            with self.assertRaisesRegex(ValueError,'frontier node lacks source structural incidence'):
                self.f.compile_assembly(material_policy=policy)

    def test_missing_mass_joint_and_unknown_joint_remain_unadmitted(self):
        for options in ({'mass':False},{'joint':False},{'kind':'*CONSTRAINED_JOINT_CYLINDRICAL_ID'}):
            self.source(**options)
            with self.assertRaisesRegex(ValueError,'auxiliary'):
                self.compile()
        self.source();self.assertEqual(self.compile()['attachments']['auxiliary_frontier']['source_node_ids'],[10])


if __name__=='__main__':unittest.main()
