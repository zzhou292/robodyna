"""Expected-observation conversion only; no solver/debugger/GPU execution."""
from copy import deepcopy
import hashlib
import os
from pathlib import Path
import unittest
from . import trajectory as t
from . import rigid_trajectory as r
from .test_trajectory import fixture


class AuxiliaryDomain(unittest.TestCase):
    def test_auxiliary_primary_is_explicit_and_does_not_change_physical_format(self):
        d,source=fixture();expected,_=t.serialize(source,d,{10:1.,20:2.})
        sequence=deepcopy(source)
        for row in sequence:
            if row['stage'] not in ('i25mainf_entry','i25mainf_return'):continue
            data=row['observation'];data['nodes']=3
            for key,values in list(data['arrays'].items()):
                if key=='ITAB':data['arrays'][key]=[20,99,10];continue
                width=3 if key in ('X','V','A') else 1
                data['arrays'][key]=values[:width]+[9.]*width+values[width:]
        extended=t.Domain(d.node_ids,(20,99,10),d.secondary_ids,d.main_count,d.intervals,d.dt_s,(99,))
        actual,_=t.serialize(sequence,extended,{10:1.,20:2.,99:9.})
        self.assertEqual(actual,expected)
        for auxiliary in ((),(10,),(100,),(99,99)):
            bad=t.Domain(d.node_ids,(20,99,10),d.secondary_ids,d.main_count,d.intervals,d.dt_s,auxiliary)
            with self.subTest(auxiliary=auxiliary),self.assertRaises(ValueError):t.serialize(sequence,bad)

    def test_secondary_ordinals_cannot_use_python_negative_indexing(self):
        self.assertEqual(r.secondary_source_ids((10,99,20),[3,1]),(20,10))
        for raw in ([0],[-1],[4],[True],[1.0]):
            with self.subTest(raw=raw),self.assertRaises(ValueError):r.secondary_source_ids((10,99,20),raw)

    def test_rigid_force_and_recovery_clocks_bind_the_actual_phase(self):
        for cycle in (0,1,302,1000):
            base=dict(NCYCLE=cycle,TT=cycle*3e-7,DT1=3e-7 if cycle else 0.,DT12=0. if not cycle else 1.5e-7 if cycle==1 else 3e-7,NSPMD=1,IRESP=0)
            for recovery in (False,True):
                actual=dict(base)
                if recovery:actual['DT12']=1.5e-7 if cycle==0 else 3e-7
                r.bind_rigid_clock(actual,base,recovery)
                for key in ('NCYCLE','TT','DT1','DT12','NSPMD','IRESP'):
                    broken=dict(actual);broken[key]+=1
                    with self.subTest(cycle=cycle,recovery=recovery,field=key),self.assertRaises(ValueError):r.bind_rigid_clock(broken,base,recovery)
                broken=dict(actual,NSPMD=True)
                with self.assertRaises(ValueError):r.bind_rigid_clock(broken,base,recovery)


@unittest.skipUnless(os.environ.get('TYPE25_REFERENCE_WORKSPACE'),'Pinned evidence only in owning qualification')
class RigidPinnedReference(unittest.TestCase):
    def test_complete_v3_conversion_retains_physical_and_rigid_phase_boundaries(self):
        manifest=t.parse(Path(__file__).with_name('rigid-trajectory-inputs.json').read_bytes())
        contact,groups,metadata=r.convert(Path(os.environ['TYPE25_REFERENCE_WORKSPACE']),manifest)
        self.assertEqual(contact[:8],b'T25REF01');self.assertEqual(groups[:8],b'T25RIG01')
        self.assertEqual(len(groups),20+1001*(8+186*8))
        self.assertEqual((metadata['active_steps'],metadata['first_active_step']),(18,302))
        self.assertEqual(metadata['node_source_ids'],tuple(range(1,19)))
        self.assertEqual(metadata['auxiliary_primary_source_id'],19)
        self.assertEqual(metadata['reference_sha256'],hashlib.sha256(contact).hexdigest())
        self.assertEqual(metadata['rigid_reference_sha256'],hashlib.sha256(groups).hexdigest())
        bad=deepcopy(manifest);bad['files']['sequence']['sha256']='0'*64
        with self.assertRaises(ValueError):r.convert(Path(os.environ['TYPE25_REFERENCE_WORKSPACE']),bad)


if __name__=='__main__':unittest.main()
