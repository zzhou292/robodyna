"""Byte-format, admission and pinned reference checks; no solver is executed."""
from copy import deepcopy
import hashlib
import importlib.util
import os
from pathlib import Path
import struct
import tempfile
import unittest
from unittest.mock import patch
from . import trajectory as t


def history(marker=0):
    value={k:[0 if k in ('IRTLM','ICONT_I') else 0.0]*(2*w) for k,w in t.STRIDES.items()}
    value['IRTLM'][0]=marker
    return value


def fixture():
    domain=t.Domain((10,20),(20,10),(20,10),3,2,.25)
    def clock(epoch):return dict(TT=epoch*.25,DT1=.25 if epoch else 0.,DT12=0. if not epoch else .125 if epoch==1 else .25,NCYCLE=epoch,NSPMD=1,IRESP=0)
    result=[dict(stage='initial_rows_after_begin',observation=dict(clock=clock(0),rows=history()))]
    for epoch in range(3):
        c=clock(epoch)
        arrays=dict(X=[1.,2.,3.,-0.,5.,6.],V=[11.,12.,13.,14.,15.,16.],A=[21.,22.,23.,24.,25.,26.],STIFN=[31.,32.],MS=[2.,1.],ITAB=[20,10])
        result.append(dict(stage='i25mainf_entry',observation=dict(clock=c,NVSIZ=128,nodes=2,arrays=arrays)))
        if not epoch:result.append(dict(stage='coefficient_controls',observation=dict(clock=c,controls=dict(KMIN=0.,KMAX=1e30,IGSTI=4,ISTIF_MSDT=0))))
        if epoch:
            result.extend([
                dict(stage='i25dst3_3_entry',observation=dict(clock=c,NVSIZ=128,jlt=2,nsn=2,arrays=dict(CAND_N=[2,1],CAND_E=[3,2]))),
                dict(stage='i25dst3_3_return',observation=dict(clock=c,NVSIZ=128,jlt=2,nsn=2,arrays=dict(PENE=[1.,2.],STIF=[3.,4.]),history={k:v for k,v in history().items() if k in ('IRTLM','PENE_OLD')})),
                dict(stage='i25for3_entry',observation=dict(clock=c,NVSIZ=128,jlt=2,nsn=2,source_occurrences_one_based=[2,1],arrays=dict(CAND_N_N=[1,2],PENE=[1.,0.],STIF=[3.,4.]))),
                dict(stage='i25for3_return',observation=dict(clock=c,NVSIZ=128,jlt=2,nsn=2,source_occurrences_one_based=[2,1],arrays=dict(H1=[1.,0.],H2=[0.,0.],H3=[0.,0.],H4=[0.,0.],STIF=[3.,4.]),assembly_rows_one_based=[1],active_force=dict(N1=[0.],N2=[0.],N3=[1.]),history={k:v for k,v in history().items() if k not in ('TIME_S','ICONT_I')}))])
        final=history(1 if epoch else 0)
        if epoch==2:final['IRTLM'][4]=1
        result.append(dict(stage='i25mainf_return',observation=dict(clock=c,NVSIZ=128,nodes=2,arrays=dict(A=[41.,42.,43.,44.,45.,46.],STIFN=[51.,52.]),rows=final)))
    return domain,result


class Contract(unittest.TestCase):
    def test_exact_scalar_order_node_permutation_and_signed_zero(self):
        domain,sequence=fixture();before=sequence[1]['observation'];after=sequence[3]['observation']
        after['rows']={k:list(range(i*20,i*20+2*w)) for i,(k,w) in enumerate(t.STRIDES.items())}
        out=t.Writer(domain);out.frame(domain,0,before,after,[dict(jlt=2,arrays=dict(CAND_N=[2,1],CAND_E=[3,2]))])
        expected=bytearray(b'T25REF01'+struct.pack('<IIIQddd',2,2,3,0,0.,0.,0.))
        expected.extend(struct.pack('<28d',-0.,5.,6.,1.,2.,3.,14.,15.,16.,11.,12.,13.,24.,25.,26.,21.,22.,23.,44.,45.,46.,41.,42.,43.,52.,51.,32.,31.))
        # STIFN precedes neither force field: reader order is A_before,A_after,STIFN_before,STIFN_after.
        expected[-32:]=struct.pack('<4d',32.,31.,52.,51.)
        for row in range(2):
            expected.extend(struct.pack('<4i15di',*[4*row+j for j in range(4)],
                21+5*row,41+2*row,20+5*row,40+2*row,22+5*row,
                63+6*row,64+6*row,65+6*row,60+6*row,61+6*row,62+6*row,
                23+5*row,24+5*row,80+2*row,81+2*row,100+row))
        expected.extend(struct.pack('<II4i',1,2,2,1,3,2))
        self.assertEqual(out.data,expected)

    def test_force_base_activity_and_terminal_exclusion(self):
        d,s=fixture();binary,stats=t.serialize(s,d,{10:1.,20:2.})
        self.assertEqual(struct.unpack_from('<III',binary,8),(2,2,3))
        self.assertEqual(stats['active_force_base_epochs'],[1])
        self.assertEqual(stats['episodes_by_secondary'],[1,0])
        self.assertEqual(stats['first_active_step'],1)
        self.assertEqual(stats['episode_start_force_base_epochs'],[[1],[]])

    def test_sequence_clock_domain_and_defined_channel_rejections(self):
        d,base=fixture()
        mutations=[lambda s:s.pop(),lambda s:s.insert(2,deepcopy(s[1])),
          lambda s:s[1]['observation']['clock'].__setitem__('DT1',.25),
          lambda s:s[1]['observation']['clock'].__setitem__('NCYCLE',False),
          lambda s:s[1]['observation']['arrays'].__setitem__('ITAB',[10,20]),
          lambda s:s[1]['observation']['arrays']['X'].__setitem__(0,float('nan')),
          lambda s:s[1]['observation']['arrays']['MS'].__setitem__(0,3.),
          lambda s:s[3]['observation']['rows']['PENE_OLD'].pop(),
          lambda s:s[5]['observation']['arrays']['CAND_E'].__setitem__(0,4),
          lambda s:s[8]['observation'].__setitem__('source_occurrences_one_based',[1,2]),
          lambda s:s[8]['observation'].__setitem__('assembly_rows_one_based',[1,2]),
          lambda s:s[6]['observation'].__setitem__('nsn',1)]
        for i,mutate in enumerate(mutations):
            with self.subTest(mutation=i):
                s=deepcopy(base);mutate(s)
                with self.assertRaises(ValueError):t.serialize(s,d,{10:1.,20:2.})
        with patch.object(t,'MAX_BINARY',30):
            with self.assertRaises(ValueError):t.serialize(base,d)

    def test_pin_extent_hash_and_json_types(self):
        with tempfile.TemporaryDirectory() as temp:
            p=Path(temp)/'input';p.write_bytes(b'original')
            manifest=dict(schema='robo_dyna.native_trajectory_inputs.v1',files=dict(x=dict(path='input',bytes=8,sha256=hashlib.sha256(b'original').hexdigest())))
            self.assertEqual(t.pinned_inputs(manifest,temp)[0]['x'],b'original')
            p.write_bytes(b'mutated!')
            with self.assertRaises(ValueError):t.pinned_inputs(manifest,temp)
            manifest['files']['x']['path']='../input'
            with self.assertRaises(ValueError):t.pinned_inputs(manifest,temp)
        for bad in ('{"x":1,"x":2}','{"x":NaN}'):
            with self.assertRaises(ValueError):t.parse(bad)
        for bad in (True,2**64,float('inf')):
            with self.assertRaises(ValueError):t.integer(bad)


@unittest.skipUnless(os.environ.get('TYPE25_REFERENCE_WORKSPACE'),'Pinned workspace observations are optional outside owning qualification')
class PinnedReference(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.workspace=Path(os.environ['TYPE25_REFERENCE_WORKSPACE'])
        cls.manifest=t.parse(Path(__file__).with_name('trajectory-inputs.json').read_bytes())

    def test_legacy_binary_is_byte_identical_to_qualified_serializer(self):
        t.pinned_inputs(self.manifest,self.workspace)
        path=self.workspace/self.manifest['files']['legacy_serializer']['path']
        spec=importlib.util.spec_from_file_location('legacy_type25_fixture',path);old=importlib.util.module_from_spec(spec);spec.loader.exec_module(old)
        values=old.load();_,expected=old.produce()
        main=old.one(values['observation.jsonl.xz'],'main')['arrays']
        classification=old.one(values['observation.jsonl.xz'],'classification')
        declared=tuple(row['id'] for row in values['scene.json']['mesh']['nodes'])
        native=tuple(main['ITAB']);secondary=tuple(native[i-1] for i in classification['arrays']['NSV'])
        domain=t.Domain(declared,native,secondary,classification['nrtm'],1000,3e-7)
        actual,_=t.serialize(values['sequence.jsonl.xz'],domain)
        self.assertEqual(actual,expected)

    def test_actual_v2_scope_identity_activity_and_repeated_conversion(self):
        first,metadata=t.convert(self.manifest,self.workspace)
        second,again=t.convert(self.manifest,self.workspace)
        self.assertEqual(first,second);self.assertEqual(metadata,again)
        self.assertEqual(metadata['reference_sha256'],hashlib.sha256(first).hexdigest())
        self.assertEqual((metadata['intervals'],metadata['frames'],metadata['active_steps'],metadata['first_active_step']),(1000,1001,38,302))
        self.assertEqual(metadata['episodes_by_secondary'],[0]*9+[2,0,1,2,0,1,2,0,1])
        self.assertEqual(len(metadata['node_source_ids']),18)
        self.assertEqual(metadata['secondary_source_ids'],metadata['node_source_ids'])
        for key in ('mass_native_tonne','inertia_native_tonne_mm2'):
            self.assertEqual(len(metadata[key]),18);self.assertTrue(all(x>0 for x in metadata[key]))


if __name__=='__main__':unittest.main()
