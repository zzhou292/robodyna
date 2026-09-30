#!/usr/bin/env python3
"""Pinned qualification operands and expected results, never a restart adapter."""
import argparse
import hashlib
import json
import lzma
import math
from pathlib import Path
import struct

HERE=Path(__file__).resolve().parent

def load():
    manifest=json.loads((HERE/'fixture/manifest.json').read_text())
    result={}
    for record in manifest['files']:
        data=(HERE/'fixture'/record['file']).read_bytes()
        assert len(data)==record['bytes'] and hashlib.sha256(data).hexdigest()==record['sha256']
        raw=lzma.decompress(data) if record['file'].endswith('.xz') else data
        assert len(raw)==record['original_bytes'] and hashlib.sha256(raw).hexdigest()==record['original_sha256']
        result[record['file']]=json.loads(raw) if record['file'].endswith('.json') else [json.loads(s) for s in raw.splitlines()]
    return result

def one(records,stage):
    found=[r.get('observation',r) for r in records if r['stage']==stage]
    assert len(found)==1,(stage,len(found))
    return found[0]

def real(x):
    assert math.isfinite(x)
    return float(x).hex()

def array(name,values,kind='double'):
    values=list(values)
    encode=real if kind=='double' else lambda x:str(int(x))
    return 'inline constexpr '+kind+' '+name+'['+str(len(values))+']={'+','.join(encode(x) for x in values)+'};\n'

def produce():
    data=load();scene=data['scene.json'];obs=data['observation.jsonl.xz'];seq=data['sequence.jsonl.xz']
    startup=one(data['startup.jsonl.xz'],'initia_return')['arrays']
    controls=one(obs,'controls');main=one(obs,'main')['arrays'];cls=one(obs,'classification')['arrays']
    assembly=data['assembly.json'];assert assembly['complete']
    assert assembly['observation']['controls']=={'JLT':3,'NIN':1,'JTASK':1,'INTTH':0,'IFORM':0,'NODADT_THERM':0,'IPARIT':0,'NPINCH':0}
    assert assembly['observation']['clock']['NSPMD']==1
    boundary=one(obs,'boundary')['arrays'];inventory=one(obs,'inventory_chunk')
    declarations=scene['mesh']['nodes'];ids=[r['id'] for r in declarations];assert ids==list(range(1,19))
    assert len(set(main['ITAB']))==18 and sorted(main['ITAB'])==ids
    index={source:i for i,source in enumerate(ids)}
    native_to_physical=[index[source] for source in main['ITAB']]
    secondary=[native_to_physical[node-1] for node in cls['NSV']]
    assert len(secondary)==18 and len(set(secondary))==18
    node_constraints=[0]*18;node_skew=[0]*18
    for native,physical in enumerate(native_to_physical):
        node_constraints[physical]=inventory['arrays']['ICODT'][native]
        node_skew[physical]=inventory['arrays']['ISKEW'][native]
    connectivity=[native_to_physical[node-1] for node in cls['IRECT']]
    assert len(connectivity)==64 and cls['MSEGTYP']==list(range(9,17))+list(range(-1,-9,-1))
    assert len(cls['NOD_NORMAL'])==192 and len(cls['GAPN_M'])==64 and len(boundary['VTX_BISECTOR'])==108
    assert inventory['controls']['NRTM']==8 and controls['controls']['NRTM']==16
    assert inventory['removal_offsets']==[0]*17 and not inventory['removal_nodes']
    # Independent source proof: PREPARE_SPLIT_I25, a62b27e6 inter_tools.F
    # 6187..6232, ascending main/corner order, omits repeated fourth T3 corner.
    offsets=[0];entries=[]
    for reference in range(1,19):
        for face in range(16):
            slots=3 if connectivity[4*face+2]==connectivity[4*face+3] else 4
            for slot in range(slots):
                if cls['ADMSR'][4*face+slot]==reference:entries.append(face+1)
        offsets.append(len(entries))
    initial=one(seq,'initial_rows_after_begin');assert initial['clock']['NCYCLE']==0
    assert initial['rows']['ICONT_I']==[0]*18
    coefficient=[r['observation']['controls'] for r in seq if r['stage']=='coefficient_controls']
    assert len(coefficient)==1 # Probe emits once and independently rejects any later change.
    assert coefficient[0]=={'KMIN':0.0,'KMAX':1e30,'IGSTI':4,'ISTIF_MSDT':0}
    header='// Generated from pinned qualification observations; never a shipping source factory.\n#pragma once\n#include <cstdint>\nnamespace native_scene_fixture {\n'
    header+=array('NodeIds',ids,'std::uint64_t')
    header+=array('PositionMm',(v for n in declarations for v in n['xyz_mm']))
    for name,key,width in [('Quads','patch',4),('Triangles','wall',3)]:
        cells=scene['mesh'][key];assert all(len(c['nodes'])==width for c in cells)
        header+=array(name+'Ids',(c['id'] for c in cells),'std::uint64_t')
        header+=array(name+'Nodes',(index[n] for c in cells for n in c['nodes']),'std::uint32_t')
    header+=array('NativeToPhysical',native_to_physical,'std::uint32_t')
    header+=array('SecondaryNodes',secondary,'std::uint32_t')
    header+=array('ConstraintCodes',node_constraints,'int')+array('SkewCodes',node_skew,'int')
    header+=array('MainNodes',connectivity,'std::uint32_t')
    header+=array('MainGlobalIds',boundary['MSEGLO'],'int')+array('MainRoles',cls['MSEGTYP'],'int')
    for name,key,kind in [('MainNormals','NOD_NORMAL','double'),('MainNormalReferences','ADMSR','int'),('Neighbors','MVOISIN','int'),('Boundaries','LBOUND','int'),('MainK','STF','double'),('SecondaryK','STFN','double'),('MainGaps','GAPN_M','double'),('MainMaximumGaps','GAP_M','double'),('SecondaryGaps','GAP_S','double')]:
        header+=array(name,cls[key],kind)
    header+=array('Bisectors',boundary['VTX_BISECTOR'])
    header+=array('NormalOffsets',offsets,'std::uint32_t')+array('NormalEntries',entries,'std::uint32_t')
    header+=array('Curvature',inventory['arrays']['CURV_MAX'])
    header+=array('InitialContact',initial['rows']['ICONT_I'],'int')
    header+=array('ObservedMass',startup['MS'])+array('ObservedInertia',startup['IN'])
    header+=array('AssemblyControls',[assembly['observation']['controls'][k] for k in ['IPARIT','NPINCH','INTTH','IFORM','NODADT_THERM']],'int')
    response=one(obs,'positive_response')
    assert response['controls']=={'MFROT':2,'IFQ':10,'IORTHFRIC':0,'INTTH':0,'ILEV':1,'INTEREFRIC':0,'IGSTI':4,'IVIS2':1,'INACTI':5,'KDTINT':0,'IDTMINS':0,'IDTMINS_INT':0,'INCONV':1}
    assert response['scalars']['ALPHA0']==1 and response['scalars']['VISC']==.05
    header+=array('NativeControls',[inventory['scalars']['MARGE'],inventory['scalars']['DRAD'],inventory['scalars']['DGAPLOAD'],coefficient[0]['KMIN'],coefficient[0]['KMAX']])
    material=scene['scene']['material']
    header+=array('Material',[material[k] for k in ['density_tonne_mm3','young_n_mm2','poisson','yield_n_mm2','plastic_hardening_n_mm2','rate_c_per_s','rate_p','rate_filter_hz']])
    header+=array('TimeAndThickness',[scene['scene']['time_step_cap_s'],scene['scene']['end_time_s'],scene['scene']['thickness_mm']])
    header+=array('VelocityMmS',scene['scene']['velocity_mm_s'])
    assert controls['clock']['IRESP']==0
    header+='inline constexpr int ResponsePrecision=0;\ninline constexpr unsigned ForcePacketSize=128;\n}\n'
    inputs={r['observation']['clock']['NCYCLE']:r['observation'] for r in seq if r['stage']=='i25mainf_entry'}
    outputs={r['observation']['clock']['NCYCLE']:r['observation'] for r in seq if r['stage']=='i25mainf_return'}
    assert sorted(inputs)==sorted(outputs)==list(range(1001))
    geometry={i:[] for i in inputs}
    for r in seq:
        if r['stage']=='i25dst3_3_entry':geometry[r['observation']['clock']['NCYCLE']].append(r['observation'])
    assert sum(map(len,geometry.values()))==40
    binary=bytearray(b'T25REF01'+struct.pack('<III',18,18,1001))
    def doubles(v):
        v=list(v);assert all(math.isfinite(x) for x in v);binary.extend(struct.pack('<'+'d'*len(v),*v))
    def integers(v):
        v=list(v);binary.extend(struct.pack('<'+'i'*len(v),*v))
    def physical(values,width):
        result=[0.]*(18*width)
        for native,destination in enumerate(native_to_physical):result[width*destination:width*(destination+1)]=values[width*native:width*(native+1)]
        return result
    for epoch in range(1001):
        before,after=inputs[epoch],outputs[epoch];clock=before['clock']
        assert before['NVSIZ']==after['NVSIZ']==128
        assert clock['DT1']==(0 if epoch==0 else 3e-7)
        assert clock['DT12']==(0 if epoch==0 else 1.5e-7 if epoch==1 else 3e-7)
        binary.extend(struct.pack('<Q',epoch));doubles([clock['TT'],clock['DT1'],clock['DT12']])
        for key in ['X','V']:doubles(physical(before['arrays'][key],3))
        for values,width in [(before['arrays']['A'],3),(after['arrays']['A'],3),(before['arrays']['STIFN'],1),(after['arrays']['STIFN'],1)]:doubles(physical(values,width))
        rows=after['rows']
        for key,width in [('IRTLM',4),('PENE_OLD',5),('STIF_OLD',2),('SECND_FR',6),('TIME_S',2),('ICONT_I',1)]:assert len(rows[key])==18*width
        for row in range(18):
            integers(rows['IRTLM'][4*row:4*row+4]);p=rows['PENE_OLD'][5*row:5*row+5];k=rows['STIF_OLD'][2*row:2*row+2];f=rows['SECND_FR'][6*row:6*row+6]
            doubles([p[1],k[1],p[0],k[0],p[2],*f[3:],*f[:3],p[3],p[4],*rows['TIME_S'][2*row:2*row+2]])
            integers([rows['ICONT_I'][row]])
        binary.extend(struct.pack('<I',len(geometry[epoch])))
        for packet in geometry[epoch]:
            count=packet['jlt'];binary.extend(struct.pack('<I',count))
            assert count<=128 and len(packet['arrays']['CAND_N'])==len(packet['arrays']['CAND_E'])==count
            integers(packet['arrays']['CAND_N']);integers(packet['arrays']['CAND_E'])
    return header.encode(),bytes(binary)

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True);parser.add_argument('--check',action='store_true');args=parser.parse_args()
    header,binary=produce();args.output.mkdir(parents=True,exist_ok=True)
    for name,data in [('ObservedScene.h',header),('native-reference.bin',binary)]:
        path=args.output/name
        if args.check:assert path.read_bytes()==data,path
        else:path.write_bytes(data)
    print(json.dumps({'status':'passed','header_bytes':len(header),'reference_bytes':len(binary),'frames':1001}))
if __name__=='__main__':main()
