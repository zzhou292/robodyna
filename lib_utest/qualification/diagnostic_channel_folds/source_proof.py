#!/usr/bin/env python3
"""Pin the baseline scalar authorities and the complete current dependency set."""
from pathlib import Path
import argparse,hashlib,json,re
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[2]
def function(text,marker):
    start=text.index(marker);begin=text.index('{',start);depth=0
    for end in range(begin,len(text)):
        depth+=(text[end]=='{')-(text[end]=='}')
        if not depth:return text[start:end+1]
    raise AssertionError(marker)
def checked():
    meta=json.loads((HERE/'source-manifest.json').read_text())
    for row in meta['files']+meta['frozen']:
        p=ROOT/row['path'];raw=p.read_bytes()
        assert len(raw)==row['bytes'] and hashlib.sha256(raw).hexdigest()==row['sha256'],str(p)
    for path in meta['unchanged_authorities']:
        assert (ROOT/path).read_bytes()==(HERE/'frozen'/path).read_bytes(),path
    return meta

def generated():
    frozen=HERE/'frozen';out={}
    s='lib_src/elements/solids/resident/'
    text=(frozen/s/'MeasurementValues.h').read_text().replace('#include "Measure.h"','#include "'+s+'Measure.h"')
    namespace='tl::fea::solids::batch_detail::read_tile_reference::'
    text=text.replace('namespace tl::fea::solids::batch_detail {','namespace '+namespace[:-2]+' {')
    text=text.replace('MeasureOperandFamily<Traits>(',namespace+'MeasureOperandFamily<Traits>(')
    text=text.replace('AccumulateMeasurementOperand<Traits>(',namespace+'AccumulateMeasurementOperand<Traits>(')
    out['solid/ReferenceMeasurementValues.h']=text
    live=function((ROOT/s/'Candidate.cu').read_text(),'__global__ void Finalize(')
    prefix='  if (operands_prepared) {\n    __shared__ measurement::Tile tile;\n    measurement::Finalize(*storage,view,identity,initial,tile);\n    return;\n  }\n  if (threadIdx.x) return;\n'
    assert live.count(prefix)==1
    body=live.replace(prefix,'').replace('__global__ void Finalize(','TL_BRICK_HD inline void Finalize(')
    body=body.replace('MeasureFinalFamily<',namespace+'MeasureFinalFamily<')
    out['solid/ReferenceFinalize.h']='#pragma once\n#include "ReferenceMeasurementValues.h"\n#include "'+s+'measurement/Begin.h"\nnamespace '+namespace[:-2]+' {\n'+body+'\n}\n'
    out['solid/CurrentFinalize.cuh']='#pragma once\n#include "'+s+'measurement/Finalize.cuh"\nnamespace tl::fea::solids::batch_detail::read_tile_current {\n'+live+'\n}\n'
    p='lib_src/elements/qbat/mapped/'
    text=(frozen/p/'Measurement.h').read_text()
    text=re.sub(r'#include "([^"]+)"',lambda m:'#include "'+str((ROOT/p/m[1]).resolve().relative_to(ROOT))+'"',text)
    ns='tl::fea::qbat::mapped::read_tile_reference::'
    text=text.replace('namespace tl::fea::qbat::mapped {','namespace '+ns[:-2]+' {')
    for old,new in [('MergeMaximum(maximum,',ns+'MergeMaximum(maximum,'),('&& FinishMeasurement(','&& '+ns+'FinishMeasurement('),('!MeasureStaged(','!'+ns+'MeasureStaged('),('auto next=BeginMeasurement(','auto next='+ns+'BeginMeasurement(')]:text=text.replace(old,new)
    out['qbat/ReferenceMeasurement.h']=text
    return out
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path);a=p.parse_args();m=checked()
    if a.output:
        for name,text in generated().items():
            dest=a.output/name;dest.parent.mkdir(parents=True,exist_ok=True);dest.write_text(text)
    print(json.dumps({'status':'source_passed','baseline':m['baseline'],'files':len(m['files']),'frozen':len(m['frozen']),'retained_bytes_added':0}))
