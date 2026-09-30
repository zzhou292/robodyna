#!/usr/bin/env python3
"""Adapt exact2733 bodies by includes, namespace and named-call qualification only."""
from pathlib import Path
import argparse,hashlib,json,re
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[2]
SOLID='tl::fea::solids::batch_detail::control2733::'
QBAT='tl::fea::qbat::mapped::control2733::'
def function(text,marker):
    start=text.index(marker);begin=text.index('{',start);depth=0
    for end in range(begin,len(text)):
        depth+=(text[end]=='{')-(text[end]=='}')
        if depth==0:return text[start:end+1]
    raise AssertionError('Unclosed function')
def includes(text,source,replacements={}):
    def replace(match):
        name=match.group(1)
        if name in replacements:return '#include "'+replacements[name]+'"'
        path=(ROOT/source).parent/name
        return '#include "'+str(path.resolve().relative_to(ROOT))+'"'
    return re.sub(r'#include "([^"]+)"',replace,text)
def generated():
    meta=json.loads((HERE/'baseline.json').read_text());assert meta['commit']=='2733b4ccb916db96fc4d7d51cdfa21566d6e227d'
    frozen={}
    for row in meta['files']:
        raw=(HERE/row['fixture']).read_bytes()
        assert len(raw)==row['bytes'] and hashlib.sha256(raw).hexdigest()==row['sha256']
        frozen[row['path']]=raw.decode()
    qpath='lib_src/elements/qbat/mapped/Measurement.h'
    q=includes(frozen[qpath],qpath)
    q=q.replace('namespace tl::fea::qbat::mapped {','namespace tl::fea::qbat::mapped::control2733 {')
    q=q.replace('MergeMaximum(maximum,',QBAT+'MergeMaximum(maximum,')
    q=q.replace('&& FinishMeasurement(', '&& '+QBAT+'FinishMeasurement(')
    q=q.replace('!MeasureStaged(', '!'+QBAT+'MeasureStaged(')
    path='lib_src/elements/solids/resident/Measure.h'
    m=includes(frozen[path],path).replace('namespace tl::fea::solids::batch_detail {','namespace tl::fea::solids::batch_detail::control2733 {')
    for name in ['Work','HourglassWork','PlasticWork','NativeDt']:
        m=re.sub(r'\b'+name+r'\(cache\)',SOLID+name+'(cache)',m)
    m=m.replace('return MeasureFamilyWithCheck<Traits>(', 'return '+SOLID+'MeasureFamilyWithCheck<Traits>(')
    path='lib_src/elements/solids/resident/MeasurementValues.h'
    v=includes(frozen[path],path,{'Measure.h':'Control2733SolidMeasure.h'}).replace('namespace tl::fea::solids::batch_detail {','namespace tl::fea::solids::batch_detail::control2733 {')
    for name in ['Work','HourglassWork','PlasticWork','NativeDt']:
        v=re.sub(r'\b'+name+r'\(cache\)',SOLID+name+'(cache)',v)
    v=v.replace('? MeasureOperandFamily<Traits>(', '? '+SOLID+'MeasureOperandFamily<Traits>(')
    v=v.replace(': MeasureValidatedFamily<Traits>(', ': '+SOLID+'MeasureValidatedFamily<Traits>(')
    path='lib_src/elements/solids/resident/Candidate.cu'
    old=function(frozen[path],'__global__ void Finalize(')
    old=old.replace('__global__ void Finalize(','TL_BRICK_HD inline void Finalize(')
    old=old.replace('!MeasureFinalFamily<','!'+SOLID+'MeasureFinalFamily<')
    current=function((ROOT/path).read_text(),'__global__ void Finalize(')
    current=current.replace('__global__ void Finalize(','TL_BRICK_HD inline void CurrentFinalize(')
    return {'Control2733Qbat.h':q,'Control2733SolidMeasure.h':m,'Control2733SolidValues.h':v,
        'Control2733SolidFinalize.h':'#pragma once\n#include "Control2733SolidValues.h"\n#include <cfloat>\nnamespace tl::fea::solids::batch_detail::control2733 {\n'+old+'\n}\n',
        'CurrentSolidFinalize.h':'#pragma once\n#include "lib_src/elements/solids/resident/MeasurementValues.h"\n#include <cfloat>\nnamespace tl::fea::solids::batch_detail::control_test {\n'+current+'\n}\n'}
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
    for name,text in generated().items():(a.output/name).write_text(text)
