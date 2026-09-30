#!/usr/bin/env python3
"""Source-authenticated ordered solid fold and unchanged direct fallback."""
from pathlib import Path
import argparse,hashlib,json,re
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[2]
PREFIX="lib_src/elements/solids/resident/"
REF="tl::fea::solids::batch_detail::read_tile_reference::"
MANIFEST="4d438cfbac75ecb93190c50ac62159ce312b69eff6dc5ca1418629b6e2066d27"
def once(text,old,new):
    assert text.count(old)==1,old
    return text.replace(old,new)
def function(text,marker):
    start=text.index(marker);begin=text.index("{",start);depth=0
    for end in range(begin,len(text)):
        depth+=(text[end]=="{")-(text[end]=="}")
        if not depth:return text[start:end+1]
    raise AssertionError(marker)
def record(p,row):
    raw=p.read_bytes();assert len(raw)==row["bytes"] and hashlib.sha256(raw).hexdigest()==row["sha256"],str(p)
    return raw.decode()
def checked():
    raw=(HERE/"source-manifest.json").read_bytes();assert hashlib.sha256(raw).hexdigest()==MANIFEST
    meta=json.loads(raw);assert meta["baseline"]=="7fc8f030fcea10be1f930cffb8745fa86fe65643"
    old={r["name"]:record(HERE/"frozen"/r["name"],r) for r in meta["frozen"]}
    for row in meta["unchanged"]+meta["production"]:record(ROOT/row["path"],row)
    for row in meta["wrapper_launches"]:
        assert row["before"].replace("Finalize<<<1,1,0,","Finalize<<<1,measurement::Threads,0,").replace("Finalize<<<1, 1, 0,","Finalize<<<1, measurement::Threads, 0,")==row["after"]
        assert record(ROOT/row["path"],row)==row["after"]
    v=old["MeasurementValues.h"];a=v.index("    diagnostics.native_internal_work_increment_j[family_index] += value.work;");b=v.index("\n  }\n  return true;",a);fold=v[a:b]
    helper="template<class Traits>\nTL_BRICK_HD inline bool AccumulateMeasurementOperand(Control& control,unsigned family_index,\n    std::size_t p,const MeasurementOperands<Traits::nodes>& value,\n    const NodalPreparedView* view) noexcept {\n  auto& diagnostics=control.diagnostics;\n"+fold+"\n  return true;\n}\n\n"
    marker="template<class Traits>\nTL_BRICK_HD inline bool MeasureOperandFamily(Storage& state, Control& control,"
    v=once(v,marker,helper+marker)
    v=once(v,fold+"\n  }\n  return true;","    if (!AccumulateMeasurementOperand<Traits>(control,family_index,p,value,view)) return false;\n  }\n  return true;")
    assert v==(ROOT/PREFIX/"MeasurementValues.h").read_text(),"Parent fold or operand preparation changed"
    c=old["Candidate.cu"];a=c.index("  Control next{};",c.index("__global__ void Finalize("));b=c.index("  const auto* prepared",a)
    begin=(ROOT/PREFIX/"measurement/Begin.h").read_text();assert c[a:b] in begin,"Initial identity changed"
    c=c[:a]+"  auto next=measurement::Begin(state,identity,initial);\n"+c[b:]
    marker="  auto& state = *storage;\n  auto next=measurement::Begin(state,identity,initial);"
    c=once(c,marker,"  if (operands_prepared) {\n    __shared__ measurement::Tile tile;\n    measurement::Finalize(*storage,view,identity,initial,tile);\n    return;\n  }\n  if (threadIdx.x) return;\n"+marker)
    c=once(c,'#include "MeasurementValues.h"','#include "MeasurementValues.h"\n#include "measurement/Finalize.cuh"')
    assert c.count("Finalize<<<1, 1, 0,")==2
    c=c.replace("Finalize<<<1, 1, 0,","Finalize<<<1, measurement::Threads, 0,")
    assert c==(ROOT/PREFIX/"Candidate.cu").read_text(),"Other production kernels or direct fallback changed"
    return old,c

def generated():
    old,current=checked()
    text=old["MeasurementValues.h"].replace('#include "Measure.h"','#include "'+PREFIX+'Measure.h"').replace("namespace tl::fea::solids::batch_detail {","namespace tl::fea::solids::batch_detail::read_tile_reference {")
    text=text.replace("MeasureOperandFamily<Traits>(",REF+"MeasureOperandFamily<Traits>(")
    # Replace calls only; the declaration has no template argument list.
    body=function(old["Candidate.cu"],"__global__ void Finalize(")
    body=body.replace("__global__ void Finalize(","TL_BRICK_HD inline void Finalize(")
    body=body.replace("MeasureFinalFamily<",REF+"MeasureFinalFamily<")
    reference='#pragma once\n#include "ReferenceMeasurementValues.h"\n#include <cfloat>\nnamespace tl::fea::solids::batch_detail::read_tile_reference {\n'+body+'\n}\n'
    live='#pragma once\n#include "'+PREFIX+'measurement/Finalize.cuh"\nnamespace tl::fea::solids::batch_detail::read_tile_current {\n'+function(current,"__global__ void Finalize(")+'\n}\n'
    # Host comparison keeps the exact serial fallback, without CUDA scheduling.
    host=function(current,"__global__ void Finalize(")
    prefix="  if (operands_prepared) {\n    __shared__ measurement::Tile tile;\n    measurement::Finalize(*storage,view,identity,initial,tile);\n    return;\n  }\n  if (threadIdx.x) return;\n"
    host=once(host,prefix,"").replace("__global__ void Finalize(","TL_BRICK_HD inline void CurrentFinalize(")
    host='#pragma once\n#include "'+PREFIX+'measurement/Begin.h"\nnamespace tl::fea::solids::batch_detail::control_test {\n'+host+'\n}\n'
    return {"ReferenceMeasurementValues.h":text,"ReferenceFinalize.h":reference,"CurrentFinalize.cuh":live,"CurrentSolidFinalize.h":host}
if __name__=="__main__":
    p=argparse.ArgumentParser();p.add_argument("--output",type=Path);a=p.parse_args();out=generated()
    if a.output:
        a.output.mkdir(parents=True,exist_ok=True)
        for name,text in out.items():(a.output/name).write_text(text)
    print(json.dumps({"status":"source_passed","baseline":"7fc8f030","retained_bytes_added":0,"shared_tile_bytes":10568,"compiled":False,"gpu_executed":False}))
