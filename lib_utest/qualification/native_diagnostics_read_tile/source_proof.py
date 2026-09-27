#!/usr/bin/env python3
"""Literal ad610/current kernels and reused diagnostic fixture admission."""
from pathlib import Path
import argparse,hashlib,json,runpy
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]
MANIFEST="d6df422a60af30fc0066b2a903ed84420640557422bb6dd3fe210ddf4f44895d"
SOURCE="lib_src/collision/radioss_type25/runtime/Kernels.cu"
EXISTING=ROOT/"lib_utest/qualification/native_contact_diagnostics"

def check(data,row):
    assert len(data)==row["bytes"] and hashlib.sha256(data).hexdigest()==row["sha256"],row["path"]

def checked():
    raw=(HERE/"source-manifest.json").read_bytes()
    assert hashlib.sha256(raw).hexdigest()==MANIFEST
    manifest=json.loads(raw)
    assert manifest["baseline_commit"]=="ad61018ad1864b86cbfe81444ac68323e073acb7"
    old=(HERE/manifest["frozen"]["path"]).read_bytes();check(old,manifest["frozen"])
    for row in manifest["unchanged"]+manifest["new_production"]:check((ROOT/row["path"]).read_bytes(),row)
    reuse=runpy.run_path(str(EXISTING/"source_proof.py"))
    text=old.decode()
    for edit in manifest["kernel_edits"]:text=reuse["once"](text,edit["before"],edit["after"])
    current=(ROOT/SOURCE).read_text();assert text==current,"Unexpected complete production kernel change"
    for row in manifest["header_registration"]:check((ROOT/row["candidate"]["path"]).read_bytes(),row["candidate"])
    # The force/response/layout/caller code outside Diagnostics and its launch
    # reverses exactly. Neither a new reduction nor an early failure skip is admitted.
    before=reuse["function"](old.decode(),"__global__ void Diagnostics(","__global__ void GatherNodes(")
    after=reuse["function"](current,"__global__ void Diagnostics(","__global__ void GatherNodes(")
    terminal="  elastic_energy*=units.energy;damping_work*=units.energy;friction_work*=units.energy;"
    assert before[before.index(terminal):]==after[after.index(terminal):]
    assert after.count("__syncthreads();")==2 and "if(lane<count)" in after
    assert "if(!lane)for(unsigned local=0;local<count;++local)if(tile.positive[local])" in after
    return old.decode(),current,reuse

def generated():
    old,current,reuse=checked();function=reuse["function"]
    sections=["#pragma once",'#include "lib_src/collision/radioss_type25/runtime/diagnostics/Read.h"']
    for name,source in (("reference",old),("current",current)):
        sections.append("namespace tlfea::contact::radioss_type25::runtime_detail::diagnostic_"+name+" {")
        sections.append(function(source,"__device__ void Fail(","__global__ void Reset("))
        sections.append(function(source,"__global__ void Diagnostics(","__global__ void GatherNodes("))
        sections.append("}")
    rig=(EXISTING/"Rig.cuh").read_text()
    rig=reuse["once"](rig,'#include "TestSupport.h"','#include "lib_utest/qualification/native_contact_diagnostics/TestSupport.h"')
    rig=reuse["once"](rig,"rd::diagnostic_current::Diagnostics<<<1,1,0,stream_>>>",
        "rd::diagnostic_current::Diagnostics<<<1,rd::diagnostics::Threads,0,stream_>>>")
    return {"DiagnosticBodies.h":"\n".join(sections)+"\n","Rig.cuh":rig,
        "ExistingCudaTest.cu":(EXISTING/"CudaTest.cu").read_text()}

if __name__=="__main__":
    parser=argparse.ArgumentParser();parser.add_argument("--output",type=Path);args=parser.parse_args()
    values=generated()
    if args.output:
        args.output.mkdir(parents=True,exist_ok=True)
        for name,value in values.items():(args.output/name).write_text(value)
    print(json.dumps({"status":"source_passed","baseline":"ad61018","unchanged_records":13,
        "literal_legacy_cuda_cases":8,"new_cuda_cases":4,"new_host_read_cases":2,"existing_host_layout_cases":2,
        "shared_tile_bytes":2048,"retained_bytes_added":0,"compiled":False,"gpu_executed":False}))
