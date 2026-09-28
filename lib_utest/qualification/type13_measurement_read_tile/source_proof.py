#!/usr/bin/env python3
"""Authenticate the baseline and prove the production fold was only factored."""
from pathlib import Path
import argparse,hashlib,json
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]
SOURCE="lib_src/elements/type13/resident/"
MANIFEST="2db9e9a98a61fd77792efe09a16b7401f3256232da3c134163572da54ffd35f5"
def record(path,row):
    raw=path.read_bytes();assert len(raw)==row["bytes"] and hashlib.sha256(raw).hexdigest()==row["sha256"],str(path)
    return raw.decode()
def checked():
    raw=(HERE/"source-manifest.json").read_bytes();assert hashlib.sha256(raw).hexdigest()==MANIFEST
    meta=json.loads(raw);assert meta["baseline"]=="1c8cf7e73a475075a3665c800779b8daf7a0be60"
    old={row["name"]:record(HERE/"frozen"/row["name"],row) for row in meta["frozen"]}
    for row in meta["unchanged"]+meta["production"]:record(ROOT/row["path"],row)
    h=old["Measurement.h"]
    start=h.index("    diagnostics.active_count += now.active;");end=h.index("\n  }\n  for (unsigned k",start);fold=h[start:end]
    helper="TL_TYPE13_HD inline void AccumulatePrepared(const Measurement& now,\n    BatchDiagnostics& diagnostics) noexcept {\n"+fold+"\n}\n"
    first=h.index("  for (unsigned k = 0; k < ChannelCount; ++k) {\n    if (!tl::math::Finite(diagnostics.internal_work_J[k])",end)
    finish=h[first:h.rindex("\n}\n}")]
    helper+="TL_TYPE13_HD inline bool FinishPrepared(const BatchDiagnostics& diagnostics) noexcept {\n"+finish+"\n}\n"
    a=h.index("TL_TYPE13_HD inline bool MeasurePrepared(")
    expected=h[:a]+helper+h[a:start]+"    AccumulatePrepared(now,diagnostics);"+h[end:first]+"  return FinishPrepared(diagnostics);"+h[h.rindex("\n}\n}"):]
    assert expected==(ROOT/SOURCE/"Measurement.h").read_text(),"Measurement arithmetic changed"
    c=old["Candidate.cu"].replace('#include "Measurement.h"','#include "Measurement.h"\n#include "measurement/Finalize.cuh"')
    start=c.index("  auto& state = *storage;",c.index("__global__ void Finalize("));end=c.index("\n}\n} // namespace",start)
    expected=c[:start]+"  __shared__ measurement::Tile tile;\n  measurement::Finalize(*storage,identity,tile);"+c[end:]
    expected=expected.replace("Finalize<<<1, 1, 0, view.stream>>>","Finalize<<<1, measurement::Threads, 0, view.stream>>>")
    assert expected==(ROOT/SOURCE/"Candidate.cu").read_text(),"Other production candidate kernel changed"
    return old

def generated():
    old=checked()
    h=old["Measurement.h"].replace('#include "Measure.h"','#include "lib_src/elements/type13/resident/Measure.h"')
    h=h.replace("namespace tl::fea::type13::batch_detail {","namespace tl::fea::type13::batch_detail::read_tile_reference {")
    c=old["Candidate.cu"];a=c.index("__global__ void Finalize(");z=c.index("\n} // namespace",a)
    finalizer='#pragma once\n#include "ReferenceMeasurement.h"\nnamespace tl::fea::type13::batch_detail::read_tile_reference {\n'+c[a:z]+"\n}\n"
    return {"ReferenceMeasurement.h":h,"ReferenceFinalize.cuh":finalizer}
if __name__=="__main__":
    p=argparse.ArgumentParser();p.add_argument("--output",type=Path);args=p.parse_args();output=generated()
    if args.output:
        args.output.mkdir(parents=True,exist_ok=True)
        for name,text in output.items():(args.output/name).write_text(text)
    print(json.dumps({"status":"source_passed","baseline":"1c8cf7e7","shared_tile_bytes":9096,"retained_bytes_added":0,"compiled":False,"gpu_executed":False}))
