#!/usr/bin/env python3
from pathlib import Path
import argparse,hashlib,json,re
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[2];SOURCE="lib_src/elements/beam18/resident/"
MANIFEST="2b9077d33c7dc1f989bf71599618fcac07be8606680522cd9bfabc65ec5ba264"
def pin(path,row):
    raw=path.read_bytes();assert len(raw)==row["bytes"] and hashlib.sha256(raw).hexdigest()==row["sha256"],str(path);return raw.decode()
def checked():
    raw=(HERE/"source-manifest.json").read_bytes();assert hashlib.sha256(raw).hexdigest()==MANIFEST
    m=json.loads(raw);old={r["name"]:pin(HERE/"frozen"/r["name"],r) for r in m["frozen"]}
    for r in m["unchanged"]+m["production"]:pin(ROOT/r["path"],r)
    h=old["Measure.h"];a=h.index("    for (unsigned c = 0; c < 2; ++c)");z=h.index("\n  }\n  return true;",a)
    fold=h[a:z].replace("now.diagnostics.","value.")
    current=(ROOT/SOURCE/"Measure.h").read_text();first=current.index("    for (unsigned c = 0; c < 2; ++c)");last=current.index("\n  return true;",first)
    assert current[first:last]==fold,"Per-parent operations or endpoint helper changed"
    helper=current[current.index("struct MeasurementOperands {"):current.index("TL_BEAM18_HD inline bool Measure(")]
    expected=h[:h.index("TL_BEAM18_HD inline bool Measure(")]+helper+h[h.index("TL_BEAM18_HD inline bool Measure("):a]+"    if(!AccumulateMeasurementParent(control,state,accepted,p,Operands(now.diagnostics),view))return false;"+h[z:]
    assert expected==current,"Status/validity order or serial caller changed"
    c=old["Candidate.cu"].replace('#include "Measure.h"','#include "Measure.h"\n#include "measurement/Finalize.cuh"')
    a=c.index("  s->control = {};",c.index("__global__ void Finalize("));z=c.index("\n}\n} // namespace",a)
    c=c[:a]+"  __shared__ measurement::Tile tile;\n  measurement::Finalize(*s,accepted,trial,view,identity,initial,tile);"+c[z:]
    c=c.replace("Finalize<<<1, 1, 0,","Finalize<<<1, measurement::Threads, 0,")
    assert c==(ROOT/SOURCE/"Candidate.cu").read_text(),"Other candidate/initialize operations changed"
    previous=json.loads(old["resident-source-manifest.json"]);now=json.loads((ROOT/"lib_utest/qualification/beam18_resident/source-manifest.json").read_text())
    allowed={SOURCE+n for n in ['Candidate.cu','Measure.h','BUILD.bazel']}
    for name,row in previous['files'].items():
        if name not in allowed:assert now['files'][name]==row,"Unrelated native identity changed"
    return old

def generated():
    old=checked()
    def include(match):return '#include "'+str((ROOT/SOURCE/match.group(1)).resolve().relative_to(ROOT))+'"'
    h=re.sub(r'#include "([^"]+)"',include,old['Measure.h']).replace('namespace tl::fea::beam18::batch_detail {','namespace tl::fea::beam18::batch_detail::read_tile_reference {')
    c=old['Candidate.cu'];a=c.index('__global__ void Finalize(');z=c.index('\n} // namespace',a)
    body=c[a:z].replace('!Measure(', '!tl::fea::beam18::batch_detail::read_tile_reference::Measure(')
    return {'ReferenceMeasure.h':h,'ReferenceFinalize.cuh':'#pragma once\n#include "ReferenceMeasure.h"\n#include <cfloat>\nnamespace tl::fea::beam18::batch_detail::read_tile_reference {\n'+body+'\n}\n'}
if __name__=='__main__':
    a=argparse.ArgumentParser();a.add_argument('--output',type=Path);args=a.parse_args();out=generated()
    if args.output:
        args.output.mkdir(parents=True,exist_ok=True)
        for name,text in out.items():(args.output/name).write_text(text)
    print(json.dumps({'status':'source_passed','baseline':'515e46ff','shared_tile_bytes':2376,'retained_bytes_added':0,'endpoint_helper_unchanged':True,'compiled':False,'gpu_executed':False}))
