#!/usr/bin/env python3
"""Freeze the complete baseline fold, status priority and kernel launch."""
from pathlib import Path
import argparse,hashlib,json,re,runpy
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]
SOURCE="lib_src/elements/qbat/mapped/"
REFERENCE="tl::fea::qbat::mapped::read_tile_reference::"
MANIFEST="21e8e0f364f981bd45ca2a76300f43ab1356c71a29a580601bcaaceaa81e33ec"

def once(text,before,after):
    assert text.count(before)==1,before
    return text.replace(before,after)

def record(path,row):
    raw=path.read_bytes()
    assert len(raw)==row["bytes"] and hashlib.sha256(raw).hexdigest()==row["sha256"],str(path)
    return raw.decode()

def checked():
    raw=(HERE/"source-manifest.json").read_bytes()
    assert hashlib.sha256(raw).hexdigest()==MANIFEST
    meta=json.loads(raw)
    assert meta["baseline"]=="92dd532ea815cedbea3942144147f1b2a8c27414"
    old={row["name"]:record(HERE/"frozen"/row["name"],row) for row in meta["frozen"]}
    for row in meta["unchanged"]+meta["production"]:record(ROOT/row["path"],row)
    v=old["MeasurementValues.h"]
    first=v.index("    if (now.active)");last=v.index("\n  }\n  return true;",first)
    fold=v[first:last]
    helper="TL_QBAT_HD inline void AccumulateMeasurementParent(const batch_detail::Model& model,\n    const MeasurementParent& now,std::size_t parent,BatchDiagnostics& d) noexcept {\n"+fold+"\n}\n\n"
    v=once(v,"TL_QBAT_HD inline bool MeasureStagedParents(",helper+"TL_QBAT_HD inline bool MeasureStagedParents(")
    v=once(v,fold+"\n  }\n  return true;","    AccumulateMeasurementParent(model,now,parent,d);\n  }\n  return true;")
    assert v==(ROOT/SOURCE/"MeasurementValues.h").read_text(),"Per-parent arithmetic changed"
    h=old["Measurement.h"]
    a=h.index("TL_QBAT_HD inline void FinalizeMeasurement(")
    start=h.index("  batch_detail::Control next{};",a);end=h.index("  if (next.status==BatchStatus::Success)",start)
    helper="TL_QBAT_HD inline batch_detail::Control BeginMeasurement(\n    const batch_detail::Storage& state,BatchDiagnostics identity) noexcept {\n"+h[start:end]+"  return next;\n}\n"
    h=h[:a]+helper+h[a:start]+"  auto next=BeginMeasurement(state,identity);\n"+h[end:]
    assert h==(ROOT/SOURCE/"Measurement.h").read_text(),"Status prescan or serial caller changed"
    c=once(old["Measurement.cu"],'#include "Measurement.h"','#include "Measurement.h"\n#include "measurement/Finalize.cuh"')
    c=once(c,'  mapped::FinalizeMeasurement(*state,view,identity,blocks);','  __shared__ mapped::measurement::Tile tile;\n  mapped::measurement::Finalize(*state,view,identity,blocks,tile);')
    c=once(c,"FinalizeMapped<<<1,1,0,view.stream>>>","FinalizeMapped<<<1,mapped::measurement::Threads,0,view.stream>>>")
    assert c==(ROOT/SOURCE/"Measurement.cu").read_text(),"Other production kernel changed"
    return old

def includes(text):
    def replace(match):
        path=(ROOT/SOURCE/match.group(1)).resolve().relative_to(ROOT)
        if match.group(1)=="MeasurementValues.h":return '#include "ReferenceMeasurementValues.h"'
        return '#include "'+str(path)+'"'
    return re.sub(r'#include "([^"]+)"',replace,text)

def generated():
    old=checked();out={}
    for name in ("MeasurementValues.h","Measurement.h"):
        text=includes(old[name]).replace("namespace tl::fea::qbat::mapped {","namespace tl::fea::qbat::mapped::read_tile_reference {")
        if name=="Measurement.h":
            text=text.replace("MergeMaximum(maximum,",REFERENCE+"MergeMaximum(maximum,")
            text=text.replace("MeasureStagedParents(state.model,",REFERENCE+"MeasureStagedParents(state.model,")
            text=text.replace("&& FinishMeasurement(","&& "+REFERENCE+"FinishMeasurement(")
            text=text.replace("!MeasureStaged(","!"+REFERENCE+"MeasureStaged(")
        out["Reference"+name]=text
    return out

if __name__=="__main__":
    p=argparse.ArgumentParser();p.add_argument("--output",type=Path);a=p.parse_args()
    out=generated()
    if a.output:
        a.output.mkdir(parents=True,exist_ok=True)
        for name,text in out.items():(a.output/name).write_text(text)
    print(json.dumps({"status":"source_passed","baseline":"92dd532e","shared_tile_bytes":10440,
        "retained_bytes_added":0,"compiled":False,"gpu_executed":False}))
