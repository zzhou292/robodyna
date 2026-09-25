#!/usr/bin/env python3
"""Read immutable test evidence only; all searched values are produced at runtime."""
import argparse,hashlib,json,importlib.util
from pathlib import Path
ROOT=Path(__file__).resolve().parent
PARENT=ROOT.parent/"radioss_type25_fixed_main_startup"
def generated():
    data=(PARENT/"evidence/native-observations.jsonl").read_bytes()
    manifest=json.loads((PARENT/"evidence/manifest.json").read_text())
    assert hashlib.sha256(data).hexdigest()==manifest["sha256"]
    rows=list(map(json.loads,data.decode().splitlines()))
    chunks=sorted([r["observation"] for r in rows if r["stage"]=="inventory_chunk"],key=lambda x:x["controls"]["ESHIFT"])
    assert len(chunks)==4 and all(c["clock"]["NCYCLE"]==0 for c in chunks)
    first=chunks[0];a=first["arrays"]
    text=["#pragma once","#include <cstdint>","namespace type25_search_startup_test::observed {"]
    def emit(kind,name,values):
        convert=lambda x:float(x).hex() if kind=="double" else str(int(x))
        text.append("inline constexpr "+kind+" "+name+"[]={"+",".join(map(convert,values))+"};")
    emit("std::uint32_t","SecondaryNodes",[x-1 for x in a["NSV"]])
    emit("double","Stiffness",a["STFN"]);emit("double","SecondaryGaps",a["GAP_S"])
    main=[];extent=[]
    for chunk in chunks:
        assert chunk["controls"]["ESHIFT"]==len(main)
        count=chunk["controls"]["NRTM"]
        assert len(chunk["arrays"]["GAP_M"])==count
        main+=chunk["arrays"]["GAP_M"];extent+=chunk["arrays"]["CURV_MAX"]
        assert all(v==0 for v in chunk["removal_offsets"]) and not chunk["removal_nodes"]
        assert chunk["scalars"]["MARGE"]==first["scalars"]["MARGE"]
    emit("double","PrimaryGaps",main);emit("double","PrimaryExtent",extent)
    text.append("inline constexpr double Margin="+float(first["scalars"]["MARGE"]).hex()+";")
    text.append("}")
    return "\n".join(text)+"\n"
if __name__=="__main__":
    p=argparse.ArgumentParser();p.add_argument("--output",type=Path,required=True);p.add_argument("--check",action="store_true")
    a=p.parse_args();text=generated()
    if a.check:assert a.output.read_text()==text
    else:a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(text)
    print("Cycle0 native search margin/gap/extent/removal evidence prepared")
