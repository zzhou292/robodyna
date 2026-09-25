#!/usr/bin/env python3
"""Pinned real fixed-wall inputs and independently observed expected fields."""
import argparse,hashlib,json,struct
from pathlib import Path
ROOT=Path(__file__).resolve().parent

def prepare():
    data=(ROOT/"evidence/native-observations.jsonl").read_bytes()
    manifest=json.loads((ROOT/"evidence/manifest.json").read_text())
    assert hashlib.sha256(data).hexdigest()==manifest["sha256"]
    rows={x["stage"]:x["observation"] for x in map(json.loads,data.decode().splitlines())}
    main=rows["main"]["arrays"];classification=rows["classification"]["arrays"];boundary=rows["boundary"]["arrays"]
    p=rows["inventory"]["primary_search_count"];g=rows["inventory"]["global_nrtm"]
    assert p==8 and g==16 and len(main["ITAB"])==18
    text=["// Generated qualification evidence; never production source values.","#pragma once","#include <cstdint>","namespace type25_startup_test::observed {"]
    def emit(kind,name,values):
        value=lambda x:float(x).hex() if kind=="double" else str(int(x))
        text.append("inline constexpr "+kind+" "+name+"[]={"+",".join(map(value,values))+"};")
    text.append("inline constexpr int InputCycle="+str(rows["main"]["clock"]["NCYCLE"])+";")
    text.append("inline constexpr int ClassificationCycle="+str(rows["classification"]["clock"]["NCYCLE"])+";")
    text.append("inline constexpr double ClassificationTime="+float(rows["classification"]["clock"]["TT"]).hex()+";")
    emit("double","Positions",main["X"]);emit("std::uint64_t","Ids",main["ITAB"])
    emit("std::uint32_t","PrimaryNodes",[x-1 for x in classification["IRECT"][:4*p]])
    emit("int","Connectivity",classification["IRECT"]);emit("int","Roles",classification["MSEGTYP"])
    emit("int","Globals",boundary["MSEGLO"]);emit("int","Neighbors",classification["MVOISIN"])
    emit("int","References",classification["ADMSR"]);emit("int","Bounds",classification["LBOUND"])
    bits=lambda x:struct.unpack("<I",struct.pack("<f",x))[0]
    emit("std::uint32_t","NormalBits",[bits(x) for x in classification["NOD_NORMAL"]])
    emit("std::uint32_t","BisectorBits",[bits(x) for x in boundary["VTX_BISECTOR"]])
    text.append("} // namespace type25_startup_test::observed")
    return "\n".join(text)+"\n"
if __name__=="__main__":
    parser=argparse.ArgumentParser();parser.add_argument("--output",type=Path,required=True);parser.add_argument("--check",action="store_true")
    args=parser.parse_args();text=prepare()
    if args.check:assert args.output.read_text()==text
    else:args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(text)
    print("Pinned observed18-node/8-primary fixed-wall evidence prepared")
