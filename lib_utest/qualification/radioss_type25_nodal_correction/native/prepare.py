#!/usr/bin/env python3
"""Extract genuine EightSlot and TYPE24 STIFINT_ICONTROL phases."""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
ROOT=Path(__file__).resolve().parent
PARENT=ROOT.parent.parent/"radioss_type25_coefficients/native/prepare.py"
spec=importlib.util.spec_from_file_location("correction_constants",PARENT)
base=importlib.util.module_from_spec(spec)
spec.loader.exec_module(base)

def generate():
    refs=base.sources()
    pin=json.loads((ROOT/"source-manifest.json").read_text())["files"][0]
    raw=(ROOT/pin["path"]).read_bytes()
    assert len(raw)==pin["bytes"] and hashlib.sha256(raw).hexdigest()==pin["sha256"]
    assert hashlib.sha1(b"blob "+str(len(raw)).encode()+b"\0"+raw).hexdigest()==pin["git_blob_sha1"]
    source=raw.decode()
    initialization=source[source.index("          sfac = -HUGE(sfac)"):source.index("! tet10")]
    tail=source[source.index("          do n=1,ninter"):source.index("          call my_dealloc(itag)")]
    assert initialization.count("stifint(n) =  sfac*stifint(n)")==1
    assert "if (icontr/=1) cycle" in initialization and "itag(n) = 1" in initialization
    assert "if (itag(ns)==0) stifint(ns) =  sfac_max*stifint(ns)" in tail
    # Preserve each original native constant expression and all selected loops.
    constants=[base.constants(refs["constant_mod.F"])]
    for name in ["EM20"]:
        matches=[l for l in refs["constant_mod.F"].splitlines() if "my_real, parameter ::" in l and l.split("::")[1].split("=")[0].strip()==name]
        assert len(matches)==1
        constants.append(matches[0].replace("my_real","REAL(C_DOUBLE)").strip())
    output=(ROOT/"Correction.F90.in").read_text()
    for tag,value in {"CONSTANTS":"\n".join(constants),"INITIAL_MAX":"          sfac_max = one\n","SOLIDS":initialization,"TYPE24":tail}.items():
        assert output.count("@"+tag+"@")==1
        output=output.replace("@"+tag+"@",value)
    assert "          sfac_max = one\n" in source
    return output

if __name__=="__main__":
    parser=argparse.ArgumentParser();parser.add_argument("--output",type=Path);parser.add_argument("--check",action="store_true");args=parser.parse_args()
    source=generate()
    if args.output:
        path=args.output/"Correction.F90"
        if args.check:assert path.read_text()==source
        else:path.parent.mkdir(parents=True,exist_ok=True);path.write_text(source)
    print("Pinned STIFINT_ICONTROL full EightSlot and TYPE24 phases verified")
