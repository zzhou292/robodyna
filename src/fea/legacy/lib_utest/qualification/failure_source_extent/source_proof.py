#!/usr/bin/env python3
"""Pin immutable family-map ownership and compare the exact source-check bodies."""
from pathlib import Path
import argparse,hashlib,json
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]
SOURCE="lib_src/elements/failure/ShellFailureStorage.cpp"
SIGNATURE="SetupReport FailureHostStorage::CheckActivitySources(unsigned slab,std::size_t count) const noexcept"
BEFORE='  for (std::size_t e = 0; e < count; ++e) {\n    if (!binding_.parent(family_, e)) {\n      return {SetupStatus::InvalidInput, "Incomplete failure readback source"};\n    }\n  }\n'
AFTER='  // Immutable family maps contain every prefix index. parent() returns null\n  // only for a missing binding/family or an index beyond that family\'s extent;\n  // the original loop did not inspect policy or numerical history values.\n  // Keep the empty prefix successful without probing an absent source.\n  if (count && !binding_.parent(family_,count-1)) {\n    return {SetupStatus::InvalidInput, "Incomplete failure readback source"};\n  }\n'

def checked(path,row):
    raw=path.read_bytes()
    assert len(raw)==row["bytes"] and hashlib.sha256(raw).hexdigest()==row["sha256"],str(path)
    return raw.decode()

def extract(text,name):
    start=text.index(SIGNATURE)
    end=text.index("\nSetupReport FailureHostStorage::CheckReadSources(",start)
    signature="SetupReport "+name+"(bool device_,const ShellBatchFailureBinding& binding_,ShellBindingFamily family_,std::size_t count_,unsigned slab,std::size_t count) noexcept"
    return text[start:end].replace(SIGNATURE,signature,1)

def generate():
    meta=json.loads((HERE/"source-manifest.json").read_text())
    assert meta["baseline"]=="92dd532ea815cedbea3942144147f1b2a8c27414"
    old=checked(HERE/"frozen/ShellFailureStorage.cpp.txt",meta["frozen"])
    assert old.count(BEFORE)==1
    new=(ROOT/SOURCE).read_text()
    assert old.replace(BEFORE,AFTER)==new,"Changes outside the availability-prefix predicate"
    for row in meta["unchanged"]:checked(ROOT/row["path"],row)
    return "#pragma once\n#include \"lib_src/elements/failure/ShellFailureStorage.h\"\nnamespace tl::fea::shell_batch_plasticity_detail::extent_reference {\n"+extract(old,"FrozenCheck")+extract(new,"CurrentCheck")+"}\n"

if __name__=="__main__":
    parser=argparse.ArgumentParser();parser.add_argument("--output",type=Path);args=parser.parse_args()
    result=generate()
    if args.output:
        args.output.mkdir(parents=True,exist_ok=True)
        (args.output/"SourceChecks.h").write_text(result)
    print(json.dumps({"status":"source_passed","baseline":"92dd532e","production_files_changed":1,
        "source_map_immutable":True,"compiled":False,"gpu_executed":False}))
