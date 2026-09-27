"""Authenticate full native controlled sources and namespace identifiers only.

No selected numerical expression, branch, array leading dimension or floor is
rewritten. Existing qualified HEPH constant/precision/common context is reused.
"""
from pathlib import Path
import argparse,hashlib,json,re
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3]
MANIFEST="d116c65b17b99543f342c076217c49f1fbe76ba102d88c55ae7ee02eaceefe28"

def verify_bytes(raw,row):
    assert len(raw)==row["bytes"] and hashlib.sha256(raw).hexdigest()==row["sha256"],row["path"]
    assert hashlib.sha1(b"blob "+str(len(raw)).encode()+b"\0"+raw).hexdigest()==row["git_blob_sha1"],row["path"]

def verified(root=ROOT):
    raw=(HERE/"source-manifest.json").read_bytes();assert hashlib.sha256(raw).hexdigest()==MANIFEST
    meta=json.loads(raw);assert meta["native_revision"]=="a62b27e6baa555d222a580d6218867d0be4d70b5"
    values={}
    for row in meta["owned_native_sources"]+meta["reused_sources"]:
        data=(root/row["path"]).read_bytes();verify_bytes(data,row)
        if row in meta["owned_native_sources"] and row.get("compile",True):values[Path(row["source"]).name]=data.decode("latin1")
    return meta,values

def generated(root=ROOT):
    meta,values=verified(root)
    names=set()
    for text in values.values():
        names.update(m.lower() for m in re.findall(r"^\s*(?:subroutine|module)\s+(\w+)",text,re.M|re.I) if m.lower()!="procedure")
    reused={"constant_mod","precision_mod","szstrainhg"}
    pattern=re.compile(r"\b(?:"+"|".join(sorted(names|reused,key=len,reverse=True))+r")\b",re.I)
    result={}
    for name,text in values.items():
        assert "IC1_NATIVE_" not in text and "HEPH_NATIVE_" not in text
        converted=pattern.sub(lambda m:("HEPH_NATIVE_" if m.group().lower() in reused else "IC1_NATIVE_")+m.group(),text)
        # Reversal authenticates that every original byte remains after removing
        # only our private identifier prefixes (including original capitalization).
        assert converted.replace("IC1_NATIVE_","").replace("HEPH_NATIVE_","")==text
        if name=="shour_ctl.F90":
            first=converted.index("            eint(i)= eint(i)+dt1*(")
            last=converted.index("/vol0(i)",first)
            expression=converted[first:last].split("eint(i)+",1)[1]
            hook="\n          call IC1_NATIVE_HOUR_WORK(i,"+expression.strip()+")\n"
            end=converted.index("\n",last)
            changed=converted[:end]+hook+converted[end:]
            assert changed.replace(hook,"",1)==converted
            converted=changed
        if name in ("sfor_n2s4.F","sfor_ns2s4.F90"):
            match=re.search(r"^.*fn\(i\)\s*=\s*\(fac\+one\)\*stif0\(i\)\*pene\(i\).*$",converted,re.M|re.I)
            assert match,name
            category=1 if name=="sfor_n2s4.F" else 2
            hook="\n      CALL IC1_NATIVE_GEOMETRY_FORCE("+str(category)+",I,FN(I))"
            changed=converted[:match.end()]+hook+converted[match.end():]
            assert changed.replace(hook,"",1)==converted
            converted=changed
        result[name]=converted
    sources={row.get("source",""):(root/row["path"]).read_text() for row in meta["owned_native_sources"]+meta["reused_sources"]}
    for name,(source,first,last) in meta["statement_slices"].items():
        result[name]="".join(sources[source].splitlines(keepends=True)[first-1:last])
    # Reuse the qualified interface extractor with original leading dimensions.
    import importlib.util
    path=root/"lib_utest/qualification/solid24_force/native/prepare_sources.py"
    spec=importlib.util.spec_from_file_location("heph_interfaces",path);existing=importlib.util.module_from_spec(spec);spec.loader.exec_module(existing)
    interfaces="      MODULE IC1_NATIVE_INTERFACES\n      INTERFACE\n"
    for symbol,file in [("SZHOUR_CTL","szhour_ctl.F"),("S8FOR_DISTOR","s8for_distor.F"),("SCUMU3","scumu3.F")]:
        value=existing.interface(values[file],symbol)
        interfaces+=pattern.sub(lambda m:("HEPH_NATIVE_" if m.group().lower() in reused else "IC1_NATIVE_")+m.group(),value)
    result["ControlledInterfaces.F"]=interfaces+"      END INTERFACE\n      END MODULE\n"
    return meta,result

if __name__=="__main__":
    p=argparse.ArgumentParser();p.add_argument("--output",type=Path);p.add_argument("--check",action="store_true");a=p.parse_args();meta,result=generated()
    if a.output:
        if not a.check:a.output.mkdir(parents=True,exist_ok=True)
        for name,text in result.items():
            path=a.output/name;raw=text.encode("latin1")
            if a.check:assert path.read_bytes()==raw,name
            else:path.write_bytes(raw)
    print(json.dumps({"status":"source_passed","native_revision":meta["native_revision"],"controlled_sources":len(result),"numerical_statements_unchanged":True,"hourglass_work_observer_only":True,"compiled":False,"executed":False}))
