"""Pinned native activation donors; no production numerical implementation read."""
import hashlib
import importlib.util
import json
import re
from pathlib import Path
ROOT=Path(__file__).resolve().parent
HELPER=ROOT.parent.parent/"radioss_type25_selection/native/Sources.py"
spec=importlib.util.spec_from_file_location("startup_selection_source",HELPER)
selection=importlib.util.module_from_spec(spec);spec.loader.exec_module(selection)
constants=selection.constants


def read():
    result={}
    for entry in json.loads((ROOT/"source-manifest.json").read_text())["files"]:
        data=(ROOT.parent/entry["path"]).read_bytes()
        assert len(data)==entry["bytes"] and hashlib.sha256(data).hexdigest()==entry["sha256"]
        assert hashlib.sha1(b"blob "+str(len(data)).encode()+b"\0"+data).hexdigest()==entry["git_blob"]
        result[Path(entry["path"]).name]=data.decode()
    return result


def routine(source,name):
    start=re.search(r"^      (?:SUBROUTINE|LOGICAL FUNCTION) "+name+r"[ \t]*\(",source,re.M)
    assert start,name
    tail=source[start.start():];end=re.search(r"^      END[ \t]*$",tail,re.M)
    assert end,name
    return tail[:end.end()]+"\n"


def between(source,first,last):
    assert source.count(first)==1,first
    tail=source.split(first,1)[1]
    assert last in tail,last
    return first+tail.split(last,1)[0]
