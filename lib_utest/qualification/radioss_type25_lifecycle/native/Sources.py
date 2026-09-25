"""Pinned lifecycle donors, reusing the established extraction/constant parser."""
import hashlib
import importlib.util
import json
from pathlib import Path
ROOT = Path(__file__).resolve().parent
HELPER = ROOT.parent.parent / "radioss_type25_selection/native/Sources.py"
spec = importlib.util.spec_from_file_location("lifecycle_selection_source", HELPER)
selection = importlib.util.module_from_spec(spec)
spec.loader.exec_module(selection)
routine = selection.routine
constants = selection.constants


def read():
    result = {}
    for entry in json.loads((ROOT / "source-manifest.json").read_text())["files"]:
        path = ROOT.parent / entry["path"]
        data = path.read_bytes()
        if len(data) != entry["bytes"] or hashlib.sha256(data).hexdigest() != entry["sha256"]:
            raise RuntimeError("Pinned lifecycle donor changed: " + entry["path"])
        result[path.name] = data.decode()
    return result
