#!/usr/bin/env python3
"""Read-only verification of the exact native phase source closure."""
import hashlib
import json
from pathlib import Path

MANIFEST_SHA256 = "4b633b308d49cf8aadaa986b435843432427e4dc40d3efc44d6fd68c578a9c7b"
FRAME_MANIFEST_SHA256 = "25b66a74fb69ff9e7d921cfa18b449642a2ab8bdb7f4b386f1a2bc3084d5f380"
REVISION = "a62b27e6baa555d222a580d6218867d0be4d70b5"


def verify():
    original = Path(__file__).resolve().parent / "original"
    data = (original / "source-manifest.json").read_bytes()
    if hashlib.sha256(data).hexdigest() != MANIFEST_SHA256:
        raise RuntimeError("Native phase source manifest hash mismatch")
    manifest = json.loads(data)
    if manifest["revision"] != REVISION or len(manifest["files"]) != 11:
        raise RuntimeError("Native phase revision/inventory mismatch")
    for entry in manifest["files"]:
        path = original / entry["path"]
        content = path.read_bytes()
        blob = hashlib.sha1(b"blob " + str(len(content)).encode() + b"\0" + content).hexdigest()
        if (len(content) != entry["bytes"] or
                hashlib.sha256(content).hexdigest() != entry["sha256"] or
                blob != entry["git_blob_sha1"]):
            raise RuntimeError("Native phase source changed: " + entry["path"])
    frame_data = (original / "frame-source-manifest.json").read_bytes()
    if hashlib.sha256(frame_data).hexdigest() != FRAME_MANIFEST_SHA256:
        raise RuntimeError("Native frame source manifest hash mismatch")
    frame_manifest = json.loads(frame_data)
    if frame_manifest["revision"] != REVISION or len(frame_manifest["files"]) != 4:
        raise RuntimeError("Native frame revision/inventory mismatch")
    for entry in frame_manifest["files"]:
        content = (original / entry["path"]).read_bytes()
        blob = hashlib.sha1(b"blob " + str(len(content)).encode() + b"\0" + content).hexdigest()
        if (len(content) != entry["bytes"] or
                hashlib.sha256(content).hexdigest() != entry["sha256"] or
                blob != entry["git_blob_sha1"]):
            raise RuntimeError("Native frame source changed: " + entry["path"])
    return {"status": "passed", "files": 15, "phase_files": 11, "frame_files": 4,
            "revision": REVISION, "manifest_sha256": MANIFEST_SHA256,
            "frame_manifest_sha256": FRAME_MANIFEST_SHA256}


if __name__ == "__main__":
    print(json.dumps(verify(), sort_keys=True))
