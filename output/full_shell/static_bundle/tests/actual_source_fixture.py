#!/usr/bin/env python3
"""Bounded original member fixture; no keyword parser or mechanics admission."""
import hashlib
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import zipfile

canonical, scope, binary = map(Path, sys.argv[1:])
with tempfile.TemporaryDirectory(prefix="robo-static-source-") as temporary:
    member = Path(temporary) / "yaris-coarse-v1l.key"
    digest, size = hashlib.sha256(), 0
    with zipfile.ZipFile(canonical / "source_model.zip") as archive:
        name = "2010-toyota-yaris-coarse-v1l/yaris-coarse-v1l.key"
        if archive.namelist().count(name) != 1 or archive.getinfo(name).file_size != 42846753:
            raise RuntimeError("Original source member identity/size changed")
        with archive.open(name) as source, member.open("xb") as output:
            while data := source.read(1024 * 1024):
                size += len(data)
                if size > 42846753:
                    raise RuntimeError("Source fixture exceeded exact byte cap")
                digest.update(data)
                output.write(data)
    if size != 42846753 or digest.hexdigest() != "67208317e6c8eb1dd43b80001508915ccaace7bc0a745e1aa5a3b33f394df301":
        raise RuntimeError("Original source fixture bytes differ")
    env = dict(os.environ, ROBO_STATIC_CANONICAL=str(canonical), ROBO_STATIC_SCOPE=str(scope), ROBO_STATIC_MEMBER=str(member))
    result = subprocess.run([str(binary), "--gtest_filter=SourceMappingActual.*"], env=env)
    raise SystemExit(result.returncode)
