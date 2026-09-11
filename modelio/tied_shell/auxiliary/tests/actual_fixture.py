"""Authenticate one bounded auxiliary member, then reuse the main source fixture."""
import hashlib
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import zipfile

canonical, scope, binary = map(Path, sys.argv[1:4])
name = '2010-toyota-yaris-coarse-v1l/set-yaris-coarse-v1l.key'
with tempfile.TemporaryDirectory(prefix='robo-tied-auxiliary-') as temporary:
    target = Path(temporary) / 'set-yaris-coarse-v1l.key'
    with zipfile.ZipFile(canonical / 'source_model.zip') as archive:
        if archive.namelist().count(name) != 1 or archive.getinfo(name).file_size != 44991:
            raise RuntimeError('Original auxiliary source size/identity changed')
        data = archive.read(name)
    if hashlib.sha256(data).hexdigest() != 'b93d5370a899f6f70299ea61cd55142c1f8b765b8ab7f9ac979d078486028929':
        raise RuntimeError('Original auxiliary source bytes changed')
    with target.open('xb') as stream:
        stream.write(data)
    helper = Path(__file__).resolve().parents[4] / 'output/full_shell/static_bundle/tests/actual_source_fixture.py'
    result = subprocess.run([sys.executable, '-B', str(helper), str(canonical), str(scope),
        str(binary), 'TiedAuxiliaryActual.*'], env=dict(os.environ, ROBO_TIED_AUX_MEMBER=str(target)))
    raise SystemExit(result.returncode)
