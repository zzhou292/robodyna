"""Add the small original wall member to existing auxiliary/main fixtures."""
import hashlib
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import zipfile

canonical, scope, declarations, binary = map(Path, sys.argv[1:5])
name = '2010-toyota-yaris-coarse-v1l/wall.key'
with tempfile.TemporaryDirectory(prefix='robo-tied-classification-') as temporary:
    target = Path(temporary) / 'wall.key'
    with zipfile.ZipFile(canonical / 'source_model.zip') as archive:
        if archive.namelist().count(name) != 1 or archive.getinfo(name).file_size != 10604:
            raise RuntimeError('Original wall source extent changed')
        data = archive.read(name)
    if hashlib.sha256(data).hexdigest() != 'ef02a4701b37d27cec81b1f9a02ab555f55ac61f68b070e8b0c18dc23b1d5155':
        raise RuntimeError('Original wall source bytes changed')
    with target.open('xb') as stream:
        stream.write(data)
    helper = Path(__file__).resolve().parents[4] / 'modelio/tied_shell/auxiliary/tests/actual_fixture.py'
    env = dict(os.environ, ROBO_TIED_WALL_MEMBER=str(target), ROBO_VEHICLE_DECLARATIONS=str(declarations))
    result = subprocess.run([sys.executable, '-B', str(helper), str(canonical), str(scope),
                             str(binary), 'TiedClassificationActual.*'], env=env)
    raise SystemExit(result.returncode)
