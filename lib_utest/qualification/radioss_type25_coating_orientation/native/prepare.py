"""Pinned SEG_INS extraction with an explicit observation-only output."""
from pathlib import Path
import argparse
import hashlib
import importlib.util
import json

ROOT = Path(__file__).resolve().parent
HELPER = ROOT.parent.parent / 'radioss_type25_selection/native/Sources.py'
spec = importlib.util.spec_from_file_location('coating_source_helpers', HELPER)
source = importlib.util.module_from_spec(spec)
spec.loader.exec_module(source)


def generate():
    donors = {}
    for entry in json.loads((ROOT / 'source-manifest.json').read_text())['files']:
        data = (ROOT.parent / entry['path']).read_bytes()
        assert len(data) == entry['bytes']
        assert hashlib.sha256(data).hexdigest() == entry['sha256']
        assert hashlib.sha1(b'blob ' + str(len(data)).encode() + b'\0' + data).hexdigest() == entry['git_blob']
        donors[Path(entry['path']).name] = data.decode()
    body = source.routine(donors['i24surfi.F'], 'SEG_INS')
    original = body
    signature = '      SUBROUTINE SEG_INS(IRECT,NDS,NNOD,INS,X)'
    observed = '      SUBROUTINE SEG_INS_OBS(IRECT,NDS,NNOD,INS,X,VOL_OBS)'
    declaration = '     .   X(3,*)\n'
    extra = '     .   X(3,*)\n      my_real VOL_OBS\n'
    ending = '      RETURN\n'
    observation = '      IF (INS /= 0) VOL_OBS=VOL\n      RETURN\n'
    for anchor in (signature, declaration, ending):
        assert body.count(anchor) == 1, anchor
    body = body.replace(signature, observed).replace(declaration, extra).replace(ending, observation)
    # Removing instrumentation must recover every original routine byte.
    assert body.replace(observed, signature).replace(extra, declaration).replace(observation, ending) == original
    constants = source.constants(donors['constant_mod.F'], [original]).replace('selection_constants', 'coating_constants')
    return {'Constants.F90': constants, 'SegIns.F': body,
            'implicit_f.inc': '      USE ISO_C_BINDING\n      USE COATING_CONSTANTS\n      IMPLICIT NONE\n#define my_real REAL(C_DOUBLE)\n',
            'Wrapper.F90': (ROOT / 'Wrapper.F90').read_text()}


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path)
    parser.add_argument('--check', action='store_true')
    args = parser.parse_args()
    for name, text in generate().items():
        if args.output:
            path = args.output / name
            if args.check:
                assert path.read_text() == text
            else:
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(text)
    print('Original SEG_INS numerical body and observation-only edits verified')
