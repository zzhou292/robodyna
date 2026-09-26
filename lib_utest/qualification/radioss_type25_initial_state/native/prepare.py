"""Whole pinned Starter initial-history routines; expected data only."""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parent


def generated(tl):
    helper = tl/'lib_utest/qualification/radioss_type25_selection/native/Sources.py'
    spec = importlib.util.spec_from_file_location('initial_state_sources', helper)
    source_tools = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(source_tools)
    donors = {}
    for entry in json.loads((ROOT/'source-manifest.json').read_text())['files']:
        raw = (ROOT/entry['path']).read_bytes()
        assert len(raw) == entry['bytes']
        assert hashlib.sha256(raw).hexdigest() == entry['sha256']
        assert hashlib.sha1(b'blob '+str(len(raw)).encode()+b'\0'+raw).hexdigest() == entry['git_blob']
        donors[Path(entry['path']).name] = raw.decode()
    original = {name: source_tools.routine(donors[name.lower()+'.F'], name)
                for name in ('I25COR3', 'I25PEN3', 'I25PWR3')}
    output = {}
    replacements = {'I25COR3':'INITIAL_STATE_COR3', 'I25PEN3':'INITIAL_STATE_PEN3',
                    'I25PWR3':'INITIAL_STATE_PWR3', 'MESSAGE_MOD':'INITIAL_STATE_MESSAGES',
                    'NAMES_AND_TITLES_MOD':'INITIAL_STATE_NAMES'}
    for name, body in original.items():
        renamed = body
        for old, new in replacements.items():
            renamed = re.sub(r'\b'+old+r'\b', new, renamed)
        restored = renamed
        for old, new in replacements.items():
            restored = re.sub(r'\b'+new+r'\b', old, restored)
        assert restored == body, 'Only reversible symbol renames are permitted'
        output[name+'.F'] = renamed
    # Discovery copies alone are uppercased; emitted original expressions stay
    # byte-exact, including lower-case native symbols and their association.
    output['Constants.F90'] = source_tools.constants(donors['constant_mod.F'],
        [x.upper() for x in original.values()]).replace('selection_constants','initial_state_constants')
    declaration = '       INTEGER MVSIZ\n       PARAMETER (MVSIZ = 512)\n'
    assert declaration in donors['starter_mvsiz_p.inc']
    output['mvsiz_p.inc'] = declaration
    output['implicit_f.inc'] = ('      USE ISO_C_BINDING\n      USE INITIAL_STATE_CONSTANTS\n'
        '      IMPLICIT NONE\n#define my_real REAL(C_DOUBLE)\n')
    output['scr05_c.inc'] = '      INTEGER, PARAMETER :: IRESP=0\n'
    output['scr03_c.inc'] = '      INTEGER, PARAMETER :: IPRI=1\n'
    output['units_c.inc'] = '      INTEGER, PARAMETER :: IOUT=6\n'
    for name in ('Boundary.F90','Wrapper.F90'):
        output[name] = (ROOT/name).read_text()
    return output


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--tl-root', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--check', action='store_true')
    args = parser.parse_args()
    for name, data in generated(args.tl_root).items():
        target = args.output/name
        if args.check:
            assert target.read_text() == data
        else:
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_text(data)
    print('Whole native Starter COR3/PEN3/PWR3 prepared; no production numerical source read')
