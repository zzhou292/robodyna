"""Complete independent INCOQ3 and exact native incidence/order blocks."""
from pathlib import Path
import argparse
import hashlib
import importlib.util
import json
ROOT = Path(__file__).resolve().parent

def between(text, start, end):
    assert text.count(start) == 1 and text.count(end) == 1
    return start + text.split(start, 1)[1].split(end, 1)[0]

def generate(tl_root):
    spec = importlib.util.spec_from_file_location('main_support_sources',
        tl_root / 'lib_utest/qualification/radioss_type25_selection/native/Sources.py')
    source = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(source)
    donors = {}
    for entry in json.loads((ROOT / 'source-manifest.json').read_text())['files']:
        data = (ROOT / entry['path']).read_bytes()
        assert len(data) == entry['bytes']
        assert hashlib.sha256(data).hexdigest() == entry['sha256']
        assert hashlib.sha1(b'blob ' + str(len(data)).encode() + b'\0' + data).hexdigest() == entry['git_blob']
        donors[Path(entry['path']).name] = data.decode()
    body = source.routine(donors['incoq3.F'], 'INCOQ3')
    body = body.replace('SUBROUTINE INCOQ3(', 'SUBROUTINE RD_SOURCE_INCOQ3(')
    body = body.replace('use element_mod', 'use main_support_element')
    cnel = donors['build_cnel.F']
    first = cnel.index('      DO K=2,5\n')
    last = cnel.index('      DO I=1,NUMELIG3D\n', first)
    counts = cnel[first:last]
    start = cnel.index('C building the matrix Nod -> Shell element')
    first = cnel.index('      DO K=2,5\n', start)
    last = cnel.index('      DO K=2,3\n', first)
    fill = cnel[first:last]
    assert 'NUMELTG6' in counts and 'NUMELTG6' in fill
    head = donors['cgrhead.F']
    start = head.index('      MODE=0') if '      MODE=0' in head else -1
    # The single source call sorts all eight unsigned words in supplied keys.
    lines = head.splitlines(keepends=True)
    calls = [i for i,x in enumerate(lines) if 'CALL MY_ORDERS' in x]
    assert len(calls) == 1
    i = calls[0]
    order = ''.join(lines[i-2:i+1])
    assert 'MODE' in order and '8)' in order.replace(' ', '')
    outputs = {
        'Incoq.F': body,
        'Constants.F90': source.constants(donors['constant_mod.F'], [body]).replace('selection_constants', 'main_support_constants'),
        'Element.F90': 'module main_support_element\n implicit none\n integer,parameter::nixc=7,nixtg=6\nend module\n',
        'my_orders.c': donors['my_orders.c'],
        'implicit_f.inc': '      USE ISO_C_BINDING\n      USE MAIN_SUPPORT_CONSTANTS\n      IMPLICIT NONE\n#define my_real REAL(C_DOUBLE)\n',
        'com01_c.inc': '      INTEGER NUMNOD\n      COMMON /RD_SUPPORT_NODES/ NUMNOD\n',
        'com04_c.inc': '      INTEGER NUMELC,NUMELTG\n      COMMON /RD_SUPPORT_COUNTS/ NUMELC,NUMELTG\n',
        'param_c.inc': '      INTEGER NPROPG,NPROPGI,NPROPM,IINTTHICK\n      PARAMETER(NPROPG=703,NPROPGI=100,NPROPM=20)\n      COMMON /RD_SUPPORT_THICK/ IINTTHICK\n'}
    for filename, changes in [('Support.F', {'CNEL_COUNTS':counts, 'CNEL_FILL':fill}), ('Order.F', {'ORDER':order})]:
        text = (ROOT / (filename + '.in')).read_text()
        for name, block in changes.items():
            assert text.count('@'+name+'@') == 1
            text = text.replace('@'+name+'@', block)
        outputs[filename] = text
    return outputs

if __name__ == '__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--tl-root',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--check',action='store_true')
    args=parser.parse_args()
    for name,text in generate(args.tl_root).items():
        path=args.output/name
        if args.check: assert path.read_text()==text,name
        else:
            path.parent.mkdir(parents=True,exist_ok=True)
            path.write_text(text)
    print('Complete INCOQ3 and exact BUILD_CNEL/MY_ORDERS blocks authenticated')
