#!/usr/bin/env python3
"""Source-authenticated native geometry oracle; no translated numeric formulas."""
from pathlib import Path
import argparse
import hashlib
import json
import re
ROOT = Path(__file__).resolve().parent

def source():
    result = {}
    manifest = json.loads((ROOT / 'source-manifest.json').read_text())
    assert manifest['revision'] == 'a62b27e6baa555d222a580d6218867d0be4d70b5'
    for item in manifest['files']:
        path = ROOT / item['path']
        data = path.read_bytes()
        assert len(data) == item['bytes']
        assert hashlib.sha256(data).hexdigest() == item['sha256']
        assert hashlib.sha1(f'blob {len(data)}\0'.encode()+data).hexdigest() == item['git_blob']
        result[path.name] = data.decode()
    return result

def constants(text):
    names = {'ZERO','ONE','TWO','FOUR','EIGHT','TEN','SIXTY4','HUNDRED',
             'EM20','HALF','FOURTH','ONE_OVER_8','ZEP015625'}
    names.update('EP%02d' % i for i in range(2,21))
    rows = []
    for line in text.splitlines():
        match = re.search(r'my_real, parameter ::\s*(\w+)\s*=', line)
        if match and match[1] in names:
            rows.append(line.replace('my_real','REAL(C_DOUBLE)',1))
    assert len(rows) == len(names)
    return '\n'.join(rows)

def between(text, start, stop):
    at = text.index(start)
    return text[at:text.index(stop,at)]

def generate():
    donor = source()
    normal = donor['norma1.F'].split('      SUBROUTINE NORMA1D(',1)[1]
    normal = between(normal, '      XX13 =', '      RETURN')
    volume = between(donor['volint.F'], '      X17 =', '      RETURN')
    orientation = donor['insol3.F'].split('      SUBROUTINE INSOL3D(',1)[1]
    orientation = between(orientation, '       XS1=ZERO', 'c       IF(NINT>0)')
    # NINV update is part of the original reversal branch; following ANCMSG
    # calls report it only and are outside this numerical value oracle.
    marker = '       NINV = NINV + 1\n'
    assert donor['insol3.F'].split('      SUBROUTINE INSOL3D(',1)[1].count(marker) >= 1
    orientation += marker
    orientation = orientation.replace('CALL NORMA1D(', 'CALL RD_MAIN_NORMA1D(')
    assert 'IF(IR/=0) RETURN' in orientation and 'IF(IC>=2)RETURN' in orientation
    assert 'IF(DDS<ZERO) RETURN' in orientation and 'IRECT(1,I)=IY(2)' in orientation
    text = (ROOT/'Reference.F.in').read_text().replace('@CONSTANTS@', constants(donor['constant_mod.F']))
    for tag, body in [('NORMAL',normal),('VOLUME',volume),('ORIENTATION',orientation)]:
        assert text.count('@'+tag+'@') == 1
        text = text.replace('@'+tag+'@',body)
    return text

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--output',type=Path)
    parser.add_argument('--check',action='store_true')
    args = parser.parse_args()
    text = generate()
    if args.output:
        path = args.output/'Reference.F'
        if args.check:
            assert path.read_text() == text
        else:
            path.parent.mkdir(parents=True,exist_ok=True)
            path.write_text(text)
    print('Pinned NORMA1D, VOLINT and INSOL3D unique-support geometry verified')
