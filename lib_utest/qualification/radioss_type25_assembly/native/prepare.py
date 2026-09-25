#!/usr/bin/env python3
"""Extract unchanged local ASS0 loops and FOR3 endpoint products for qualification."""
import argparse
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parent


def between(text, start, stop):
    assert text.count(start) == 1, start
    rest = text.split(start, 1)[1]
    assert stop in rest, stop
    return start + rest.split(stop, 1)[0]


def generate():
    donors = {}
    for item in json.loads((ROOT / 'source-manifest.json').read_text())['files']:
        path = ROOT.parent / item['path']
        data = path.read_bytes()
        assert len(data) == item['bytes'] and hashlib.sha256(data).hexdigest() == item['sha256'], path
        donors[path.name] = data.decode()
    endpoint = between(donors['i25for3.F'],
        '      DO I=1,JLT\n        IF(PENE(I) == ZERO)CYCLE\n!!\n',
        'C\nC\nC SPMD: Identification')
    ass0 = donors['i25ass3.F'].split('      SUBROUTINE I25ASS0(', 1)[1]
    main = ass0.split('        ELSE ! NPINCH > 0\n', 1)[1].split('        ENDIF ! NPINCH > 0', 1)[0]
    assert main.count('DO I=1,JLT') == 1 and main.count('ENDDO') == 1
    local_branch = ass0.split('      IF(INTTH == 0 ) THEN\n', 1)[1].split('C       \n      ELSE', 1)[0]
    secondary = between(local_branch + '@END@',
        '         DO I=1,JLT\n           IF(HH(I)==ZERO)CYCLE\n', '@END@')
    # Retain the complete secondary loop, including its unreachable remote arm.
    # The wrapper supplies explicit local NSVG and dummy native-shaped buffers.
    text = (ROOT / 'Wrapper.F.in').read_text()
    for tag, value in {'ENDPOINT': endpoint, 'MAIN': main, 'SECONDARY': secondary}.items():
        assert text.count('@' + tag + '@') == 1
        text = text.replace('@' + tag + '@', value)
    return text


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path)
    parser.add_argument('--check', action='store_true')
    args = parser.parse_args()
    text = generate()
    if args.output:
        if args.check:
            assert args.output.read_text() == text
        else:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(text)
    print('Pinned FOR3 endpoint and complete selected ASS0 loops verified')
