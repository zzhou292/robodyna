#!/usr/bin/env python3
"""Verify authenticated original cards and their exact binary64 SI test arrays."""
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parent

def generated():
    manifest = json.loads((ROOT / 'source-curves.json').read_text())
    lines = ['// Generated from authenticated source cards by verify_curves.py.',
             '// Original stresses are MPa; multiplication by 1e6 converts to Pa.',
             '#pragma once', '#include <array>', '#include <cstdint>',
             'namespace continuation_test {',
             'struct OriginalCurve { std::uint64_t id; const double* x; const double* y; std::uint32_t count; };']
    for entry in manifest['curves']:
        data = (ROOT / entry['path']).read_bytes()
        if len(data) != entry['bytes'] or hashlib.sha256(data).hexdigest() != entry['sha256']:
            raise RuntimeError('Original curve card changed: ' + entry['path'])
        records = [line for line in data.decode('ascii').splitlines()
                   if line and line[0] not in '*$']
        if int(records[0][:10]) != entry['id']:
            raise RuntimeError('Curve ID mismatch')
        points = [tuple(float(v) for v in line.split()) for line in records[1:]]
        if len(points) != entry['count'] or any(len(p) != 2 for p in points):
            raise RuntimeError('Point count mismatch')
        for name, column, scale in [('x', 0, 1.), ('y', 1, 1.e6)]:
            values = ', '.join(repr(p[column] * scale) for p in points)
            lines.append(f'inline constexpr double {name}{entry["id"]}[]{{{values}}};')
    lines.append('inline constexpr std::array<OriginalCurve,3> Curves{{')
    for e in manifest['curves']:
        lines.append(f'  {{{e["id"]}, x{e["id"]}, y{e["id"]}, {e["count"]}}},')
    lines += ['}};', '} // namespace continuation_test', '']
    return '\n'.join(lines).encode()

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--generate', action='store_true')
    args = parser.parse_args()
    path = ROOT / 'OriginalCurves.h'
    expected = generated()
    if args.generate:
        path.write_bytes(expected)
    elif path.read_bytes() != expected:
        raise RuntimeError('Generated test arrays changed')
    print('Verified three complete original curve cards and SI arrays')
