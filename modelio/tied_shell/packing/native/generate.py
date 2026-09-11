"""Verify pinned sources and stage narrow native packing routines for tests."""
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parent


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def source(path):
    entry = next(row for row in MANIFEST['sources'] if row['source'] == path)
    raw = (ROOT / 'original' / path).read_bytes()
    require(len(raw) == entry['bytes'], 'Native source size changed: ' + path)
    require(hashlib.sha256(raw).hexdigest() == entry['sha256'], 'Native source SHA-256 changed: ' + path)
    require(hashlib.sha1(b'blob ' + str(len(raw)).encode() + b'\0' + raw).hexdigest() == entry['git_blob_sha1'],
            'Native source Git blob changed: ' + path)
    return raw.decode()


def between(text, first, last):
    require(text.count(first) == 1 and text.count(last) == 1, 'Native source extraction boundary changed')
    return text[text.index(first):text.index(last)]


def routine(text):
    text = text[text.index('      SUBROUTINE '):]
    # Integer-only routines: native common variables come from a small test
    # module. Printing is disabled; a native warning/error is a test failure.
    lines = []
    injected = False
    for line in text.splitlines():
        if not injected and line.lstrip().upper().startswith('USE '):
            lines.append('      USE PACKING_CONTEXT')
            injected = True
        if line.startswith('#include'):
            if 'implicit_f.inc' in line:
                lines.append('      IMPLICIT NONE')
            continue
        lines.append(line)
    require(injected, 'Missing native context insertion boundary')
    return '\n'.join(lines) + '\n'


MANIFEST = json.loads((ROOT / 'source-manifest.json').read_text())
require(MANIFEST['revision'] == 'a62b27e6baa555d222a580d6218867d0be4d70b5', 'Native revision changed')
destination = Path(sys.argv[1])
destination.mkdir(parents=True, exist_ok=True)
base = 'starter/source/interfaces/interf1/'
for name in ('inpoint', 'insurf', 'count3'):
    (destination / (name + '.F')).write_text(routine(source(base + name + '.F')))
sets = 'starter/source/model/sets/'
(destination / 'shell_surface_buffer.F').write_text(routine(source(sets + 'shell_surface_buffer.F')))
text = source(sets + 'create_surface_from_element.F')
surface_sort = between(text, '      NIX = 6\n', '!---\n!     clause surf allocation')
(destination / 'SurfaceSort.inc').write_text(surface_sort)
text = source(sets + 'create_node_from_element.F')
node_sort = between(text, '      LIMIT = NUMNOD/2', '! Decide whether the result is stored')
(destination / 'NodeSort.inc').write_text(node_sort)
# Complete original C radix implementation; no production sorting helper.
(destination / 'my_orders.c').write_text(source('common_source/tools/sort/my_orders.c'))
print('Verified', len(MANIFEST['sources']), 'pinned packing sources')
