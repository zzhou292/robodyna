#!/usr/bin/env python3
"""Verify that diagnostic work leaves the frozen mechanics sources unchanged."""
import hashlib,json
from pathlib import Path
here=Path(__file__).resolve().parent
root=here.parents[2]
proof=json.loads((here/'source-proof.json').read_text())
for row in proof['unchanged_mechanics']:
    actual=hashlib.sha256((root/row['file']).read_bytes()).hexdigest()
    if actual!=row['sha256']:raise SystemExit('Changed mechanics: '+row['file'])
print('Unchanged mechanics files:',len(proof['unchanged_mechanics']))
