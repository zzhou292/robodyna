#!/usr/bin/env python3
"""Verify pinned donors and prepare the complete, renamed native failure leaf."""
import argparse
import hashlib
import json
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parent


def prepare(output):
    manifest = json.loads((ROOT / 'source-manifest.json').read_text())
    for entry in manifest['sources']:
        raw = (ROOT / entry['path']).read_bytes()
        if len(raw) != entry['bytes'] or hashlib.sha256(raw).hexdigest() != entry['sha256']:
            raise ValueError('Changed failure donor: ' + entry['path'])
        if 'git_blob_sha1' in entry:
            blob = hashlib.sha1(b'blob ' + str(len(raw)).encode() + b'\0' + raw).hexdigest()
            if blob != entry['git_blob_sha1']:
                raise ValueError('Changed pinned failure blob')
    text = (ROOT / 'original/fail_johnson_c.F').read_text()
    text, count = re.subn(r'^      USE CRACKXFEM_MOD\s*$', '', text, flags=re.M)
    if count != 1:
        raise ValueError('Unused native module declaration changed')
    text = re.sub(r'\bFAIL_JOHNSON_C\b', 'SHELL_CONSTANT_FAILURE_REF', text, flags=re.I)
    output.mkdir(parents=True, exist_ok=True)
    path = output / 'NativeFailure.F'
    if not path.exists() or path.read_text() != text:
        path.write_text(text)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    prepare(parser.parse_args().output)
