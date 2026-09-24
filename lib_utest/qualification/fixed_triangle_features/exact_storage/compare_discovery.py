#!/usr/bin/env python3
"""Run separately linked discovery adapters and retain exact fieldwise evidence."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import tempfile
import time

OUTPUT_CAP = 16 * 1024 * 1024
ERROR_CAP = 1024 * 1024
TIMEOUT_SECONDS = 120


def require(value, message):
    if not value:
        raise ValueError(message)


def executable(argument):
    """Accept CMake absolute paths and ordinary Bazel runfile logical paths."""
    supplied = Path(argument)
    candidates = [supplied]
    logical = [argument]
    if not supplied.is_absolute():
        for workspace in (os.environ.get('TEST_WORKSPACE'), '_main'):
            if workspace:
                logical.append(workspace + '/' + argument)
        runfiles = os.environ.get('RUNFILES_DIR') or os.environ.get('TEST_SRCDIR')
        if runfiles:
            candidates.extend(Path(runfiles) / name for name in logical)
        manifest = os.environ.get('RUNFILES_MANIFEST_FILE')
        if manifest:
            with Path(manifest).open(encoding='utf-8') as stream:
                for row in stream:
                    key, separator, physical = row.rstrip('\n').partition(' ')
                    if separator and key in logical:
                        candidates.append(Path(physical))
    for candidate in candidates:
        if candidate.is_file() and os.access(candidate, os.X_OK):
            return candidate.resolve()
    raise ValueError('Cannot resolve executable or Bazel runfile: ' + argument)


def digest(path):
    value = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            value.update(block)
    return value.hexdigest()


def run(binary, label, directory):
    output = directory / (label + '.jsonl')
    error = directory / (label + '.stderr')
    reason = None
    started = time.monotonic()
    with output.open('xb') as stdout, error.open('xb') as stderr:
        process = subprocess.Popen([str(binary)], stdout=stdout, stderr=stderr)
        try:
            while process.poll() is None:
                if output.stat().st_size > OUTPUT_CAP or error.stat().st_size > ERROR_CAP:
                    reason = 'bounded output capacity exceeded'
                    process.kill()
                    break
                if time.monotonic() - started > TIMEOUT_SECONDS:
                    reason = 'driver timeout'
                    process.kill()
                    break
                time.sleep(0.02)
            code = process.wait(timeout=10)
        finally:
            if process.poll() is None:
                process.kill()
                process.wait(timeout=10)
    if output.stat().st_size > OUTPUT_CAP or error.stat().st_size > ERROR_CAP:
        reason = 'bounded output capacity exceeded'
    return {'binary': str(binary), 'binary_sha256': digest(binary),
            'exit_code': code, 'reason': reason,
            'output': output.name, 'output_bytes': output.stat().st_size,
            'output_sha256': digest(output), 'stderr': error.name,
            'stderr_bytes': error.stat().st_size,
            'elapsed_seconds': time.monotonic() - started}


def read_complete(path):
    with path.open('rb') as stream:
        data = stream.read(OUTPUT_CAP + 1)
    require(len(data) <= OUTPUT_CAP, 'Driver output exceeds comparison bound')
    lines = data.splitlines()
    require(len(lines) > 2, 'Missing discovery parity records')
    header, footer = json.loads(lines[0]), json.loads(lines[-1])
    require(header.get('schema') == 'fixed_triangle_discovery_parity.v1', 'Wrong discovery parity schema')
    require(footer.get('complete') is True, 'Driver did not finish its complete corpus')
    require(type(footer.get('records')) is int and footer['records'] == len(lines) - 2,
            'Discovery record count differs from completed stream')
    require(footer['records'] >= 200, 'Expected bounded permutation/recovery corpus did not execute')
    for line in lines[1:-1]:
        record = json.loads(line)
        require(set(record) == {'scenario', 'workers', 'permutation', 'action', 'report', 'publication'},
                'Malformed discovery comparison record')
        require(record['workers'] in (1, 4), 'Unexpected worker profile')
        for channel in ('features', 'intersections'):
            view = record['publication'][channel]
            require(type(view['complete']) is bool and view['count'] == len(view['values']),
                    'View completeness field or serialized extent is invalid')
    return data, footer['records']


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--adaptive', required=True)
    parser.add_argument('--wide', required=True)
    parser.add_argument('--output-dir', required=True, type=Path)
    args = parser.parse_args()
    adaptive, wide = executable(args.adaptive), executable(args.wide)
    require(adaptive != wide, 'Parity requires two separately linked executables')
    require(not args.output_dir.is_symlink(), 'Output directory must not be a symlink')
    args.output_dir.mkdir(parents=True, exist_ok=True)
    require(args.output_dir.is_dir(), 'Output root is not a directory')
    # A fresh child allows repeat CTest/Bazel invocations without overwriting
    # earlier successful or failed evidence. The parent chooses retention.
    directory = Path(tempfile.mkdtemp(prefix='discovery-parity-', dir=args.output_dir))
    result = {'schema': 'fixed_triangle_discovery_parity_comparison.v1',
              'scope': 'separate adaptive/wide adapters, unchanged discovery/geometry, exact fieldwise output; not a performance benchmark',
              'directory': str(directory.resolve()), 'status': 'failed', 'runs': []}
    try:
        # Run both sequentially even if one driver rejects the corpus, so both
        # complete/partial outputs remain available for diagnosis.
        for binary, label in ((adaptive, 'adaptive'), (wide, 'wide')):
            result['runs'].append(run(binary, label, directory))
        require(all(row['exit_code'] == 0 and row['reason'] is None for row in result['runs']),
                'A discovery parity driver failed; inspect retained outputs')
        a, count_a = read_complete(directory / 'adaptive.jsonl')
        b, count_b = read_complete(directory / 'wide.jsonl')
        result['records'] = {'adaptive': count_a, 'wide': count_b}
        result['exact_bytes_equal'] = a == b
        if a != b:
            first = next((i for i, pair in enumerate(zip(a, b)) if pair[0] != pair[1]), min(len(a), len(b)))
            result['first_differing_byte'] = first
            result['first_differing_line'] = a[:first].count(b'\n') + 1
        require(a == b, 'Adaptive and wide complete discovery output differs')
        result['status'] = 'passed'
    except Exception as error:
        result['error'] = str(error)
    finally:
        with (directory / 'comparison.json').open('x', encoding='utf-8') as stream:
            json.dump(result, stream, indent=2)
            stream.write('\n')
    print(json.dumps(result, indent=2))
    return 0 if result['status'] == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
