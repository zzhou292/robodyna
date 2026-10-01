"""Read and filter retained source evidence without pretending it is executable."""

import fnmatch
import json
from pathlib import Path

SCHEMA = 'robodyna.chrono_demo_test_inventory.v1'


def read_inventory(path):
    result = json.loads(Path(path).read_text())
    if result.get('schema') != SCHEMA or not isinstance(result.get('files'), list):
        raise ValueError('Unsupported Chrono inventory')
    seen = set()
    for row in result['files']:
        name = row.get('source', '')
        if not name or name in seen or Path(name).is_absolute() or '..' in Path(name).parts:
            raise ValueError('Invalid or duplicate source path')
        seen.add(name)
        if row['bazel']['status'] not in ('source_only', 'runnable_target'):
            raise ValueError('Unknown root build exposure')
        if (row['bazel']['status'] == 'runnable_target') != bool(row['bazel']['targets']):
            raise ValueError('Runnable status requires an actual declared target')
    return result


def select(document, kind=None, module=None, language=None, status=None, pattern='*'):
    result = []
    for row in document['files']:
        if row['kind'] not in ('demo', 'unit_test'):
            continue
        if kind and row['kind'] != kind:
            continue
        if module and row['module'] != module:
            continue
        if language and row['language'] != language:
            continue
        if not fnmatch.fnmatchcase(row['source'], pattern):
            continue
        if status == 'runnable' and row['bazel']['status'] != 'runnable_target':
            continue
        if status == 'pending' and row['bazel']['status'] != 'source_only':
            continue
        if status == 'qualified' and not row['runtime']['evidence']:
            continue
        result.append(row)
    return sorted(result, key=lambda row: (row['kind'], row['module'], row['source']))


def human_rows(rows):
    for row in rows:
        yield f"{row['kind']} | {row['module']} | {row['language']}"
        yield '  source: ' + row['source']
        cmake = row['cmake']
        references = ', '.join(f"{ref['path']}:{ref['line']}" for ref in cmake['source_mentions'])
        yield '  CMake: ' + cmake['status'] + (': ' + references if references else '')
        if cmake['gate_conditions']:
            yield '  gates: ' + '; '.join(cmake['gate_conditions'])
        yield '  root Bazel: ' + (', '.join(row['bazel']['targets']) or 'pending; retained as source only')
        evidence = row['runtime']['evidence']
        if not evidence:
            status = 'none recorded for this original case'
        elif any(item.get('matches_current_source') is False for item in evidence):
            status = 'historical; current source differs from the recorded run'
        elif any(item.get('matches_current_source') is True for item in evidence):
            status = 'recorded for matching source bytes; dependency qualification is separate'
        else:
            status = 'recorded; see the named target and evidence scope'
        yield '  runtime evidence: ' + status
