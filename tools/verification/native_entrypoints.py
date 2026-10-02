"""Find retained native demo entrypoints by source contents, not filename prefixes."""

import argparse
import collections
import json
from pathlib import Path
import re

from tools.verification.chrono_inventory import cmake_exposure, cmake_index, digest, git, tree_files


NATIVE_SUFFIXES = frozenset((".cpp", ".cc", ".cxx", ".c", ".cu", ".mm"))
_NON_CODE = re.compile(
    r'R"(?P<raw>[^\s()\\]{0,16})\([\s\S]*?\)(?P=raw)"'
    r'|"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'|//[^\n]*|/\*[\s\S]*?\*/')
_ENTRY = re.compile(r'\b(?:int|auto|void)\s+(?:(?:WINAPI|APIENTRY)\s+)?(main|wmain|WinMain|wWinMain)\s*\(')
_BODY = re.compile(r'\s*(?:noexcept(?:\s*\([^)]*\))?\s*)?(?:->\s*int\s*)?(?:try\s*)?\{')


def entrypoints(text):
    """Report lexical definitions, excluding prototypes/comments/quoted examples.

    Conditional branches are retained as source evidence. This is deliberately
    not a C++ compiler or proof that a selected feature profile builds the body.
    """
    code = _NON_CODE.sub(lambda match: re.sub(r'[^\n]', ' ', match[0]), text)
    result = []
    for match in _ENTRY.finditer(code):
        cursor, depth = match.end(), 1
        while cursor < len(code) and depth:
            depth += (code[cursor] == '(') - (code[cursor] == ')')
            cursor += 1
        if depth or not _BODY.match(code, cursor):
            continue
        prefix = code[:match.start()]
        result.append(dict(name=match[1], line=prefix.count('\n') + 1,
                           lexical_brace_depth=prefix.count('{') - prefix.count('}')))
    return result


def audit(repository):
    root = Path(repository).resolve(strict=True)
    sources = json.loads((root / 'docs/migration/SOURCES.json').read_text())
    pin = next(row for row in sources['sources'] if row['component'] == 'chrono_capabilities')
    original = tree_files(root, pin['source_tree'])
    cmakes, parents = cmake_index(root, pin['path'], original)
    programs, support, missing = [], [], []
    for name, identity in sorted(original.items()):
        if not name.startswith('src/demos/') or Path(name).suffix not in NATIVE_SUFFIXES:
            continue
        path = root / pin['path'] / name
        if not path.is_file():
            missing.append(name)
            continue
        raw = path.read_bytes()
        definitions = entrypoints(raw.decode('utf-8', errors='surrogateescape'))
        row = dict(path=pin['path'] + '/' + name, original_git_blob=identity['git_blob'],
                   current_sha256=digest(raw), entrypoints=definitions,
                   cmake=cmake_exposure(name, 'demo' if definitions else 'support', cmakes, parents))
        (programs if definitions else support).append(row)
    return dict(
        schema='robodyna.native_demo_entrypoints.v1',
        audited_head=git(root, 'rev-parse', 'HEAD').decode().strip(), source_pin=pin,
        methodology='Pinned native sources scanned after masking comments and strings; definitions require a body; no configure, compile or execution.',
        summary=dict(native_source_files=len(programs) + len(support),
                     native_program_files=len(programs), support_files=len(support),
                     non_demo_prefixed_programs=sum(not Path(row['path']).name.startswith('demo_') for row in programs),
                     by_module=dict(sorted(collections.Counter(Path(row['path']).parts[5] for row in programs).items()))),
        programs=programs, support=support, missing=missing,
        non_global_scope_candidates=[row['path'] for row in programs if any(x['lexical_brace_depth'] for x in row['entrypoints'])],
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repository', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = audit(args.repository)
    with args.output.open('x') as stream:
        json.dump(result, stream, indent=2)
        stream.write('\n')
    print(json.dumps(result['summary'], sort_keys=True))
    return bool(result['missing'])


if __name__ == '__main__':
    raise SystemExit(main())
