"""Lexical CMake source evidence, deliberately not a configure-time evaluator."""

import re
from pathlib import PurePosixPath


_COMMENT_OR_STRING = re.compile(r'\\.|"(?:\\.|[^"\\])*"|#\[(=*)\[[\s\S]*?\]\1\]|#[^\n]*')
_START = re.compile(r'\b([A-Za-z_][A-Za-z_0-9]*)\s*\(')


def commands(text):
    """Read balanced commands and lexical conditions while preserving source lines."""
    masked = _COMMENT_OR_STRING.sub(
        lambda match: match[0] if match[0].startswith(('"', '\\')) else re.sub(r'[^\n]', ' ', match[0]), text)
    result, conditions, position = [], [], 0
    while match := _START.search(masked, position):
        cursor, depth, quoted = match.end(), 1, False
        while cursor < len(masked) and depth:
            char = masked[cursor]
            if char == '\\':
                cursor += 2
                continue
            if char == '"':
                quoted = not quoted
            elif not quoted:
                depth += (char == '(') - (char == ')')
            cursor += 1
        if depth:
            raise ValueError('Unbalanced CMake command: ' + match[1])
        name, args = match[1].lower(), masked[match.end():cursor - 1].strip()
        line = masked.count('\n', 0, match.start()) + 1
        if name == 'endif':
            if conditions:
                conditions.pop()
        elif name == 'elseif':
            if conditions:
                conditions[-1] = args
        elif name == 'else':
            if conditions:
                conditions[-1] = 'else of (' + conditions[-1] + ')'
        result.append(dict(command=name, arguments=' '.join(args.split()), line=line,
                           conditions=list(conditions)))
        if name == 'if':
            conditions.append(args)
        position = cursor
    return result


def describe(path, text):
    rows = commands(text)
    return {'path': path, 'conditions': [row for row in rows if row['command'] in ('if', 'elseif')],
            'subdirectories': [row for row in rows if row['command'] == 'add_subdirectory'],
            'build_calls': [row for row in rows if row['command'] in
                            ('add_executable', 'add_library', 'build_demos', 'build_utests', 'build_pyutests', 'add_test')],
            'packages': [row for row in rows if row['command'] == 'find_package'],
            'commands': rows}


def source_mentions(description, filename):
    stem = PurePosixPath(filename).stem
    pattern = re.compile(r'(?<![A-Za-z0-9_])' + re.escape(stem) + r'(?![A-Za-z0-9_])')
    return [row for row in description['commands'] if row['command'] not in
            ('message', 'if', 'elseif', 'endif') and pattern.search(row['arguments'])]
