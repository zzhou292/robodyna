"""Parse pinned fixed-form argument order; no guessed per-function stack offsets."""
import re


def arguments(text, name):
    statements = '\n'.join(line[6:] for line in text.splitlines()
                           if len(line) > 6 and line[0] not in 'cC*!#')
    matches = list(re.finditer(r'\bSUBROUTINE\s+'+re.escape(name)+r'\s*\(([^)]*)\)',
                              statements,re.IGNORECASE|re.DOTALL))
    if len(matches) != 1:
        raise ValueError(f'Exactly one pinned signature required:{name}')
    names = tuple(value.strip().upper() for value in matches[0][1].split(','))
    if len(set(names)) != len(names) or any(not re.fullmatch('[A-Z][A-Z0-9_]*',v) for v in names):
        raise ValueError(f'Invalid or ambiguous native ABI:{name}')
    return names


def verify_common_prefix(text, block, names):
    statements='\n'.join(line[6:] for line in text.splitlines()
                         if len(line)>6 and line[0] not in 'cC*!#')
    match=re.search(r'COMMON\s*/'+re.escape(block)+r'/\s*([^\n]*(?:\n\s+[^\n]*)*)',statements,re.I)
    if not match:
        raise ValueError(f'Missing common block:{block}')
    # Only the explicitly listed initial scalar fields are used. A dimensioned
    # field or declaration reached before that prefix causes a strict mismatch.
    prefix=tuple(v.strip().upper() for v in match[1].split(',')[:len(names)])
    if prefix != tuple(names):
        raise ValueError(f'Native common prefix differs:{block}:{prefix}')


def leading_stride(text, name):
    """Literal first dimension of a native assumed-size rank-two declaration."""
    statements='\n'.join(line[6:] for line in text.splitlines()
                         if len(line)>6 and line[0] not in 'cC*!#')
    extents={int(m[1]) for m in re.finditer(r'\b'+re.escape(name)+r'\s*\(\s*([0-9]+)\s*,\s*\*\s*\)',statements,re.I)}
    if len(extents)!=1 or not 0<next(iter(extents))<=16:
        raise ValueError('Ambiguous or absent native row stride:'+name)
    return next(iter(extents))
