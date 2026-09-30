"""Exact-width native input fields; reject overflow rather than truncate values."""
import math


def integer(value):
    if type(value) is not int or not -999999999 <= value <= 9999999999:
        raise ValueError('Native integer field exceeds10 columns')
    return f'{value:10d}'


def real(value):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError('Finite native real required')
    text = repr(float(value))  # Shortest binary64 round-trip representation.
    if len(text) > 20 or float(text) != value:
        raise ValueError('Native real cannot round-trip in20 columns')
    return text.rjust(20)


def ints(*values):
    return ''.join(integer(v) for v in values)


def reals(*values):
    return ''.join(real(v) for v in values)


def node_group(identifier, title, nodes):
    return [f'/GRNOD/NODE/{identifier}', title] + [ints(*nodes[i:i+10]) for i in range(0,len(nodes),10)]
