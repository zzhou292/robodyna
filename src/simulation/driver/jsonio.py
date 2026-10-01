"""Bounded, unambiguous JSON for operator-owned manifests."""

import json
import math
from pathlib import Path

MAX_MANIFEST_BYTES = 1024 * 1024


def require(condition, message):
    if not condition:
        raise ValueError(message)


def _object(pairs):
    result = {}
    for key, value in pairs:
        require(key not in result, f"duplicate JSON field: {key}")
        result[key] = value
    return result


def _nonfinite(value):
    raise ValueError(f"nonfinite JSON: {value}")


def read_json(path, byte_cap=MAX_MANIFEST_BYTES):
    path = Path(path)
    require(path.is_file() and not path.is_symlink(), f"regular manifest required: {path}")
    with path.open("rb") as stream:
        data = stream.read(byte_cap + 1)
    require(len(data) <= byte_cap, f"JSON exceeds {byte_cap} bytes")
    return json.loads(data, object_pairs_hook=_object, parse_constant=_nonfinite)


def write_new(path, value, byte_cap=MAX_MANIFEST_BYTES):
    encoded = (json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n").encode()
    require(len(encoded) <= byte_cap, f"output JSON exceeds {byte_cap} bytes")
    with Path(path).open("xb") as stream:
        stream.write(encoded)


def fields(value, required, optional=()):
    require(type(value) is dict, "JSON object required")
    require(set(required) <= value.keys(), f"missing fields: {sorted(set(required) - value.keys())}")
    require(value.keys() <= set(required) | set(optional),
            f"unknown fields: {sorted(value.keys() - set(required) - set(optional))}")


def integer(value, name, low=0, high=(1 << 63) - 1):
    require(type(value) is int and low <= value <= high, f"{name} must be an integer in [{low}, {high}]")
    return value


def real(value, name, low=0, high=float("inf"), positive=False):
    require(type(value) in (int, float) and math.isfinite(value) and low <= value <= high
            and (not positive or value > 0), f"invalid {name}")
    return float(value)


def boolean(value, name):
    require(type(value) is bool, f"{name} must be true or false")
    return value


def resolve(base, text):
    require(type(text) is str and 0 < len(text) <= 4096 and "\x00" not in text, "invalid input path")
    path = Path(text).expanduser()
    return (base / path).absolute() if not path.is_absolute() else path
