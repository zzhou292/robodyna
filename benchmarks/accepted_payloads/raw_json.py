"""Read original top-level JSON value spellings for exact numerical comparison."""
import json


def _unique(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate JSON key: {key}")
        result[key] = value
    return result


def _nonfinite(value):
    raise ValueError(f"nonfinite JSON constant: {value}")


def read_object(path, max_bytes=None):
    if max_bytes is None:
        text = path.read_text(encoding="utf-8")
    else:
        if type(max_bytes) is not int or max_bytes <= 0:
            raise ValueError("positive JSON byte cap required")
        with path.open("rb") as stream:
            data = stream.read(max_bytes + 1)
        if len(data) > max_bytes:
            raise ValueError("JSON byte cap exceeded")
        text = data.decode("utf-8")
    decoder = json.JSONDecoder(object_pairs_hook=_unique, parse_constant=_nonfinite)
    values = decoder.decode(text)
    if not isinstance(values, dict):
        raise ValueError("summary must be a JSON object")
    raw = {}
    pos = text.index("{") + 1
    for _ in values:
        while text[pos].isspace():
            pos += 1
        key, pos = decoder.raw_decode(text, pos)
        while text[pos].isspace():
            pos += 1
        if text[pos] != ":":
            raise ValueError("missing JSON colon")
        pos += 1
        while text[pos].isspace():
            pos += 1
        start = pos
        _, pos = decoder.raw_decode(text, pos)
        raw[key] = text[start:pos]
        while text[pos].isspace():
            pos += 1
        if text[pos] == ",":
            pos += 1
    return values, raw
