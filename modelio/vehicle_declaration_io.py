"""Existing archive/metadata authentication; no second geometry parser."""
import json
import zipfile
from pathlib import Path
from ._legacy import require, sha256, file_sha256
from .canonical_geometry import read_json
from .source_blocks import scan_declarations, MAX_SOURCE_BYTES
from .yaris_part import _compile_archive_part, _member, REFERENCE, VEHICLE
from .vehicle_declarations import compile_vehicle_declarations, MAX_BYTES


def compile_archive_vehicle_declarations(archive_path, assets, scope_path):
    return _compile_archive_metadata(archive_path, assets, scope_path, compile_vehicle_declarations)


def _compile_archive_metadata(archive_path, assets, scope_path, compiler):
    scope_path = Path(scope_path)
    require(0 < scope_path.stat().st_size <= 32 * 1024 * 1024, 'vehicle scope byte cap exceeded')
    scope_bytes = scope_path.read_bytes(); scope = json.loads(scope_bytes)
    seed, authority = _compile_archive_part(archive_path)
    reference = read_json(REFERENCE)
    manifest = Path(assets) / 'manifest.json'
    require(file_sha256(manifest) == scope['canonical']['manifest_sha256'] and
            scope['source']['archive_sha256'] == authority['source']['archive_sha256'] and
            scope['units'] == as_units(seed.units), 'vehicle source authority mismatch')
    with zipfile.ZipFile(archive_path) as archive:
        member = reference['archive_member_prefix'] + VEHICLE
        _member(archive, member, MAX_SOURCE_BYTES)
        with archive.open(member) as stream: index = scan_declarations(stream, VEHICLE)
    identities = dict(canonical_manifest_sha256=file_sha256(manifest), scope_sha256=sha256(scope_bytes),
        scope_bytes=len(scope_bytes), archive_sha256=authority['source']['archive_sha256'],
        member_sha256=index.sha256, member_bytes=index.source_bytes, tire_policy=scope['tire_policy'])
    return compiler(index, scope, seed.units, identities)


def as_units(units):
    from dataclasses import asdict
    return asdict(units)


def write_vehicle_declarations(path, report):
    encoded = (json.dumps(report, sort_keys=True, separators=(',', ':'), allow_nan=False) + '\n').encode()
    require(len(encoded) <= MAX_BYTES, 'vehicle declaration byte cap exceeded')
    with Path(path).open('xb') as stream:
        stream.write(encoded); stream.flush()
