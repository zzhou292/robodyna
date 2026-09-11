"""Reuse the authenticated archive metadata loader; create-only bounded sidecar."""
from pathlib import Path
from ._legacy import require
from .vehicle_declaration_io import _compile_archive_metadata
from .vehicle_section_resolution import compile_vehicle_section_resolution, _encoded, MAX_BYTES


def compile_archive_vehicle_section_resolution(archive_path, assets, scope_path, *, include_glass=False):
    def compile_selected(index, scope, units, authority):
        return compile_vehicle_section_resolution(index, scope, units, authority, include_glass=include_glass)
    return _compile_archive_metadata(archive_path, assets, scope_path, compile_selected)


def write_vehicle_section_resolution(path, report):
    encoded = _encoded(report)
    require(len(encoded) <= MAX_BYTES, 'vehicle section resolution byte cap exceeded')
    with Path(path).open('xb') as stream:
        stream.write(encoded)
        stream.flush()
