"""Whole-part geometry composition using the existing authenticated reader.

Offline assembly limits are separate from the unchanged single-part limits and
all TL mechanics capacities. Parts are read serially; no global solver exists.
"""
from dataclasses import dataclass

from ._legacy import require
from .canonical_geometry import load_part_geometry


@dataclass(frozen=True)
class AssemblyGeometry:
    parts: tuple
    source_node_ids: tuple
    shared_node_ids: tuple
    shell_count: int
    qeph_parent_count: int
    t3_parent_count: int


def load_assembly_geometry(asset_dir, archive_path, declarations, reference, limits):
    parts, nodes, shared, elements = [], set(), set(), set()
    total = quads = triangles = 0
    for declaration in declarations:
        # The existing reader consumes named offline limits. Its default
        # GeometryLimits contract stays 256 shells / 512 nodes for legacy users.
        geometry = load_part_geometry(asset_dir, archive_path, declaration, reference, limits=limits)
        part_nodes = {n.source_id for n in geometry.nodes}
        shared.update(nodes & part_nodes)
        nodes.update(part_nodes)
        require(len(nodes) <= limits.nodes, 'assembly node cap exceeded')
        for shell in geometry.shells:
            require(shell.source_id not in elements, 'duplicate assembly shell ID')
            elements.add(shell.source_id)
            total += 1
            require(total <= limits.shells, 'assembly shell cap exceeded')
            quads += shell.arity == 4
            triangles += shell.arity == 3
        parts.append(geometry)
    require(parts, 'empty assembly geometry')
    identities = {(p.archive_sha256, p.member_sha256, p.canonical_manifest_sha256) for p in parts}
    require(len(identities) == 1, 'assembly parts have inconsistent source provenance')
    return AssemblyGeometry(tuple(parts), tuple(sorted(nodes)), tuple(sorted(shared)), total, quads, triangles)
