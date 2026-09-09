#!/usr/bin/env python3
"""Compile only the pinned Yaris V1l fixed wall into a conforming SI mesh.

This is a scoped preparation tool, not a general LS-DYNA deck reader. It never
runs a solver or downloads assets. It retains exact input bytes and IDs.
"""
import argparse
from collections import Counter, defaultdict
import hashlib
import json
import math
from pathlib import Path
import sys

WALL_SHA256 = 'ef02a4701b37d27cec81b1f9a02ab555f55ac61f68b070e8b0c18dc23b1d5155'
COMBINE_SHA256 = '3e0137cd8c569a71a4281cc307549dc2f20772bc67eac0658ae73682dbe242a2'
ARCHIVE_SHA256 = 'aff8194c456726a678d6cc11f644316ca70f3d9b37c4db622726b7b2985b0451'
MM_TO_M = 0.001
STITCH_TOLERANCE_MM = 1e-7


class WallImportError(ValueError):
    pass


def require(condition, message):
    if not condition:
        raise WallImportError(message)


def sha256(data):
    return hashlib.sha256(data).hexdigest()


def blocks(text):
    """Preserve blank cards, remove comments; retain line numbers for errors."""
    result = []
    current = None
    ended = False
    for number, raw in enumerate(text.splitlines(), 1):
        if raw.lstrip().startswith('$'):
            continue
        line = raw.split('$', 1)[0].rstrip()
        if line.lstrip().startswith('*'):
            require(not ended, f'line {number}: keyword after *END')
            keyword = line.strip().upper()
            current = [keyword, []]
            result.append(current)
            ended = keyword == '*END'
        elif current is not None:
            require(not ended or not line.strip(), f'line {number}: data after *END')
            current[1].append((number, line))
        else:
            require(not line.strip(), f'line {number}: data before keyword')
    require(result and result[0][0] == '*KEYWORD', 'missing *KEYWORD')
    require(ended, 'missing *END')
    return result


def one(source_blocks, keyword, keep_blank=False):
    matches = [data for name, data in source_blocks if name == keyword]
    require(len(matches) == 1, f'expected exactly one {keyword} block')
    return matches[0] if keep_blank else [r for r in matches[0] if r[1].strip()]


def fixed(record, widths, converters, allow_trailing=False):
    number, line = record
    values, start = [], 0
    try:
        for width, convert in zip(widths, converters):
            field = line[start:start + width].strip()
            require(bool(field), f'line {number}: missing fixed-width field')
            value = convert(field.replace('D', 'E').replace('d', 'e'))
            require(not isinstance(value, float) or math.isfinite(value),
                    f'line {number}: nonfinite number')
            values.append(value)
            start += width
    except ValueError as error:
        raise WallImportError(f'line {number}: invalid fixed-width record: {error}') from error
    require(allow_trailing or not line[start:].strip(),
            f'line {number}: unsupported extra fields')
    return values


def parse_wall(text):
    source_blocks = blocks(text)
    allowed = {'*KEYWORD', '*END', '*RIGIDWALL_PLANAR_FINITE_FORCES_ID',
               '*RIGIDWALL_PLANAR_FINITE_ID', '*SET_NODE_GENERAL', '*SET_SEGMENT',
               '*PART', '*SECTION_SHELL', '*MAT_RIGID', '*ELEMENT_SHELL', '*NODE'}
    require(all(name in allowed for name, _ in source_blocks),
            'unsupported keyword in scoped wall file')
    require(len(source_blocks) == len(allowed), 'duplicate or missing wall block')
    nodes = {}
    for record in one(source_blocks, '*NODE'):
        nid, x, y, z = fixed(record, [8, 16, 16, 16], [int, float, float, float])
        require(nid > 0 and nid not in nodes, f'duplicate/invalid node ID {nid}')
        nodes[nid] = (x, y, z)
    quads = {}
    for record in one(source_blocks, '*ELEMENT_SHELL'):
        eid, pid, *ids = fixed(record, [8] * 6, [int] * 6)
        require(eid > 0 and eid not in quads, f'duplicate/invalid shell ID {eid}')
        require(pid == 1001, f'shell {eid}: unexpected wall part {pid}')
        require(len(set(ids)) == 4, f'shell {eid}: repeated corner node')
        require(all(n in nodes for n in ids), f'shell {eid}: missing node reference')
        quads[eid] = tuple(ids)
    require(set(nodes) == set(range(1001, 1063)), 'expected source nodes 1001..1062 (62)')
    require(set(quads) == set(range(1001, 1047)), 'expected source quads 1001..1046 (46)')
    require(len(set(nodes.values())) == len(nodes), 'duplicate vertex coordinates')
    require(all(abs(p[0] - 4600.0) <= 1e-9 for p in nodes.values()),
            'wall must be planar at source X=4600 mm')
    used = {n for ids in quads.values() for n in ids}
    require(used == set(nodes), 'unreferenced wall node')

    segment_records = one(source_blocks, '*SET_SEGMENT')
    require(fixed(segment_records[0], [10], [int]) == [1001], 'unexpected segment set ID')
    segments = [tuple(fixed(r, [10] * 4, [int] * 4)) for r in segment_records[1:]]
    require(len(segments) == 45 and len({frozenset(s) for s in segments}) == 45,
            'expected 45 unique lower-wall segments')
    require({frozenset(s) for s in segments} ==
            {frozenset(quads[e]) for e in range(1001, 1046)},
            'segment set 1001 must match lower 45 source quads')
    part = one(source_blocks, '*PART')
    require(len(part) == 2 and part[0][1].strip() == 'Wall', 'unexpected wall part definition')
    require(fixed(part[1], [10] * 3, [int] * 3) == [1001] * 3,
            'wall part/section/material IDs differ from scoped input')
    section = one(source_blocks, '*SECTION_SHELL')
    require(len(section) == 2, 'missing wall section card')
    require(fixed(section[1], [10] * 4, [float] * 4) == [1.0] * 4,
            'expected 1 mm source display-shell thickness')

    analytic_regions = []
    for keyword, expected_id, expected_count in [
            ('*RIGIDWALL_PLANAR_FINITE_FORCES_ID', 1, 5),
            ('*RIGIDWALL_PLANAR_FINITE_ID', 2, 4)]:
        records = one(source_blocks, keyword)
        require(len(records) == expected_count, f'{keyword}: unexpected card count')
        require(fixed(records[0], [10], [int]) == [expected_id], 'unexpected rigid wall ID')
        require(fixed(records[1], [10] * 3, [int] * 3) == [1, 0, 0],
                'unsupported rigid-wall membership/offset card')
        xt, yt, zt, xh, yh, zh, friction = fixed(records[2], [10] * 7, [float] * 7)
        xe, ye, ze, width, height = fixed(records[3], [10] * 5, [float] * 5)
        require((xt, xh, yh, zh) == (4600.0, 4599.0, yt, zt) and
                (xe, ye, ze) == (xt, yt - 1.0, zt),
                'unsupported analytical wall frame')
        require(friction == 0.6 and width > 0 and height > 0,
                'unexpected wall friction or dimensions')
        analytic_regions.append(dict(source_wall_id=expected_id, friction=friction,
                                     bounds_mm=[[xt, yt - width, zt],
                                                [xt, yt, zt + height]]))
    return dict(nodes=nodes, quads=quads, analytic_regions=analytic_regions)


def parse_transform(text):
    """Read only the single wall include/translation from the pinned combine."""
    source_blocks = blocks(text)
    transform = one(source_blocks, '*DEFINE_TRANSFORMATION')
    require(len(transform) == 2, 'expected one translation, no rotation/other transform')
    transform_id = fixed(transform[0], [10], [int])[0]
    require(transform[1][1][:10].strip() == 'TRANSL', 'only TRANSL is supported')
    translation = fixed((transform[1][0], transform[1][1][10:]),
                        [10] * 3, [float] * 3)
    require(translation == [-4550.0, 0.0, 0.0], 'unexpected Yaris wall translation')
    include = one(source_blocks, '*INCLUDE_TRANSFORM', keep_blank=True)
    require(len(include) == 5 and include[0][1].strip() == 'wall.key',
            'expected single wall.key include with five cards')
    offsets = fixed(include[1], [10] * 7, [int] * 7)
    rigid_offset = fixed(include[2], [10], [int])[0]
    require(offsets == [10000000] * 7 and rigid_offset == 10000000,
            'unexpected wall include ID offsets')
    require(not include[3][1].strip(), 'nondefault include unit factors are unsupported')
    require(fixed(include[4], [10], [int]) == [transform_id] and transform_id == 1000001,
            'missing/mismatched wall transformation ID')
    return dict(translation_mm=translation, transformation_id=transform_id,
                node_id_offset=offsets[0], element_id_offset=offsets[1],
                part_id_offset=offsets[2], rigid_wall_id_offset=rigid_offset)


def cross_yz(a, b, c):
    return (b[1] - a[1]) * (c[2] - a[2]) - (b[2] - a[2]) * (c[1] - a[1])


def edge_points(a_id, b_id, nodes):
    a, b = nodes[a_id], nodes[b_id]
    dy, dz = b[1] - a[1], b[2] - a[2]
    length2 = dy * dy + dz * dz
    require(length2 > 0, 'zero-length source edge')
    points = []
    for nid, p in nodes.items():
        if nid in (a_id, b_id):
            continue
        t = ((p[1] - a[1]) * dy + (p[2] - a[2]) * dz) / length2
        if 0 < t < 1 and abs(cross_yz(a, b, p)) <= STITCH_TOLERANCE_MM * math.sqrt(length2):
            points.append((t, nid))
    return [nid for _, nid in sorted(points)]


def triangulate(parsed):
    nodes, quads = parsed['nodes'], parsed['quads']
    triangles, stitched = [], []
    for eid, corners in sorted(quads.items()):
        require(all(cross_yz(nodes[corners[i - 1]], nodes[corners[i]],
                             nodes[corners[(i + 1) % 4]]) > 0 for i in range(4)),
                f'source quad {eid} is not a convex +X-oriented quadrilateral')
        polygon = []
        for i, a in enumerate(corners):
            b = corners[(i + 1) % 4]
            interior = edge_points(a, b, nodes)
            polygon.extend([a] + interior)
            if interior:
                stitched.append(dict(source_quad_id=eid, source_edge=[a, b],
                                     inserted_source_node_ids=interior))
        # A fan from a suitable original corner suffices for this exact convex
        # wall. Try corners deterministically; reject rather than emit zero-area
        # triangles when a corner lies on a subdivided edge's supporting line.
        fan = None
        for corner in corners:
            pivot = polygon.index(corner)
            ring = polygon[pivot:] + polygon[:pivot]
            candidate = [(ring[0], ring[i], ring[i + 1]) for i in range(1, len(ring) - 1)]
            if all(cross_yz(*(nodes[n] for n in tri)) > 1e-12 for tri in candidate):
                fan = candidate
                break
        require(fan is not None, f'quad {eid}: no valid scoped convex fan')
        # Source winding is +X. Reverse each triangle for the -X impact front.
        triangles.extend(dict(source_quad_id=eid, source_node_ids=[a, c, b])
                         for a, b, c in fan)
    require(stitched == [dict(source_quad_id=1046, source_edge=[1006, 1060],
                              inserted_source_node_ids=list(range(1012, 1060, 6)))],
            'hanging-edge topology differs from the scoped Yaris wall')
    require(len(triangles) == 100, 'expected 100 conforming collision triangles')
    return triangles, stitched


def validate_mesh(vertices, triangles):
    """Checks geometric area, edge incidence/orientation and one disk topology."""
    require(vertices and triangles, 'empty wall mesh')
    require(all(all(math.isfinite(v) for v in p) and abs(p[0] - 0.05) <= 1e-12
                for p in vertices.values()), 'compiled wall must be finite and planar at X=0.05 m')
    scaled = {n: tuple(v / MM_TO_M for v in p) for n, p in vertices.items()}
    edges = defaultdict(list)
    seen, used = set(), set()
    areas = []
    for index, record in enumerate(triangles):
        ids = record['source_node_ids']
        key = tuple(sorted(ids))
        require(len(set(ids)) == 3 and key not in seen, 'degenerate/duplicate triangle')
        seen.add(key)
        used.update(ids)
        signed_double_area = cross_yz(*(vertices[n] for n in ids))
        require(signed_double_area < -1e-18, 'triangle winding must face -X with positive area')
        areas.append(-0.5 * signed_double_area)
        for a, b in zip(ids, ids[1:] + ids[:1]):
            edges[tuple(sorted((a, b)))].append((a, b, index))
    require(used == set(vertices), 'mesh must use all source vertices')
    neighbors = defaultdict(set)
    boundary = []
    for key, uses in edges.items():
        require(len(uses) in (1, 2), 'nonmanifold mesh edge')
        if len(uses) == 2:
            require(uses[0][:2] == uses[1][:2][::-1], 'shared edge winding mismatch')
            neighbors[uses[0][2]].add(uses[1][2])
            neighbors[uses[1][2]].add(uses[0][2])
        else:
            boundary.append(list(key))
        # A conforming triangle edge cannot bypass another source vertex.
        require(not edge_points(key[0], key[1], scaled), 'unresolved hanging mesh vertex')
    visited, pending = set(), [0]
    while pending:
        item = pending.pop()
        if item not in visited:
            visited.add(item)
            pending.extend(neighbors[item] - visited)
    require(len(visited) == len(triangles), 'mesh has disconnected triangles')
    degrees = Counter(n for edge in boundary for n in edge)
    require(all(d == 2 for d in degrees.values()), 'boundary is not a closed loop')
    require(len(vertices) - len(edges) + len(triangles) == 1, 'mesh must be one disk without holes')
    return dict(area_m2=math.fsum(areas), unique_edges=len(edges),
                interior_edges=len(edges) - len(boundary), boundary_edges=sorted(boundary),
                euler_characteristic=1)


def compile_wall(wall_bytes, combine_bytes, verify_pins=True):
    if verify_pins:
        require(sha256(wall_bytes) == WALL_SHA256, 'wall.key SHA256 differs from pinned original')
        require(sha256(combine_bytes) == COMBINE_SHA256, 'combine.key SHA256 differs from pinned original')
    parsed = parse_wall(wall_bytes.decode('ascii'))
    transform = parse_transform(combine_bytes.decode('ascii'))
    vertices = {nid: tuple((p[i] + transform['translation_mm'][i]) * MM_TO_M
                           for i in range(3)) for nid, p in parsed['nodes'].items()}
    triangles, stitched = triangulate(parsed)
    checks = validate_mesh(vertices, triangles)
    source_area = math.fsum(cross_yz(parsed['nodes'][q[0]], parsed['nodes'][q[1]], parsed['nodes'][q[2]]) / 2 +
                            cross_yz(parsed['nodes'][q[0]], parsed['nodes'][q[2]], parsed['nodes'][q[3]]) / 2
                            for q in parsed['quads'].values()) * MM_TO_M ** 2
    require(math.isclose(checks['area_m2'], source_area, rel_tol=1e-12, abs_tol=1e-14),
            'triangulation changed the source quad area')
    node_order = sorted(vertices)
    indices = {nid: i for i, nid in enumerate(node_order)}
    for i, tri in enumerate(triangles):
        tri.update(triangle_id=i + 1, vertex_indices=[indices[n] for n in tri['source_node_ids']],
                   assembled_source_quad_id=tri['source_quad_id'] + transform['element_id_offset'])
    obj_lines = ['# Yaris V1l rigid wall; metres; impact-facing normal -X',
                 '# Source local IDs and assembled IDs are mapped in manifest.json',
                 'o yaris_fixed_wall']
    for nid in node_order:
        obj_lines.append(f'# source_node {nid}')
        obj_lines.append('v ' + ' '.join(format(v, '.17g') for v in vertices[nid]))
    for tri in triangles:
        obj_lines.append(f"g source_quad_{tri['source_quad_id']}")
        obj_lines.append('f ' + ' '.join(str(i + 1) for i in tri['vertex_indices']))
    obj_bytes = ('\n'.join(obj_lines) + '\n').encode('ascii')
    bounds = [[min(p[i] for p in vertices.values()) for i in range(3)],
              [max(p[i] for p in vertices.values()) for i in range(3)]]
    analytic = []
    for region in parsed['analytic_regions']:
        analytic.append(dict(source_wall_id=region['source_wall_id'],
                             assembled_wall_id=region['source_wall_id'] + transform['rigid_wall_id_offset'],
                             source_analytic_bounds_m=[[(p[i] + transform['translation_mm'][i]) * MM_TO_M
                                                       for i in range(3)] for p in region['bounds_mm']]))
    manifest = dict(schema='tlfea.yaris_fixed_wall.v1', model='2010 Toyota Yaris coarse V1l',
                    scope='Fixed wall geometry only; no vehicle or material dynamics imported',
                    source=dict(wall_file='source/wall.key', wall_sha256=sha256(wall_bytes),
                                combine_file='source/combine.key', combine_sha256=sha256(combine_bytes),
                                model_archive_reference_sha256=ARCHIVE_SHA256),
                    generator=dict(file='robo-dyna/tools/import_yaris_wall.py', sha256=sha256(Path(__file__).read_bytes())),
                    transform=transform, source_length_unit='mm', output_length_unit='m', length_scale=MM_TO_M,
                    counts=dict(source_nodes=62, source_quads=46, collision_vertices=62, collision_triangles=100),
                    vertices=[dict(vertex_index=indices[n], source_node_id=n,
                                   assembled_source_node_id=n + transform['node_id_offset'], position_m=list(vertices[n]))
                              for n in node_order],
                    source_quads=[dict(source_quad_id=e, source_part_id=1001,
                                       assembled_source_quad_id=e + transform['element_id_offset'],
                                       assembled_source_part_id=1001 + transform['part_id_offset'], source_node_ids=list(q))
                                  for e, q in sorted(parsed['quads'].items())],
                    triangles=triangles, stitching=dict(tolerance_m=STITCH_TOLERANCE_MM * MM_TO_M, edges=stitched),
                    bounds_m=bounds, validation=checks,
                    contact=dict(representation='fixed_triangle_mesh', front_normal=[-1, 0, 0],
                                 friction=0.6, additional_wall_offset_m=0, source_display_thickness_m=0.001,
                                 dynamic_wall_dofs=0, velocity_m_per_s=[0, 0, 0],
                                 analytic_force_generation=False, analytic_source_regions=analytic,
                                 boundary_policy='Mesh is authoritative; finite analytical bounds are reporting only',
                                 half_thickness_policy='Vehicle half-thickness is applied separately exactly once'),
                    reaction_groups=dict(whole_wall_triangle_ids=list(range(1, 101)),
                                         source_segment_set_1001_triangle_ids=[t['triangle_id'] for t in triangles
                                                                              if t['source_quad_id'] != 1046]),
                    artifacts={'wall.obj': dict(sha256=sha256(obj_bytes), bytes=len(obj_bytes))})
    return obj_bytes, manifest


def write_artifacts(output, obj_bytes, manifest, wall_bytes, combine_bytes):
    manifest_bytes = (json.dumps(manifest, indent=2, sort_keys=True, allow_nan=False) + '\n').encode()
    files = {'wall.obj': obj_bytes, 'manifest.json': manifest_bytes,
             'source/wall.key': wall_bytes, 'source/combine.key': combine_bytes}
    files['SHA256SUMS'] = ''.join(f'{sha256(data)}  {name}\n' for name, data in sorted(files.items())).encode()
    for name, data in files.items():
        path = output / name
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_name(path.name + '.tmp')
        temporary.write_bytes(data)
        temporary.replace(path)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-model', type=Path, required=True,
                        help='Existing extracted directory containing original wall.key and combine.key')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    try:
        wall_bytes = (args.source_model / 'wall.key').read_bytes()
        combine_bytes = (args.source_model / 'combine.key').read_bytes()
        obj_bytes, manifest = compile_wall(wall_bytes, combine_bytes)
        write_artifacts(args.output, obj_bytes, manifest, wall_bytes, combine_bytes)
    except (OSError, ValueError) as error:
        parser.exit(1, f'wall import failed: {error}\n')
    print(json.dumps(dict(output=str(args.output), counts=manifest['counts'],
                          area_m2=manifest['validation']['area_m2'],
                          mesh_sha256=manifest['artifacts']['wall.obj']['sha256']), sort_keys=True))


if __name__ == '__main__':
    main()
