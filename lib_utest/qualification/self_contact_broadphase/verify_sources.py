#!/usr/bin/env python3
"""Bind complete SAP donor bytes and the small shared traversal extraction."""
from pathlib import Path
import hashlib
import json
import re
import subprocess
import sys

here = Path(__file__).resolve().parent
root = here.parents[2]
manifest_bytes = (here / 'source-manifest.json').read_bytes()
assert hashlib.sha256(manifest_bytes).hexdigest() == '25a38aac3a87ff59f91a76890d48611be71e5f55342d96693e5edb3babd698c6'
manifest = json.loads(manifest_bytes)
for row in manifest['files']:
    path = Path(row['path'])
    assert not path.is_absolute() and '..' not in path.parts
    raw = (root / path).read_bytes()
    assert len(raw) == row['bytes'] and hashlib.sha256(raw).hexdigest() == row['sha256'], str(path)

def without_rule(text, rule, name):
    marker = rule + '(\n    name = "' + name + '",'
    begin = text.index(marker)
    depth = 0
    end = text.index('(', begin)
    while end < len(text):
        depth += (text[end] == '(') - (text[end] == ')')
        end += 1
        if depth == 0:
            break
    assert end <= len(text) and text[end:end + 2] == '\n\n', name
    return text[:begin] + text[end + 2:]

composition = manifest['integration_review']['discovery_crossing_build_composition']
build = (root / composition['current_record']['path']).read_text()
restored = without_rule(
    build, 'filegroup', 'fixed_triangle_feature_discovery_sources')
restored = without_rule(
    restored, 'cc_library', 'fixed_triangle_feature_discovery')
restored = without_rule(
    restored, 'filegroup', 'represented_interval_crossing_source_proof')
restored = without_rule(
    restored, 'cc_library', 'represented_interval_crossing')
old = restored.encode()
previous = composition['previous_record']
assert len(old) == previous['bytes']
assert hashlib.sha256(old).hexdigest() == previous['sha256']

def body(text, name):
    begin = text.index('{', text.index(name + '('))
    end, depth = begin + 1, 1
    while depth:
        depth += (text[end] == '{') - (text[end] == '}')
        end += 1
    return text[begin + 1:end - 1]

def compact(text):
    text = re.sub(r'/\*.*?\*/|//[^\n]*', '', text, flags=re.S)
    return re.sub(r'\s+', '', text)

def same(a, b, label):
    assert compact(a) == compact(b), label

reference = here / 'reference'
collision = root / 'lib_src/collision'
assert (reference / 'HydroelasticBroadphase.cu').read_bytes() == (collision / 'HydroelasticBroadphase.cu').read_bytes()
old = (reference / 'HydroelasticBroadphase.cuh').read_text()
new = (collision / 'HydroelasticBroadphase.cuh').read_text()
aabb = 'struct AABB {\n  double3 min;\n  double3 max;\n  int objectId;\n};\n'
assert aabb in old
expected = old.replace(aabb, '').replace('"HydroelasticCollisionTypes.cuh"', '"HydroelasticBroadphaseTypes.cuh"')
same(new, expected, 'legacy public API/defaults and exact AABB extraction')
assert compact(aabb) in compact((collision / 'HydroelasticBroadphaseTypes.cuh').read_text())
old = (reference / 'HydroelasticBroadphaseFunc.cuh').read_text()
new = (collision / 'HydroelasticBroadphaseFunc.cuh').read_text()
sweep = (collision / 'broadphase/Sweep.h').read_text()
for name in ('computeAABBKernel', 'countSameMeshPairsKernel', 'extractSortKeysKernel', 'reorderAABBsKernel'):
    same(body(old, name), body(new, name), 'unchanged ' + name)
neighbor = body(old, 'isNeighborPair')
for before, after in [('neighborHashes', 'hashes'), ('numHashes', 'count'), ('temp', 'temporary'), ('mid', 'middle')]:
    neighbor = re.sub(r'\b' + before + r'\b', after, neighbor)
neighbor = neighbor.replace('long long hash = ((long long)idA << 32)',
                            'const long long hash = (static_cast<long long>(idA) << 32)')
neighbor = neighbor.replace('int middle =', 'const int middle =')
same(neighbor, body(sweep, 'IsNeighborPair'), 'complete neighbor binary search/order')
same(body(old, 'broadphaseAxisValue'), body(sweep, 'AxisValue'), 'axis helper')
same(body(new, 'isNeighborPair'), 'return broadphase_detail::IsNeighborPair(a,b,hashes,count);', 'neighbor compatibility wrapper')
same(body(new, 'broadphaseAxisValue'), 'return broadphase_detail::AxisValue(p,axis);', 'axis compatibility wrapper')
# The shared traversal is the original stop/overlap prefix with only names,
# const and redundant expression parentheses changed. The original policy and
# write operations are checked separately below.
value = body(old, 'countCollisionsKernel')
geometry = value[value.index('  const AABB& Ai'):value.index('    if (overlapX && overlapY && overlapZ)')]
geometry = geometry.replace('  unsigned long long count = 0;', '')
for before, after in [('sortedAABBs', 'sorted'), ('Ai', 'a'), ('Aj', 'b'),
                      ('broadphaseAxisValue', 'AxisValue'), ('overlapX', 'x'), ('overlapY', 'y'), ('overlapZ', 'z')]:
    geometry = re.sub(r'\b' + before + r'\b', after, geometry)
geometry = re.sub(r'bool ([xyz]) = \(([^;]+)\);', r'const bool \1 = \2;', geometry)
geometry += 'if (x && y && z && keep(a.objectId, b.objectId)) consumer(a.objectId, b.objectId);}'
same(geometry, body(sweep, 'VisitLater'), 'exact sweep stop/axis overlap/inclusive comparisons')
legacy = 'if (!enable_self_collision && mesh_ids != nullptr && mesh_ids[a] == mesh_ids[b]) return false; return !IsNeighborPair(a,b,neighbors,neighbor_count);'
same(body(sweep[sweep.index('struct LegacyFilter'):], 'operator()'), legacy, 'same mesh then neighbor policy')
assert 'if (!enableSelfCollision && elementMeshIds != nullptr)' in value
assert 'int meshIdA = elementMeshIds[Ai.objectId];' in value
assert 'int meshIdB = elementMeshIds[Aj.objectId];' in value
assert 'if (meshIdA == meshIdB)' in value
assert 'if (!isNeighborPair(Ai.objectId, Aj.objectId, neighborHashes,' in value
prefix = 'unsigned int i = blockIdx.x * blockDim.x + threadIdx.x; if (i >= n) return;'
filter_args = 'broadphase_detail::LegacyFilter{neighborHashes,numHashes,elementMeshIds,enableSelfCollision}'
same(body(new, 'countCollisionsKernel'), prefix + 'broadphase_detail::CountPairs count; broadphase_detail::VisitLater(sortedAABBs,n,i,axis,' + filter_args + ',count); collisionCounts[i] = count.count;', 'legacy count wrapper')
same(body(new, 'generateCollisionPairsKernel'), prefix + 'broadphase_detail::WriteLegacyPairs output{collisionPairs,collisionOffsets[i]}; broadphase_detail::VisitLater(sortedAABBs,n,i,axis,' + filter_args + ',output);', 'legacy fill wrapper')
assert 'collisionPairs[writeIdx++] = CollisionPair(Ai.objectId, Aj.objectId);' in body(old, 'generateCollisionPairsKernel')
same(body(sweep[sweep.index('struct WriteLegacyPairs'):], 'operator()'), 'pairs[offset++] = CollisionPair(a,b);', 'exact old pair field order')
implementation = (collision / 'SelfContactBroadphase.cpp').read_text()
evaluate = body(implementation, 'SelfContactBroadphase::Evaluate')
assert evaluate.index('s.complete = false') < evaluate.index('ExplicitStream(stream)')
assert evaluate.index('s.host_control.invalid_parent != UINT32_MAX') < evaluate.index('bp::SortBoxes')
assert evaluate.index('count > f.pair_capacity') < evaluate.index('bp::Fill') < evaluate.index('static_cast<int>(count)')
assert evaluate.rindex('cudaStreamSynchronize') < evaluate.index('s.complete = true')
assert 'cudaMalloc' not in evaluate and 'cudaFree' not in evaluate
assert 'stream != cudaStreamLegacy && stream != cudaStreamPerThread' in implementation
subprocess.run([sys.executable, '-B', str(here.parent / 'mapped_wall_assembly_inputs/verify_sources.py')], check=True)
print(json.dumps({'status': 'passed', 'records': len(manifest['files']),
                  'legacy_owner': 'complete unchanged .cu', 'numerical_execution': False}))
