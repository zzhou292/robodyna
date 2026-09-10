#!/usr/bin/env python3
"""Owning MB1 input map; source reads only, never builds or runs mechanics.

Default lists the declared local closure without hashing. --write creates the
pre-execution map once; --check compares it with current inputs. Native source
and preparation verification remains with the two pinned owning verifiers.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re

HERE = Path(__file__).resolve().parent
TL = HERE.parents[3]
WORKSPACE = TL.parent
QUAL = 'lib_utest/qualification/'
# Separate translation units/build files are explicit; local C/C++ includes
# below are recursive. No directory-wide free-response/contact dependency.
PATTERNS = [
    'BUILD', 'MODULE.bazel', 'MODULE.bazel.lock', '.bazelrc',
    'lib_src/elements/BUILD.bazel', 'lib_src/elements/ShellBatch*',
    'lib_src/elements/qeph/BUILD.bazel', 'lib_src/elements/qeph/CMakeLists.txt',
    'lib_src/elements/qeph/QephBatch*',
    'lib_src/elements/t3/BUILD.bazel', 'lib_src/elements/t3/CMakeLists.txt',
    'lib_src/elements/t3/T3Batch*',
    'lib_src/solvers/BUILD.bazel', 'lib_src/solvers/CMakeLists.txt',
    'lib_src/solvers/FENodalState*', 'lib_src/solvers/ExplicitNodalStep*',
    'lib_src/solvers/ExplicitTranslationStep*',
    'lib_src/materials/BUILD.bazel', 'lib_src/math/BUILD.bazel',
    'lib_src/collision/BUILD.bazel', 'lib_utest/BUILD.bazel',
    'lib_utest/utest_nodal_temporal_cuda.cu',
    QUAL+'t3/CMakeLists.txt', QUAL+'t3/T3Force.cmake',
    QUAL+'t3/source-manifest.json', QUAL+'t3/force-source-manifest.json',
    QUAL+'t3/verify_sources.py', QUAL+'t3/verify_force_sources.py',
    QUAL+'t3/mixed/CMakeLists.txt', QUAL+'t3/mixed/README.md',
    QUAL+'t3/mixed/prepare_source_map.py', QUAL+'t3/mixed/MixedShell*',
    QUAL+'t3/mixed_binding/CMakeLists.txt', QUAL+'t3/mixed_binding/source-map.json',
    QUAL+'t3/batch/CMakeLists.txt', QUAL+'t3/batch/T3BatchFailureTest.cu',
    QUAL+'qeph/batch/CMakeLists.txt', QUAL+'qeph/batch/QephBatchFailureTest.cu',
    QUAL+'qeph/source-manifest.json', QUAL+'qeph/verify_sources.py',
]
# The exact original/extracted Fortran closure is delegated to manifests;
# authored wrappers, native C++ bridge translation units and build context are
# still direct MB1 inputs. Native regression test bodies are not linked here.
for family, names in {
    'qeph': ['QephReference.cpp', 'QephGeometry.cpp', 'QephHistory.cpp', 'QephForceReference.cpp'],
    't3': ['T3Reference.cpp', 'T3Geometry.cpp', 'T3Kinematics.cpp', 'T3History.cpp', 'T3ForceReference.cpp'],
}.items():
    prefix = QUAL+'native/'+family+'/'
    PATTERNS += [prefix+n for n in names]
    PATTERNS += [prefix+p for p in ['*.F', 'CMakeLists.txt',
                                  'source-manifest.json', 'prepare_sources.py', 'verify_sources.py']]
PATTERNS += [QUAL+'native/t3/T3Engine.cmake', QUAL+'native/t3/T3Force.cmake']
PREREQUISITES = ['crash-work/checkpoints/'+name+'/manifest.json' for name in [
    'qeph-shared-helpers-1', 't3-resident-batch-1', 'shell-mixed-binding-1', 'qeph-wall-cw0-1']]
ABI = 'crash-work/checkpoints/robo-dyna-restart-20260910T0010Z/abi/measurements.json'
EVIDENCE = PREREQUISITES+[ABI, 'crash-work/reports/mixed-shell-abi-build-1.json',
                        'planning/T3_RESIDENT_BATCH_DESIGN.md']
INCLUDE = re.compile(r'^\s*#\s*include\s*([<"])([^>"\n]+)[>"]', re.MULTILINE)
CODE = {'.cpp', '.cu', '.h', '.cuh', '.hpp', '.cc'}


def closure():
    paths = set()
    for pattern in PATTERNS:
        found = [p.resolve() for p in TL.glob(pattern) if p.is_file()]
        if not found:
            raise RuntimeError('Missing declared input: '+pattern)
        paths.update(found)
    paths.update(WORKSPACE/p for p in EVIDENCE)
    pending = list(paths)
    external = set()
    while pending:
        path = pending.pop()
        if path.suffix not in CODE:
            continue
        for delimiter, name in INCLUDE.findall(path.read_text()):
            local = next((p.resolve() for p in [path.parent/name, TL/name] if p.is_file()), None)
            if local is None:
                if delimiter == '"':
                    raise RuntimeError('Unresolved quoted local include: '+str(path)+': '+name)
                external.add(name)
            elif local not in paths:
                local.relative_to(TL)  # A local compiler include cannot escape TL.
                paths.add(local)
                pending.append(local)
    return sorted(p.relative_to(WORKSPACE).as_posix() for p in paths), sorted(external)


def record(path):
    data = (WORKSPACE/path).read_bytes()
    return {'path': path, 'bytes': len(data), 'sha256': hashlib.sha256(data).hexdigest()}


def source_map(paths, external):
    abi = json.loads((WORKSPACE/ABI).read_text())
    expected = {'qeph_storage': 14832, 't3_storage': 6224, 'publication_storage': 304,
                'qeph_alignment': 8, 't3_alignment': 8, 'publication_alignment': 8}
    if any(abi.get(k) != v for k, v in expected.items()):
        raise RuntimeError('Retained host ABI differs from reviewed forecast')
    return {
        'schema': 'tl.shell-mixed-publication-source-map.v1',
        'stage': 'source-and-independent-review-frozen-before-first-execution',
        'scope': 'MB1 prescribed-only Q4/T3 joint publication; declared linked/build inputs and recursive local C/C++ includes, with delegated pinned native closure; not a hermetic compiler dependency closure',
        'cuda_execution_qualified': False, 'mixed_force_feedback_qualified': False,
        'contact_qualified': False, 'long_response_stability_qualified': False,
        'qualification_id': '0x4d42315052455331', 'configuration_id': '0x4d42314d4f444c31',
        'test_functions': 8, 'native_cell_interval_checks': 20, 'additional_authenticity_test_functions': 3,
        'host_abi_forecast': expected, 'device_allocation_measurement_pending': True,
        'explicit_allocation_count': 9, 'prerequisites': [record(p) for p in PREREQUISITES],
        'delegated_native_closures': [record('Total-Lagrangian-FEA/'+QUAL+'native/'+f+'/source-manifest.json') for f in ['qeph', 't3']],
        'external_dependency_scope': 'Toolchain, CUDA runtime/headers, Fortran runtime, GTest and system headers are supplied by the bounded build environment; prepared native receipts are verified separately by owning scripts.',
        'external_includes': external, 'input_count': len(paths), 'inputs': [record(p) for p in paths],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--write', action='store_true', help='create source-map.json; existing output fails')
    mode.add_argument('--check', action='store_true', help='verify exact current map and input closure')
    args = parser.parse_args()
    paths, external = closure()
    if not (args.write or args.check):
        print(json.dumps({'input_count': len(paths), 'paths': paths, 'external_includes': external}, indent=2))
        return
    encoded = (json.dumps(source_map(paths, external), indent=2)+'\n').encode()
    destination = HERE/'source-map.json'
    if args.write:
        with destination.open('xb') as output:
            output.write(encoded)
    elif destination.read_bytes() != encoded:
        raise RuntimeError('Mixed source map or declared input closure changed')
    print(json.dumps({'status': 'passed', 'input_count': len(paths), 'check_only': args.check}))


if __name__ == '__main__':
    main()
