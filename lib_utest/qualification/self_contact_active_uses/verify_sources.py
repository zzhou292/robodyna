#!/usr/bin/env python3
"""Prove the isolated active-use policy boundary without executing mechanics."""
from pathlib import Path
import json

here = Path(__file__).resolve().parent
root = here.parents[2]
collision = root / "lib_src/collision"
types = (collision / "SelfContactActiveUseTypes.h").read_text()
values = (collision / "SelfContactActiveUseValues.h").read_text()
binding = (collision / "SelfContactActiveUseBinding.cpp").read_text()
build = (collision / "self_contact_active_use/Build.cpp").read_text()
queries = (collision / "self_contact_active_use/Queries.cpp").read_text()
layout = (collision / "self_contact_active_use/Layout.cpp").read_text()
public = (collision / "SelfContactActiveUseBinding.h").read_text()
facet_public = (collision / "FixedContactFacetBinding.h").read_text()
facet_binding = (collision / "FixedContactFacetBinding.cpp").read_text()

assert 'SymmetricDirectedVertexDualReferenceV1' in types
assert 'SymmetricDirectedVertexAndEdgePointDualReferenceV2' in types
assert 'Q4CenterAreaContactModel' in types
q4 = build[build.index('if (source.arity == 4)'):build.index('SurfaceTriangle parent;')]
assert 'PrepareQ4MaterialMeasure' in q4
assert 'EvaluateQ4MaterialDensity(measure, 0, 0' in q4
assert 'q4_bounds::Scale({density.lower, density.upper}, 4' in q4
assert 'area_enclosure' not in q4
t3 = build[build.index('SurfaceTriangle parent;'):build.index('bool Dual(')]
assert 'PrepareT3MaterialMeasure' in t3 and 'measure.area_enclosure()' in t3
assert 'q4_bounds::DividePositive' in build and 'q4_bounds::Certify' in build
assert 'use.facet_valence' in build and 'use.directed_vf_area_m2' in build
assert 'use.directed_endpoint_dual_area_m2[endpoint]' in build
assert 'endpoint_use.directed_vf_area_m2' in build
assert 'FixedContactFacetReadCursor facet_reader' in build
assert 'facets.Describe' not in build
assert 'facet_reader.Initialize(facets)' in build
assert 'descriptor.facet' in build
assert 'std::sort(out.vertex_uses' not in build
assert 'std::sort(scratch.vertex_order' in build
assert 'startup_index_bytes' in layout
cursor_public = facet_public[
    facet_public.index('class FixedContactFacetReadCursor {'):]
for deleted in (
        'FixedContactFacetReadCursor(const FixedContactFacetReadCursor&) = delete',
        'FixedContactFacetReadCursor(FixedContactFacetReadCursor&&) = delete',
        'const FixedContactFacetReadCursor&) = delete',
        'FixedContactFacetReadCursor&&) = delete'):
    assert deleted in cursor_public
assert 'std::optional<FixedContactFacetBinding> binding_' in cursor_public
assert 'FixedContactFacet current_' in cursor_public
assert 'FixedContactFacet*' not in cursor_public[
    cursor_public.index('FixedContactFacetReport Initialize('):
    cursor_public.index(' private:')]
cursor_binding = facet_binding[
    facet_binding.index('FixedContactFacetReadCursor::Initialize('):
    facet_binding.index('Status FixedContactFacetBinding::Approximation(')]
assert cursor_binding.count('binding.OutputDisjoint(this, sizeof(*this))') == 1
assert 'binding_.emplace(binding)' in cursor_binding
assert 'current_ = {}' in cursor_binding
assert 'report.status == FixedContactFacetStatus::Ok ? &current_ : nullptr' in cursor_binding
assert 'CountSelfContactActiveUses' in values
assert 'numeric_limits<std::size_t>::max()' in values
assert 'activity.current[i] > activity.base[i]' in build
assert 'complete_rigid_group' in queries and 'ExcludedSameRigidGroup' in queries
assert 'source_part_id' not in queries
assert 'UnsupportedCinSecondary' in queries
assert 'CompleteLocalSupportNeedsRuntimeActivity' in queries
assert 'UnadmittedEdgeEdgeForceArea' in queries
assert 'AdmittedEdgeEdge' in queries
assert 'EdgeParameter' in queries
assert 'InterpolateArea' in queries
assert 'q4_bounds::Add' in queries
assert 'SameParentNeedsCurrentRegularity' in queries
assert 'next.parent[0] == next.parent[1]' in queries
assert queries.count('next.activity_base_identity = activity.base') == 2
assert queries.count('next.activity_current_identity = activity.current') == 2
assert queries.count('next.activity_parent_count = activity.parent_count') == 2
assert 'CoveredByIndependentAdmittedVertexFace' not in types
assert 'independent_vf' not in queries
assert 'surface.physical()->execution()' in layout
assert "Supplied rigid source is not S0 execution's actual binding" in layout
for text in (binding, build, queries):
    for forbidden in ('cudaMalloc', 'cudaFree', 'ApplyPenalty',
                      'AssembleAccepted', 'AdvanceStaggeredCin'):
        assert forbidden not in text, forbidden
for text in (build, queries):
    assert 'std::vector' not in text
assert 'ClassifyVertexFace' in public and 'ClassifyEdgeEdge' in public
assert 'SelfContactActivityView' in public
assert 'Authenticates(const SelfContactPairClassification&)' in public
assert 'pair.binding_identity == impl_.get()' in binding
cmake = (collision / "SelfContactActiveUseBinding.cmake").read_text()
bazel = (collision / "BUILD.bazel").read_text()
assert 'tl_self_contact_active_uses' in cmake
assert 'name = "self_contact_active_uses"' in bazel
source_group = bazel[bazel.index(
    'name = "self_contact_active_use_source_proof"'):
    bazel.index('cc_library(', bazel.index(
        'name = "self_contact_active_use_source_proof"'))]
assert '"FixedContactFacetBinding.cpp"' in source_group
assert '"FixedContactFacetBinding.h"' in source_group
qualification_cmake = (here / "CMakeLists.txt").read_text()
qualification_bazel = (here / "BUILD.bazel").read_text()
assert "ScalingTest.cpp" in qualification_cmake
assert "ScalingTest.cpp" in qualification_bazel
assert 'name = "source_check"' in qualification_bazel
assert "self-contact-refinement" in qualification_cmake
assert "m2-self-contact-refinement" in qualification_bazel
assert "symmetric-edge-area" in qualification_cmake
assert "symmetric-edge-area" in qualification_bazel
print(json.dumps({
    "status": "passed",
    "policy": "SymmetricDirectedVertexAndEdgePointDualReferenceV2",
    "q4_measure": "center-area-uniform-natural-v1",
    "allocation_per_query": False,
    "mechanics_execution": False,
    "runtime_tie_activity_available": False,
}))
