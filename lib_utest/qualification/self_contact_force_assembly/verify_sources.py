#!/usr/bin/env python3
"""Prove the isolated accepted-state force/STI source boundary."""

from pathlib import Path
import json

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
COLLISION = ROOT / "lib_src/collision"

public = (COLLISION / "SelfContactForceAssembly.h").read_text()
types = (COLLISION / "SelfContactForceTypes.h").read_text()
values = (COLLISION / "SelfContactForceValues.h").read_text()
operations = (COLLISION / "self_contact_force/Operations.cu").read_text()
source = (COLLISION / "self_contact_force/Source.cpp").read_text()
layout = (COLLISION / "self_contact_force/Layout.cpp").read_text()
initialize = (COLLISION / "self_contact_force/Initialize.cpp").read_text()
cmake = (COLLISION / "SelfContactForceAssembly.cmake").read_text()
bazel = (COLLISION / "BUILD.bazel").read_text()
nodal_header = (ROOT / "lib_src/solvers/FENodalState.h").read_text()
nodal_state = (ROOT / "lib_src/solvers/FENodalState.cu").read_text()
nodal_identity = (ROOT / "lib_src/solvers/NodalTrialIdentity.h").read_text()
physical_owner = (
    ROOT / "lib_src/elements/ShellPhysicalOwnerAssembly.cpp").read_text()

assert "Forecast(" in public and "Initialize(" in public
assert "AssembleAccepted(" in public and "DiscardTrial()" in public
assert "stiffness_per_area_n_m3" in types
assert "vertex_use = UINT32_MAX" in types
assert "facet_use = UINT32_MAX" in types
assert "Diagnostic completeness only" in types
for forbidden in ("effective_mass", "line_area", "damping", "friction",
                  "approximation_radius"):
    assert forbidden not in types
assert "RepresentedSelfContactStiffness" in values
assert "mass_detail::UpperProduct" in values
assert "BuildSelfContactForceIncidence" in values
assert "EvaluateSurfacePenaltyPair" in operations
assert "view.accepted.position_xyz" in operations
assert "view.accepted.velocity_xyz" in operations
assert "view.translation_fixed_bits[node]" in operations
assert "BorrowAssembly(" in operations
assert "AuthenticateAssemblyView(token, view)" in operations
assert "AssemblyRangeDisjoint(" in operations
assert operations.index("StageNodes<<<") < operations.index("CheckNodes<<<")
assert operations.index("CheckNodes<<<") < operations.index("PublishNodes<<<")
assert "force->force_n" in operations
assert "majorant->diagonal_n_m" in operations
assert "couple_x" in operations and "couple_z" in operations
assert "rotational_stiffness[node]" not in operations
assert "atomicAdd" not in operations
assert "cudaMalloc" not in operations
assert "cudaMalloc" in initialize
assert "owner->Discard()" in initialize
assert "assembler_identity" in initialize
assert "Authenticates(" in initialize
assert "Duplicate canonical self-contact feature" in (
    COLLISION / "self_contact_force/Values.cpp").read_text()
assert "AdmittedVertexFace" in source
assert "ClassifyVertexFace(" in source
assert "SameClassification(" in source
assert "UnsupportedCinSecondary" in source
assert "PositiveSelfContactArea" in source
assert "exact discovered target stratum" in source
assert "MakeLayout(config.event_capacity" in layout
assert "host_arena_bytes = layout.bytes" in layout
assert "device_bytes = layout.bytes" in layout
assert "startup_scratch_bytes = proof.bytes" in layout
assert "retained - sizeof(SelfContactActiveUseBinding)" in layout
assert "AuthenticateInitial(" in initialize
assert "AuthenticateAssemblyView(" in nodal_header
assert "AssemblyRangeDisjoint(" in nodal_header
assert "SameAssembly(" in nodal_identity
assert "ActiveAssemblyView()" in nodal_state
assert "owner.AuthenticateAssemblyView(token,view)" in physical_owner
assert "tl_self_contact_force_assembly" in cmake
assert 'name = "self_contact_force_assembly"' in bazel
assert "nodal_assembly_authentication_source_proof" in bazel
qualification_cmake = (HERE / "CMakeLists.txt").read_text()
assert "SELF_CONTACT_FORCE_CUDA" in qualification_cmake
assert "rigid-cin-response" in qualification_cmake
cuda = (HERE / "CudaTest.cu").read_text()
for gate in (
    "EventPermutationPreservesCanonicalAssemblyAndAllocationExactly",
    "ActualCinMasterGetsDenseForceMomentAndStiWhileSecondaryGetsNone",
    "ActualOwnerPartialAndFullyFixedMasksKeepFullReactionChannels",
    "SurfaceCinSecondary",
    "CompleteNodalValidation",
):
    assert gate in cuda
qualification_bazel = (HERE / "BUILD.bazel").read_text()
assert '":root_cuda_sources"' in qualification_bazel
assert "m2-rigid-cin-response" in qualification_bazel

print(json.dumps({
    "status": "passed",
    "scope": "accepted-state CUDA force/STI scratch",
    "floating_atomics": False,
    "attempt_allocations": False,
    "candidate_geometry": False,
    "history": False,
}))
