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

assert "Forecast(" in public and "Initialize(" in public
assert "AssembleAccepted(" in public and "DiscardTrial()" in public
assert "stiffness_per_area_n_m3" in types
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
assert operations.index("StageNodes<<<") < operations.index("CheckNodes<<<")
assert operations.index("CheckNodes<<<") < operations.index("PublishNodes<<<")
assert "force->force_n" in operations
assert "majorant->diagonal_n_m" in operations
assert "couple_x" in operations and "couple_z" in operations
assert "rotational_stiffness[node]" not in operations
assert "atomicAdd" not in operations
assert "cudaMalloc" not in operations
assert "cudaMalloc" in initialize
assert "Duplicate canonical self-contact feature" in (
    COLLISION / "self_contact_force/Values.cpp").read_text()
assert "AdmittedVertexFace" in source
assert "UnsupportedCinSecondary" in source
assert "PositiveSelfContactArea" in source
assert "activity_current_identity" in source
assert "MakeLayout(config.event_capacity" in layout
assert "host_arena_bytes = layout.bytes" in layout
assert "device_bytes = layout.bytes" in layout
assert "startup_scratch_bytes = proof.bytes" in layout
assert "AuthenticateInitial(" in initialize
assert "tl_self_contact_force_assembly" in cmake
assert 'name = "self_contact_force_assembly"' in bazel
assert "SELF_CONTACT_FORCE_CUDA" in (
    HERE / "CMakeLists.txt").read_text()

print(json.dumps({
    "status": "passed",
    "scope": "accepted-state CUDA force/STI scratch",
    "floating_atomics": False,
    "attempt_allocations": False,
    "candidate_geometry": False,
    "history": False,
}))
