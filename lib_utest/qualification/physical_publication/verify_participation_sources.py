#!/usr/bin/env python3
"""Focused source proof for fixed scratch participation publication wiring."""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
HEADER = ROOT / "lib_src/elements/publication/ShellPhysicalScratchParticipation.h"
SOURCE = ROOT / "lib_src/elements/publication/ShellPhysicalScratchParticipation.cpp"
PHYSICAL_STATE = ROOT / "lib_src/elements/publication/PhysicalState.h"
TRANSACTION = ROOT / "lib_src/elements/publication/PhysicalTransaction.cpp"
WALL_HEADER = ROOT / "lib_src/collision/NodalWallMappedContact.h"
WALL_SOURCE = ROOT / "lib_src/collision/nodal_wall_mapped/Operations.cu"
WALL_INITIALIZE = ROOT / "lib_src/collision/nodal_wall_mapped/Initialize.cpp"
ELEMENTS_CMAKE = ROOT / "lib_src/elements/ShellBatchPublication.cmake"
ELEMENTS_BAZEL = ROOT / "lib_src/elements/BUILD.bazel"
COLLISION_BAZEL = ROOT / "lib_src/collision/BUILD.bazel"
CUDA_WIRING = Path(__file__).resolve().parent / "Cuda.cmake"
HOST_TEST = Path(__file__).resolve().parent / "HostTest.cpp"
WALL_CUDA_WIRING = (
    Path(__file__).resolve().parent.parent / "physical_mesh_wall" /
    "CMakeLists.txt")
WALL_CUDA_TEST = (
    Path(__file__).resolve().parent.parent / "physical_mesh_wall" /
    "ParticipationTest.cu")


def require(text: str, token: str, where: Path) -> None:
    if token not in text:
        raise RuntimeError(f"{where}: missing {token!r}")


header = HEADER.read_text()
source = SOURCE.read_text()
transaction = TRANSACTION.read_text()
for token in (
    "MappedWall = 0",
    "SelfContact = 1",
    "class ShellPhysicalScratchParticipationReceipt",
    "friend class ShellBatchPublication",
    "NodalPreparedView prepared_",
    "RecordAcceptedAssembly",
    "SealCandidate",
    "RecordMappedWallAcceptedAssembly",
    "SealMappedWallCandidate",
    "friend class ::tlfea::contact::NodalWallMappedContact",
    "RecordSelfContactAcceptedAssembly",
    "SealSelfContactCandidate",
    "friend class ::tlfea::contact::SelfContactTransaction",
    "ShellPhysicalScratchReceiptRoster",
):
    require(header, token, HEADER)
for forbidden in (
    "std::vector",
    "std::map",
    "std::function",
    "cudaMalloc",
    "cudaMemcpy",
    "cudaStreamSynchronize",
    "cudaGetLastError",
):
    if forbidden in source:
        raise RuntimeError(f"{SOURCE}: forbidden per-attempt/device operation {forbidden!r}")
for token in (
    "ForecastPhysicalScratchParticipation",
    "ConfigurePhysicalScratchParticipation",
    "SealPhysicalScratchParticipation",
    "ValidatePhysicalScratchSeal",
    "ConsumePhysicalScratchSeal",
    "Scratch assembly can be recorded only by its concrete transaction",
    "Scratch candidate can be sealed only by its concrete transaction",
):
    require(source, token, SOURCE)
physical_state = PHYSICAL_STATE.read_text()
for token in (
    "struct PhysicalRuntimeState",
    "std::uintptr_t tagged",
    "SetCinCounts",
    "CinWitnessCount",
    "SetScratchParticipation",
    "std::is_trivially_copyable_v<PhysicalRuntimeState>",
):
    require(physical_state, token, PHYSICAL_STATE)
for forbidden in ("union PhysicalRuntimeState", "runtime.cin", "runtime.scratch"):
    if forbidden in physical_state or forbidden in source:
        raise RuntimeError(
            f"tagged runtime source retains inactive-union access {forbidden!r}")

wall_header = WALL_HEADER.read_text()
wall_source = WALL_SOURCE.read_text()
for token in (
    "class NodalWallMappedTransactionReceipt",
    "ShellPhysicalScratchParticipation participation_",
    "roster_entry()",
):
    require(wall_header, token, WALL_HEADER)
for forbidden in (
    "RecordMappedWallAcceptedAssembly(\n      std::uint64_t",
    "SealMappedWallCandidate(\n      std::uint64_t",
):
    if forbidden in header:
        raise RuntimeError(f"{HEADER}: mapped wall accepts caller source identity")
record = wall_source.index("RecordMappedWallAcceptedAssembly")
assembly_validation = wall_source.index("ReadDiagnostics(false,next)")
if record < assembly_validation:
    raise RuntimeError(f"{WALL_SOURCE}: wall records before complete assembly")
seal = wall_source.index("SealMappedWallCandidate")
candidate_validation = wall_source.index("ReadDiagnostics(true,next)")
if seal < candidate_validation:
    raise RuntimeError(f"{WALL_SOURCE}: wall seals before candidate validation")
for token in (
    "FailConfigured",
    "NodalWallMappedTransactionReceipt* receipt",
):
    require(wall_source, token, WALL_SOURCE)
for token in ("FailConfigured", "participation_.DiscardTrial()"):
    require(WALL_INITIALIZE.read_text(), token, WALL_INITIALIZE)
require(transaction, "ValidatePhysicalScratchSeal(owner,authentic)", TRANSACTION)
require(transaction, "ConsumePhysicalScratchSeal();", TRANSACTION)
commit = transaction.index("owner.Commit(token)")
if "participation" in transaction[commit:].lower():
    raise RuntimeError(
        f"{TRANSACTION}: participation operation appears after owner commit")
for wiring in (ELEMENTS_CMAKE, ELEMENTS_BAZEL):
    require(wiring.read_text(), "ShellPhysicalScratchParticipation.cpp", wiring)
require(CUDA_WIRING.read_text(), "ParticipationTest.cu", CUDA_WIRING)
for token in (
    "TaggedPhysicalRuntimeSupportsZeroWitnessByteCopyAndBothDestructionPaths",
    "std::memcpy",
    "SetScratchParticipation",
):
    require(HOST_TEST.read_text(), token, HOST_TEST)
require(COLLISION_BAZEL.read_text(),
        "nodal_wall_mapped_publication_source_files", COLLISION_BAZEL)
require(WALL_CUDA_WIRING.read_text(), "ParticipationTest.cu", WALL_CUDA_WIRING)
for token in (
    "NodalWallMappedContact",
    "wall.roster_entry()",
    "NodalWallMappedTransactionReceipt",
    "PlacementNewAba",
    "MappedWall",
):
    require(WALL_CUDA_TEST.read_text(), token, WALL_CUDA_TEST)
print("fixed scratch participation source proof: PASS")
