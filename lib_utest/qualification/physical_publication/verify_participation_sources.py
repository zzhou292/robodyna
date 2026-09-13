#!/usr/bin/env python3
"""Focused source proof for fixed scratch participation publication wiring."""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
HEADER = ROOT / "lib_src/elements/publication/ShellPhysicalScratchParticipation.h"
SOURCE = ROOT / "lib_src/elements/publication/ShellPhysicalScratchParticipation.cpp"
TRANSACTION = ROOT / "lib_src/elements/publication/PhysicalTransaction.cpp"
ELEMENTS_CMAKE = ROOT / "lib_src/elements/ShellBatchPublication.cmake"
ELEMENTS_BAZEL = ROOT / "lib_src/elements/BUILD.bazel"
CUDA_WIRING = Path(__file__).resolve().parent / "Cuda.cmake"


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
    "Self-contact assembly can be recorded only by its transaction",
    "Self-contact candidate can be sealed only by its transaction",
):
    require(source, token, SOURCE)
require(transaction, "ValidatePhysicalScratchSeal(owner,authentic)", TRANSACTION)
require(transaction, "ConsumePhysicalScratchSeal();", TRANSACTION)
commit = transaction.index("owner.Commit(token)")
if "participation" in transaction[commit:].lower():
    raise RuntimeError(
        f"{TRANSACTION}: participation operation appears after owner commit")
for wiring in (ELEMENTS_CMAKE, ELEMENTS_BAZEL):
    require(wiring.read_text(), "ShellPhysicalScratchParticipation.cpp", wiring)
require(CUDA_WIRING.read_text(), "ParticipationTest.cu", CUDA_WIRING)
print("fixed scratch participation source proof: PASS")
