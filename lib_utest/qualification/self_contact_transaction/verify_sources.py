#!/usr/bin/env python3
"""Focused source proof for the fixed self-contact transaction."""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
HEADER = ROOT / "lib_src/collision/SelfContactTransaction.h"
TYPES = ROOT / "lib_src/collision/SelfContactTransactionTypes.h"
CANDIDATE = ROOT / "lib_src/collision/self_contact_transaction/Candidate.cpp"
TRANSACTION = ROOT / "lib_src/collision/self_contact_transaction/Transaction.cpp"
CMAKE = ROOT / "lib_src/collision/SelfContactTransaction.cmake"
BAZEL = ROOT / "lib_src/collision/BUILD.bazel"
QUAL_CMAKE = Path(__file__).resolve().parent / "CMakeLists.txt"
QUAL_BAZEL = Path(__file__).resolve().parent / "BUILD.bazel"
CUDA = Path(__file__).resolve().parent / "CudaTest.cu"
OWNER = (Path(__file__).resolve().parent /
         "../physical_publication/OwnerStartup.cu").resolve()


def require(text: str, token: str, where: Path) -> None:
    if token not in text:
        raise RuntimeError(f"{where}: missing {token!r}")


header = HEADER.read_text()
types = TYPES.read_text()
candidate = CANDIDATE.read_text()
transaction = TRANSACTION.read_text()

storage_path = ROOT / "lib_src/collision/self_contact_transaction/Storage.h"
storage = storage_path.read_text()
for token in ("SelfContactForceAssembly force",
              "ShellPhysicalScratchParticipation participation"):
    require(storage, token, storage_path)

for token in (
    "roster_entry()",
    "AssembleAccepted",
    "SealCandidate",
    "DiscardTrial",
):
    require(header, token, HEADER)

for forbidden in (
    "SelfContactForceAssembly&",
    "ShellPhysicalScratchParticipation&",
    "participation()",
    "force()",
):
    if forbidden in header:
        raise RuntimeError(f"{HEADER}: exposes private transaction state {forbidden!r}")

for token in (
    "class SelfContactTransactionReceipt",
    "ShellPhysicalScratchParticipationReceipt participation_",
    "scratch_receipts()",
    "class SelfContactAcceptedAssemblyReceipt",
):
    require(types, token, TYPES)

for token in (
    "CopyAccepted",
    "CopyPrepared",
    "regularity->Certify",
    "discovery->Discover",
    "crossing->Certify",
    "ValidateCandidatePublications",
    "participation.SealSelfContactCandidate",
):
    require(candidate, token, CANDIDATE)

force_call = transaction.index("force.AssembleAccepted")
record_call = transaction.index(
    "participation.RecordSelfContactAcceptedAssembly")
if force_call >= record_call:
    raise RuntimeError(f"{TRANSACTION}: participation recorded before force success")

for path in (CANDIDATE,):
    text = path.read_text()
    for forbidden in ("std::vector", "std::map", "std::function",
                      "cudaMalloc", "new "):
        if forbidden in text:
            raise RuntimeError(
                f"{path}: forbidden per-attempt allocation token {forbidden!r}")

for wiring in (CMAKE, BAZEL):
    text = wiring.read_text()
    for token in ("self_contact_transaction/Arena.cpp",
                  "self_contact_transaction/Candidate.cpp",
                  "self_contact_transaction/Transaction.cpp",
                  "self_contact_transaction/Values.cpp"):
        require(text, token, wiring)

for token in (
    "SELF_CONTACT_TRANSACTION_CUDA",
    "CudaTest.cu",
    "tl_self_contact_transaction",
):
    require(QUAL_CMAKE.read_text(), token, QUAL_CMAKE)
for token in ("host_check", "source_check", "root_cuda_sources"):
    require(QUAL_BAZEL.read_text(), token, QUAL_BAZEL)

cuda = CUDA.read_text()
for token in (
    "InitializeExecutionCatalog",
    "InitializeExecution",
    "execution.Initialize",
    "rig.InitializeAgainst(physical)",
    "surface.Initialize(\n        physical",
    "publication.ConfigurePhysicalScratchParticipation(\n"
    "                rig.owner, physical",
):
    require(cuda, token, CUDA)
if "rig.fixture.physical" in cuda:
    raise RuntimeError(
        f"{CUDA}: transaction fixture fell back to legacy physical authority")

owner = OWNER.read_text()
for token in (
    "InitializeAgainst(const fe::ShellPhysicalBinding& physical",
    "InitializeMapped(q,physical",
    "InitializeMapped(t,physical",
    "InitializeMapped(b,physical",
    "InitializePhysical(owner,physical",
):
    require(owner, token, OWNER)

print("fixed self-contact transaction source proof: PASS")
