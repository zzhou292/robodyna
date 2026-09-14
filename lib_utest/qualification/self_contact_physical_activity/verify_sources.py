#!/usr/bin/env python3
"""Focused source proof for publication-validated self-contact activity."""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
HERE = Path(__file__).resolve().parent
HEADER = ROOT / "lib_src/collision/SelfContactPhysicalActivity.h"
TYPES = ROOT / "lib_src/collision/SelfContactPhysicalActivityTypes.h"
ACTIVITY = ROOT / "lib_src/collision/self_contact_physical_activity/Activity.cpp"
STORAGE = ROOT / "lib_src/collision/self_contact_physical_activity/Storage.h"
PUBLICATION = ROOT / "lib_src/elements/publication/PhysicalReadback.cpp"
CMAKE = ROOT / "lib_src/collision/SelfContactPhysicalActivity.cmake"
BAZEL = ROOT / "lib_src/collision/BUILD.bazel"
CUDA = HERE / "CudaTest.cu"


def require(text: str, token: str, where: Path) -> None:
    if token not in text:
        raise RuntimeError(f"{where}: missing {token!r}")


header = HEADER.read_text()
types = TYPES.read_text()
activity = ACTIVITY.read_text()
storage = STORAGE.read_text()
publication = PUBLICATION.read_text()

for token in (
    "ValidatePhysicalSources(",
    "ValidateAcceptedActivitySources(",
    "ValidatePhysicalAssembly(owner, token, view)",
    "ValidatePhysicalCandidate(",
    "CopyAcceptedParentActivity(",
    "CopyPreparedParentActivity(",
    "source.family_index",
    "ValidateTransition(&base, &current, 1)",
):
    require(activity, token, ACTIVITY)

for token in (
    "ShellPhysicalBinding physical",
    "ShellPhysicalParticipants participants",
    "ShellPhysicalPublicationIdentity identity",
    "HostArena arena",
    "std::uint64_t generation",
):
    require(storage, token, STORAGE)

for token in (
    "std::weak_ptr<self_contact_physical_activity::State>",
    "SelfContactActivityView activity() const noexcept",
    "base_buffer_identity_",
    "current_buffer_identity_",
):
    require(types, token, TYPES)

accepted_signature = header[
    header.index("CaptureAccepted("):header.index("CapturePrepared(")
]
if "SelfContactActivityView" in accepted_signature:
    raise RuntimeError(f"{HEADER}: CaptureAccepted accepts caller activity")

for token in (
    "ValidatePhysicalAssembly(",
    "AuthenticateAssemblyView(token,view)",
    "assembled_epoch==view.accepted.base_epoch",
    "assembled_attempt==view.attempt",
    "ValidatePhysicalCandidate(",
    "SamePhysicalDiagnostics(expected,state.physical->candidate)",
):
    require(publication, token, PUBLICATION)

for forbidden in ("std::vector", "std::map", "std::function",
                  "cudaMalloc"):
    if forbidden in activity:
        raise RuntimeError(
            f"{ACTIVITY}: forbidden per-attempt allocation token {forbidden!r}")

for path in (CMAKE, BAZEL):
    text = path.read_text()
    for token in (
        "self_contact_physical_activity/Activity.cpp",
        "self_contact_physical_activity/Layout.cpp",
        "self_contact_physical_activity/Values.cpp",
    ):
        require(text, token, path)

cuda = CUDA.read_text()
for token in (
    "InitializeExecutionCatalog",
    "InitializeExecution",
    "CaptureAccepted",
    "CapturePrepared",
    "CopyAcceptedParentActivity",
    "LateNonfinite",
    "DiscardTrial",
):
    require(cuda, token, CUDA)

transaction_types = ROOT / "lib_src/collision/SelfContactTransactionTypes.h"
transaction_storage = (
    ROOT / "lib_src/collision/self_contact_transaction/Storage.h")
transaction_accepted = (
    ROOT / "lib_src/collision/self_contact_transaction/Transaction.cpp")
transaction_prepared = (
    ROOT / "lib_src/collision/self_contact_transaction/Candidate.cpp")
require(transaction_types.read_text(), "SelfContactPhysicalActivityForecast",
        transaction_types)
require(transaction_storage.read_text(),
        "SelfContactPhysicalActivity physical_activity", transaction_storage)
require(transaction_accepted.read_text(),
        "physical_activity.CaptureAccepted", transaction_accepted)
require(transaction_prepared.read_text(),
        "physical_activity.CapturePrepared", transaction_prepared)

print("self-contact physical activity source proof: PASS")
