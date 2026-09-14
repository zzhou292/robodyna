#!/usr/bin/env python3
"""Light source/build proof for the fixed wall+self runtime composition."""

from pathlib import Path
import argparse


def require(condition: bool, message: str) -> None:
    if not condition:
        raise RuntimeError(message)


def ordered(text: str, names: list[str], context: str) -> None:
    positions = [text.find(name) for name in names]
    require(all(value >= 0 for value in positions),
            f"{context}: missing ordered stage {names}")
    require(positions == sorted(positions),
            f"{context}: stages are out of order {names}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("app_root", type=Path)
    parser.add_argument("tl_root", type=Path)
    args = parser.parse_args()

    case = args.app_root / "case"
    dynamics = case / "vehicle_dynamics"
    contact = case / "vehicle_self_contact"
    wall = case / "vehicle_wall"

    trial = (dynamics / "Trial.cpp").read_text()
    ordered(
        trial,
        ["s.solids.AssembleAccepted", "wall->Assemble",
         "self_contact->Assemble", "activity->UploadAttempt",
         "s.owner.SealAssembly", "AdvanceStaggeredCin"],
        "accepted transaction")
    ordered(
        trial,
        ["s.solids.EvaluateCandidate", "s.publication.PreparePhysical",
         "wall->Evaluate", "self_contact->SealCandidate"],
        "candidate transaction")

    commit = (dynamics / "VehiclePhysicalDynamics.cpp").read_text()
    require(commit.count("SealPhysicalScratchParticipation(") == 1,
            "dynamics must seal the merged scratch roster exactly once")
    ordered(
        commit,
        ["receipts.mapped_wall", "receipts.self_contact",
         "SealPhysicalScratchParticipation(", "CommitPhysical("],
        "common commit")
    ordered(
        commit,
        ["if(wall) wall->Discard()",
         "if(self_contact) self_contact->Discard()",
         "state().owner.Discard()", "state().publication.DiscardTrial()"],
        "common discard")
    require(
        commit.find("// No allocation, device call, readback or other "
                    "fallible work after success.") >
        commit.find("CommitPhysical("),
        "owner commit must retain the no-fallible-work boundary")

    prepare = (contact / "runtime" / "Prepare.cpp").read_text()
    combined = prepare[prepare.find("LoadedWallSelfContact::Prepare("):]
    require(combined.count(
        "ConfigurePhysicalScratchParticipation(") == 1,
        "combined factory must configure the fixed roster exactly once")
    ordered(
        combined,
        ["VehicleWallStartup::PrepareUnconfigured",
         "VehicleSelfContactStartup::PrepareUnconfigured",
         "{wall_entry, self_entry}"],
        "combined startup")
    require("wall_without_publication" in prepare and
            "transaction_without_publication" in prepare and
            "participation.publication_host_bytes" in prepare,
            "combined forecast must replace standalone publication charges")

    startup_header = (
        contact / "VehicleSelfContactStartup.h").read_text()
    private = startup_header.find("private:")
    require(private >= 0 and
            startup_header.find("roster_entry()", private) >= 0 and
            startup_header.find("roster_entry()") >= private,
            "self roster issuer must remain private")
    require("FirstProfileStiffnessPerAreaNPerM3 = 2e9" in
            startup_header,
            "first profile stiffness must remain fixed at 2e9 N/m3")
    require("config.event_capacity <= transaction.max_global_events" in
            (contact / "VehicleSelfContactStartup.cpp").read_text(),
            "force capacity must fit the complete global event ledger")
    setup = (contact / "SelectedSelfContactSource.h").read_text()
    require("source_fields" not in
            (contact / "VehicleSelfContactStartup.cpp").read_text(),
            "runtime startup must not apply original contact fields")
    require("RuntimeCoefficientCensus runtime_coefficients" in setup,
            "setup must retain explicit zero applied-coefficient census")

    wall_stage = (wall / "loaded" / "Stages.cpp").read_text()
    require("SealPhysicalScratchParticipation(" not in wall_stage,
            "wall stage must contribute a receipt, not seal independently")

    cmake = (
        contact / "VehicleSelfContactRuntime.cmake").read_text()
    require("SelfContactTransaction.cmake" in cmake and
            "tl_self_contact_transaction" in cmake,
            "app CMake must own the TL transaction target")
    tl_cmake = (
        args.tl_root / "lib_src" / "collision" /
        "SelfContactTransaction.cmake").read_text()
    require("SelfContactPhysicalActivity.cmake" in tl_cmake and
            "tl_self_contact_physical_activity" in tl_cmake,
            "TL CMake transaction must include physical activity")
    bazel = (
        args.tl_root / "lib_src" / "collision" /
        "BUILD.bazel").read_text()
    transaction = bazel[bazel.find('name = "self_contact_transaction"'):]
    require('":self_contact_physical_activity"' in transaction,
            "TL Bazel transaction must depend on physical activity")
    transaction_cuda = (
        args.tl_root / "lib_utest" / "qualification" /
        "self_contact_transaction" / "CudaTest.cu").read_text()
    for control in [
            "SingleParentZeroPairZeroEventStillParticipatesAndCommits",
            "AcceptedInteriorEeForceCandidateRetryAndRollbackKeepForceSti",
            "ActualT3RemovalFiltersCandidateAndLongInactiveRetryCommits",
            "ExactPassThroughUnresolvedReasonRollsBackAndRetriesExactly"]:
        require(control in transaction_cuda,
                f"qualified TL transaction control is missing: {control}")
    require("ParticipationFailure" in transaction_cuda and
            "const auto expired = accepted" in transaction_cuda and
            "forged.t3.attempt" in transaction_cuda,
            "missing/stale/late transaction rollback controls are absent")

    error = (contact / "SelfContactStageError.h").read_text()
    require("SelfContactTransactionReport report_" in error and
            "required_events() const noexcept" in error and
            "report.candidate > configured_event_capacity" in error,
            "app stage error must preserve the typed exact event count")
    gate = (
        case / "vehicle_startup" / "shell_execution" / "tests" /
        "self_contact" / "RuntimeGateTest.cpp").read_text()
    for value in ["376930", "337092", "315963", "653055",
                  "1584464", "5989248", "4096", "1542092504"]:
        require(value in gate,
                f"actual runtime gate is missing exact value {value}")
    require("SelfContactTransactionLimits::Vehicle(" in gate and
            "V5_SELF_CONTACT_RUNTIME" in gate and
            "SelfContactOnly::Preflight(" in gate and
            "SelfContactOnly::Prepare(" in gate and
            "LoadedWallSelfContact::Prepare(" in gate,
            "actual self-only/wall+self runtime gate is incomplete")
    require("IGNORE=1" not in gate,
            "runtime gate must not bypass any transaction result")
    fixture_cmake = (
        case / "vehicle_startup" / "shell_execution" /
        "SelfContactTests.cmake").read_text()
    for name in [
            "vehicle_self_contact_runtime_${runtime_gate}",
            "vehicle_wall_self_contact_runtime_${runtime_gate}",
            "FullV5ForecastStartupOwnsExactIdentityAndMemory",
            "FullV5OneAttemptIsTypedFailClosedAndRetryStable",
            "FullV5CombinedForecastStartupOwnsBothFixedSlotsOnce",
            "FullV5CombinedAttemptSealsBothReceiptsAndRetries"]:
        require(name in fixture_cmake,
                f"runtime CTest registration is missing: {name}")
    require("RESOURCE_LOCK vehicle_self_contact_gpu" in fixture_cmake and
            "TIMEOUT ${runtime_timeout}" in fixture_cmake and
            "RuntimeGateTest.cpp" in fixture_cmake,
            "runtime gates need source proof and finite serialized properties")

    print("vehicle self-contact runtime source/CMake/Bazel proof passed")


if __name__ == "__main__":
    main()
