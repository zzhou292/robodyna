#include "Publication.h"
#include "case/vehicle_run/SelfContactDocument.h"
#include "case/vehicle_self_contact/SelfContactStageError.h"
#include "output/ArtifactIO.h"

namespace crash::cases::vehicle_run::diagnostics::detail {
void CheckCapturedFailure(const Result& run,
    const tlfea::contact::SelfContactTransactionReport& report, std::uint64_t epoch) {
    output::Require(run.loop.kind == StopKind::PhysicsRejected &&
        epoch == run.loop.progress.accepted.epoch && run.rejected_self_contact.has_value(),
        "Failure capture differs from the normal controller rejection or accepted epoch");
    const auto& rejected = *run.rejected_self_contact;
    output::Require(rejected.stage() == vehicle_self_contact::SelfContactRuntimeStage::CandidateSeal &&
        vehicle_run::detail::SelfContactErrorDocument(vehicle_self_contact::SelfContactStageError(report,
            vehicle_self_contact::SelfContactRuntimeStage::CandidateSeal, 0)) ==
        vehicle_run::detail::SelfContactErrorDocument(rejected),
        "Captured typed failure differs from the normal controller rejection");
}
FailureReport PublishEvidence(const Result& run,const CaptureEvidence& failure,
    const std::filesystem::path& destination,const ExportOperation& export_operation) {
    FailureReport result;
    if (!failure.seen) {
        if (run.loop.kind == StopKind::PhysicsRejected)
            result.status = FailureStatus::RejectionWithoutPair;
        return result;
    }
    if (!failure.complete || !failure.owners_equivalent) {
        result.status = FailureStatus::CaptureIncomplete;
        result.error = failure.error;
        return result;
    }
    // Preserve the captured evidence before checking consistency, as the
    // acceptance harness did. Neither export nor checking alters run.Result.
    try {
        result.sha256 = export_operation(destination);
        result.manifest = destination / "failure.json";
    } catch (const std::exception& error) {
        result.status = FailureStatus::ExportFailed;
        result.error = std::string(error.what()).substr(0, 4096);
        return result;
    } catch (...) {
        result.status = FailureStatus::ExportFailed;
        result.error = "Unknown failure companion export error";
        return result;
    }
    try {
        CheckCapturedFailure(run, failure.report, failure.accepted_epoch);
        result.status = FailureStatus::Captured;
    } catch (const std::exception& error) {
        result.status = FailureStatus::CaptureIncomplete;
        result.error = std::string(error.what()).substr(0, 4096);
    } catch (...) {
        result.status = FailureStatus::CaptureIncomplete;
        result.error = "Unknown captured failure consistency error";
    }
    return result;
}
FailureReport PublishFailure(const Result& run,const fixture::CandidateFailureFixture& failure,
    const std::filesystem::path& destination) {
    const CaptureEvidence evidence{failure.seen(),failure.complete(),failure.owners_equivalent(),
        failure.phase().accepted_epoch,failure.report(),failure.error()};
    return PublishEvidence(run,evidence,destination,
        [&](const auto& directory) { return failure.Export(directory); });
}
} // namespace crash::cases::vehicle_run::diagnostics::detail
