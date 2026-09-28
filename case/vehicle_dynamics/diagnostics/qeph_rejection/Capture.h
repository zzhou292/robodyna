#pragma once
#include "Codec.h"
#include "lib_src/assembly/ShellPhysicalBinding.h"
#include <array>
namespace crash::cases::vehicle_dynamics::diagnostics::qeph_rejection {
struct SourceRequest {
    native::RejectedSourceIds source;
    bool law_available = false;
    tl::fea::ShellSectionLaw law = tl::fea::ShellSectionLaw::Unspecified;
};
// Owned diagnostic values survive trial discard. No owner, token, stream,
// source pointer or partially written failed force packet is retained here.
struct CaptureState {
    bool seen = false, complete = false, metadata_available = false;
    native::RejectedCaptureStatus status = native::RejectedCaptureStatus::NoRejectedCandidate;
    int cuda_status = 0;
    native::RejectedReportValues original;
    SourceRequest requested;
    native::RejectedCandidateMetadata metadata;
    native::RejectedCandidateInput input;
    std::array<char, 512> message{};
};
inline constexpr std::size_t CaptureWorkspaceBytes = sizeof(CaptureState) +
    native::ForecastRejectedCandidateCapture().peak_host_bytes + SerializationWorkspaceBytes;
const char* StatusName(native::RejectedCaptureStatus) noexcept;
// Shared value admission for live capture and host fault-policy tests. It
// never changes a physical report or claims a failed diagnostic is a snapshot.
void FinishCapture(CaptureState&, const native::BatchReport&,
    const native::RejectedCaptureReport&, const SourceRequest&) noexcept;
void CaptureRejected(CaptureState&, native::QephBatch&, tl::fea::FENodalState&,
    const tl::fea::NodalTrialToken&, const tl::fea::NodalPreparedView&,
    const native::BatchReport&, const tl::fea::ShellPhysicalBinding&) noexcept;
}
