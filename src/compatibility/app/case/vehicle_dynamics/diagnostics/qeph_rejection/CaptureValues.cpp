#include "Capture.h"
#include <algorithm>
namespace crash::cases::vehicle_dynamics::diagnostics::qeph_rejection {
namespace {
bool Same(const native::RejectedReportValues& a, const native::BatchReport& b) noexcept {
    return a.status == b.status && a.element == b.element && a.node == b.node &&
        a.element_status == b.element_status && a.nodal_status == b.nodal_status;
}
bool SameSource(const native::RejectedSourceIds& a, const native::RejectedSourceIds& b) noexcept {
    return a.available == b.available && a.parent == b.parent && a.part == b.part &&
        a.material == b.material && a.section == b.section;
}
void Message(CaptureState& state, const char* value) noexcept {
    state.message.fill(0);
    if (!value) return;
    for (std::size_t i = 0; i + 1 < state.message.size() && value[i]; ++i) state.message[i] = value[i];
}
}
const char* StatusName(native::RejectedCaptureStatus value) noexcept {
    using S = native::RejectedCaptureStatus;
    switch (value) {
        case S::Captured: return "captured";
        case S::InvalidInput: return "invalid_input";
        case S::NoRejectedCandidate: return "no_rejected_candidate";
        case S::StaleTrial: return "stale_trial";
        case S::UnsupportedFailure: return "unsupported_failure";
        case S::InvalidSource: return "invalid_source";
        case S::ResourceLimit: return "resource_limit";
        case S::DeviceFailure: return "device_failure";
    }
    return "unknown";
}
void FinishCapture(CaptureState& state, const native::BatchReport& original,
    const native::RejectedCaptureReport& report, const SourceRequest& request) noexcept {
    state.seen = true;
    state.complete = false;
    state.original = {original.status, original.element, original.node, original.element_status, original.nodal_status};
    state.requested = request;
    state.status = report.status;
    state.cuda_status = static_cast<int>(report.cuda_status);
    state.metadata_available = report.metadata_available;
    if (report.metadata_available) state.metadata = report.metadata;
    Message(state, report.message);
    if (report.status != native::RejectedCaptureStatus::Captured) return;
    const auto& captured = state.input;
    const bool identity = report.metadata_available && Same(report.metadata.original, original) &&
        Same(captured.metadata.original, original) &&
        captured.metadata.owner.owner_id == report.metadata.owner.owner_id &&
        captured.metadata.owner.epoch == report.metadata.owner.epoch &&
        captured.metadata.candidate.attempt == report.metadata.candidate.attempt &&
        captured.metadata.candidate.base_epoch == captured.metadata.owner.epoch &&
        captured.metadata.accepted.epoch == captured.metadata.owner.epoch &&
        !captured.metadata.candidate.valid;
    const bool source = (!request.source.available || SameSource(request.source, captured.source)) &&
        (!request.law_available || request.law == captured.law);
    if (!identity || !source) {
        state.status = native::RejectedCaptureStatus::InvalidSource;
        Message(state, "Captured QEPH inputs differ from the requested report, attempt or source mapping");
        return;
    }
    state.complete = true;
}
}
