#include "Capture.h"
namespace crash::cases::vehicle_dynamics::diagnostics::qeph_rejection {
void CaptureRejected(CaptureState& state, native::QephBatch& batch, tl::fea::FENodalState& owner,
    const tl::fea::NodalTrialToken& token, const tl::fea::NodalPreparedView& prepared,
    const native::BatchReport& original, const tl::fea::ShellPhysicalBinding& physical) noexcept {
    if (state.seen || original.status == native::BatchStatus::Success) return;
    SourceRequest request;
    if (original.element != UINT32_MAX && physical.execution()) {
        if (const auto* parent = physical.execution()->parent(tl::fea::ShellBindingFamily::Qeph, original.element)) {
            const auto& source = parent->source;
            request.source = {true, source.source_parent_id, source.source_part_id, source.material_id, source.section_id};
            request.law_available = true;
            request.law = parent->law;
        }
    }
    // The native diagnostic copy is failure-atomic and uses the same borrowed
    // prepared owner before the caller's mandatory discard. Never substitute
    // its result for the physical operation's original BatchReport.
    const auto report = batch.CopyRejectedCandidate(owner, token, prepared, original, &state.input);
    FinishCapture(state, original, report, request);
}
}
