#include "NodalMeshOutput.h"
#include <cmath>

namespace crash::visual {
namespace {
bool ValidTiming(const tl::fea::NodalStamp& s, NodalOutputTiming timing) noexcept {
    using namespace tl::fea;
    if (!std::isfinite(s.time) || s.time < 0 || !std::isfinite(s.fixed_dt) || s.fixed_dt <= 0 ||
        !std::isfinite(s.velocity_time)) return false;
    if (timing == NodalOutputTiming::CollocatedOnly)
        return IsCollocatedNodalTiming(s.temporal_scheme, s.velocity_phase) && s.velocity_time == s.time;
    if (timing != NodalOutputTiming::StaggeredHalfKick || !s.has_rotations ||
        s.temporal_scheme != NodalTemporalScheme::StaggeredHalfKickStart) return false;
    if (!s.epoch)
        return s.time == 0 && s.velocity_time == 0 && s.velocity_phase == NodalVelocityPhase::Collocated &&
               !s.reactions_valid && s.reaction_kick_dt == 0;
    // Use the owner's actual previous endpoint, not time-dt or epoch*dt: those
    // recomputations can round differently from the accepted recurrence.
    return s.velocity_phase == NodalVelocityPhase::PreviousMidpoint && s.reactions_valid &&
           s.reaction_base_epoch == s.epoch - 1 && std::isfinite(s.reaction_time) && s.reaction_time >= 0 &&
           s.time == s.reaction_time + s.fixed_dt &&
           s.velocity_time == s.reaction_time + .5 * s.fixed_dt &&
           s.reaction_time < s.velocity_time && s.velocity_time < s.time &&
           s.reaction_kick_dt == (s.epoch == 1 ? .5 * s.fixed_dt : s.fixed_dt);
}

bool SameBinding(const tl::fea::NodalStamp& a, const tl::fea::NodalStamp& b) noexcept {
    return a.owner_id == b.owner_id && a.node_count == b.node_count && a.fixed_dt == b.fixed_dt &&
           a.temporal_scheme == b.temporal_scheme && a.has_rotations == b.has_rotations;
}
}  // namespace

Report NodalMeshOutput::Initialize(const tl::fea::FENodalState& owner, const Binding& binding) {
    return Initialize(owner, binding, NodalOutputTiming::CollocatedOnly);
}

Report NodalMeshOutput::Initialize(const tl::fea::FENodalState& owner, const Binding& binding,
                                 NodalOutputTiming timing) {
    if (identity_.owner) return {Status::InvalidBinding, "Output is already initialized"};
    const auto stamp = owner.accepted();
    if (!stamp.owner_id || !stamp.node_count)
        return {Status::InvalidBinding, "TL source is not initialized"};
    if (!ValidTiming(stamp, timing))
        return {Status::InvalidBinding, "TL temporal scheme or phase does not match the selected output protocol"};
    if (stamp.owner_id != binding.identity.owner)
        return {Status::WrongOwner, "Output binding does not identify this TL owner"};
    if (stamp.node_count != binding.tl_node_count || stamp.node_count > tl::fea::MaxTranslationNodes)
        return {Status::InvalidBinding, "Output binding does not match the TL node space"};
    const auto report = surface_.Initialize(binding);
    if (report.status != Status::Ok) return report;
    identity_ = binding.identity;
    bound_ = stamp;
    timing_ = timing;
    return {Status::Ok, "TL accepted output bound"};
}

Report NodalMeshOutput::Publish(tl::fea::FENodalState& owner) {
    if (!identity_.owner) return {Status::NotInitialized, "TL output is not bound"};
    const auto current = owner.accepted();
    if (!ValidTiming(current, timing_))
        return {Status::InvalidFrame, "TL temporal scheme or phase does not match the selected output protocol"};
    if (current.owner_id != identity_.owner)
        return {Status::WrongOwner, "Snapshot source is a different TL owner"};
    if (!SameBinding(current, bound_))
        return {Status::InvalidFrame, "TL source node count, timestep or storage changed"};
    // Avoid a readback when no new accepted state exists. Trial readiness is
    // deliberately not consulted: the only publication authority is the owner.
    const auto* shown = surface_.frame();
    if (shown && (current.epoch <= shown->epoch || current.time <= shown->time))
        return {Status::StaleFrame, "No newer accepted TL frame"};
    const unsigned next = 1 - published_;
    auto& staged = captures_[next];
    tl::fea::NodalStamp stamp;
    const auto copied = owner.CopyAccepted({staged.position.data(), staged.velocity.data(), bound_.node_count,
        bound_.has_rotations ? staged.orientation.data() : nullptr,
        bound_.has_rotations ? staged.omega.data() : nullptr}, &stamp);
    if (copied.status != tl::fea::NodalStatus::Ok)
        return {Status::InvalidFrame, copied.message};
    if (!SameBinding(stamp, bound_) || !ValidTiming(stamp, timing_) ||
        stamp.epoch != current.epoch || stamp.time != current.time)
        return {Status::InvalidFrame, "Accepted readback changed its source identity"};
    // CopyAccepted validates all returned fields, including unit quaternions.
    // Only the mesh adapter decides whether the accepted geometry is drawable.
    const auto report = surface_.Publish({staged.position.data(), staged.velocity.data(), nullptr, bound_.node_count},
                                         {identity_, stamp.epoch, stamp.time});
    if (report.status != Status::Ok) return report;
    published_ = next;
    stamp_ = stamp;
    available_ = true;
    return report;
}

tl::fea::HostNodalKinematicsView NodalMeshOutput::fields() const noexcept {
    if (!available_) return {};
    const auto& saved = captures_[published_];
    return {saved.position.data(), saved.velocity.data(), bound_.has_rotations ? saved.omega.data() : nullptr,
            bound_.node_count, bound_.has_rotations ? saved.orientation.data() : nullptr};
}
}  // namespace crash::visual
