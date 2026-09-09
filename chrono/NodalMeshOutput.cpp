#include "NodalMeshOutput.h"
#include "lib_src/solvers/FENodalState.h"

namespace crash::visual {
Report NodalMeshOutput::Initialize(const tl::fea::FENodalState& owner, const Binding& binding) {
    if (identity_.owner) return {Status::InvalidBinding, "Output is already initialized"};
    const auto stamp = owner.accepted();
    if (!stamp.owner_id || !stamp.node_count)
        return {Status::InvalidBinding, "TL source is not initialized"};
    if (stamp.owner_id != binding.identity.owner)
        return {Status::WrongOwner, "Output binding does not identify this TL owner"};
    if (stamp.node_count != binding.tl_node_count || stamp.node_count > position_.size() / 3)
        return {Status::InvalidBinding, "Output binding does not match the TL node space"};
    const auto report = surface_.Initialize(binding);
    if (report.status != Status::Ok) return report;
    identity_ = binding.identity;
    node_count_ = stamp.node_count;
    return {Status::Ok, "TL accepted output bound"};
}

Report NodalMeshOutput::Publish(tl::fea::FENodalState& owner) {
    if (!identity_.owner) return {Status::NotInitialized, "TL output is not bound"};
    const auto current = owner.accepted();
    if (current.owner_id != identity_.owner)
        return {Status::WrongOwner, "Snapshot source is a different TL owner"};
    if (current.node_count != node_count_)
        return {Status::InvalidFrame, "TL source node count changed"};
    // Avoid a readback when no new accepted state exists. Trial readiness is
    // deliberately not consulted: the only publication authority is the owner.
    const auto* shown = surface_.frame();
    if (shown && (current.epoch <= shown->epoch || current.time <= shown->time))
        return {Status::StaleFrame, "No newer accepted TL frame"};
    tl::fea::NodalStamp stamp;
    const auto copied = owner.CopyAccepted({position_.data(), velocity_.data(), node_count_}, &stamp);
    if (copied.status != tl::fea::NodalStatus::Ok)
        return {Status::InvalidFrame, copied.message};
    if (stamp.owner_id != identity_.owner || stamp.node_count != node_count_)
        return {Status::InvalidFrame, "Accepted readback changed its source identity"};
    return surface_.Publish({position_.data(), velocity_.data(), nullptr, node_count_},
                            {identity_, stamp.epoch, stamp.time});
}
}  // namespace crash::visual
