#pragma once
#include "Values.h"
#include "output/ArtifactIO.h"
namespace crash::cases::vehicle_self_contact::native::coated::detail {
// Shared source packing for the existing assessment and downstream phase
// diagnostics. View borrows this object's arrays; no physical owner or clock.
class TopologyInput {
  public:
    TopologyInput(const Inputs& input, const Classification& classified, const Order& order)
        : units_(input.units) {
        output::Require(classified.roles.size() == input.shells.size(), "Coated role domain differs from source");
        ids_.reserve(input.nodes.size());
        coordinates_.reserve(3*input.nodes.size());
        primary_.reserve(order.primary_to_physical.size());
        for (const auto& node : input.nodes) {
            ids_.push_back(node.source_id);
            const auto x = node.native_position;
            coordinates_.insert(coordinates_.end(), {x.x, x.y, x.z});
        }
        for (const auto physical : order.primary_to_physical) {
            output::Require(physical < input.shells.size(), "Coated primary mapping is outside source");
            auto face = input.shells[physical].primary;
            face.side_role = SideRole(classified.roles[physical].state);
            primary_.push_back(face);
        }
        const auto packed_bytes = ids_.capacity()*sizeof(std::uint64_t) + coordinates_.capacity()*sizeof(double);
        output::Require(packed_bytes <= 2*input.nodes.size()*(sizeof(std::uint64_t)+3*sizeof(double)) &&
            primary_.capacity() <= 2*order.primary_to_physical.size(),
            "TL input packing exceeded its reserved capacity");
    }
    s::Input View() const noexcept {
        s::Input view;
        view.profile = s::Profile::ResolvedShellSides;
        view.topology = s::TopologyPolicy::NativeResolvedShellSides;
        view.node_source_ids = ids_.data(); view.node_count = ids_.size();
        view.positions = {coordinates_.data(), std::uint32_t(ids_.size()), 3, 1};
        view.primary = primary_.data(); view.primary_count = primary_.size();
        view.coordinates = s::Coordinates::Native; view.units = units_;
        view.source_generation = 1; // Immutable source assessment, not physical time.
        return view;
    }
  private:
    std::vector<std::uint64_t> ids_;
    std::vector<double> coordinates_;
    std::vector<s::PrimaryFace> primary_;
    n::UnitScale units_;
};
}
