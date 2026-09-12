#pragma once
#include "case/vehicle_wall/VehicleWallSetup.h"
#include "lib_src/solvers/NodalCinStructuralLimit.h"
#include "output/ArtifactIO.h"
#include <algorithm>
#include <vector>

namespace crash::cases::vehicle_dynamics::limiter {
struct SourceNodes {
    std::vector<std::size_t> direct, with_donors;
    bool Contains(std::size_t node) const noexcept {
        return std::binary_search(with_donors.begin(),with_donors.end(),node);
    }
    bool Direct(std::size_t node) const noexcept {
        return std::binary_search(direct.begin(),direct.end(),node);
    }
};
inline SourceNodes SelectNodes(const vehicle_wall::VehicleWallSetup& setup,
                              const tl::fea::NodalCinStructuralLimit& receipt) {
    using Kind = tl::fea::NodalCinLimitKind;
    const auto& physical = setup.execution().model();
    const auto& domain = physical.source_domain().domain();
    output::Require(receipt.source_instance_id == domain.source_instance_id(),
                    "Limiter source instance differs from retained physical source");
    SourceNodes result;
    result.direct.reserve(1024);
    if(receipt.values.kind == Kind::Unbounded) return result;
    output::Require(receipt.values.node < domain.node_count() &&
        domain.nodes()[receipt.values.node].source_id == receipt.source_node_id,
        "Limiter source node differs from the actual physical domain");
    if(receipt.values.kind == Kind::RigidTrace) {
        const auto& binding = physical.rigid_assembly();
        output::Require(receipt.values.group < binding.groups().size(),"Limiter rigid group is absent");
        const auto& group = binding.groups()[receipt.values.group];
        output::Require(group.source_kind == receipt.source_kind && group.source_id == receipt.source_group_id &&
            group.source_node_set_id == receipt.source_node_set_id && group.member_count <= 1024 &&
            group.member_offset <= binding.members().size() &&
            group.member_count <= binding.members().size()-group.member_offset,
            "Limiter rigid source identity or member extent differs");
        for(std::size_t i = 0; i < group.member_count; ++i)
            result.direct.push_back(binding.members()[group.member_offset+i].domain_node);
    } else {
        output::Require(receipt.values.kind == Kind::OrdinaryTranslation ||
            receipt.values.kind == Kind::OrdinaryRotation,"Limiter kind is unavailable");
        result.direct.push_back(receipt.values.node);
    }
    std::sort(result.direct.begin(),result.direct.end());
    const auto cin = setup.attachments().attachments().model().rows();
    output::Require(cin.count <= 65536,"Limiter CIN diagnostic extent exceeds fixed report scope");
    result.with_donors.reserve(result.direct.size()+cin.count);
    result.with_donors.insert(result.with_donors.end(),result.direct.begin(),result.direct.end());
    // A secondary cannot also be a master in the admitted CIN model. One pass
    // collects the actual direct donors; no undirected transitive closure.
    for(const auto& row : cin) {
        bool contributes = false;
        for(const auto node : row.master_domain_nodes) contributes |= result.Direct(node);
        if(contributes) result.with_donors.push_back(row.secondary_domain_node);
    }
    std::sort(result.with_donors.begin(),result.with_donors.end());
    result.with_donors.erase(std::unique(result.with_donors.begin(),result.with_donors.end()),result.with_donors.end());
    return result;
}
} // namespace crash::cases::vehicle_dynamics::limiter
