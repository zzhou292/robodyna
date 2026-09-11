#pragma once
#include "VehiclePhysicalDomain.h"
#include "output/ArtifactIO.h"
#include <set>

namespace crash::modelio::physical_domain::detail {
struct Selection {
    std::vector<GroupSelection> groups;
    std::set<std::uint64_t> point_nodes;
    Counts counts;
};
Selection Select(const std::vector<physical_scope::Group>&,
                 const std::vector<physical_scope::PointMass>&);
void PrepareTopology(const tl::fea::rigid::NodalRigidPartTopology&, const Selection&,
                     std::size_t byte_cap, tl::fea::rigid::NodalRigidPartTopology&);
} // namespace crash::modelio::physical_domain::detail
