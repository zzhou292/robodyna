#pragma once

#include "lib_src/collision/Q4PlanarContact.h"

namespace crash::case_data {
inline bool ValidGuidedContactBackend(tlfea::contact::Q4PlanarIntegrationBackend backend) noexcept {
    using Backend=tlfea::contact::Q4PlanarIntegrationBackend;
    return backend==Backend::ScalarDyadicSquares || backend==Backend::RectangularDyadic;
}
inline bool ValidGuidedContactDepths(tlfea::contact::Q4PlanarIntegrationBackend backend,
                                     std::uint32_t u,std::uint32_t v,std::uint32_t deepest) noexcept {
    return ValidGuidedContactBackend(backend) && u<=tlfea::contact::MaxQ4IntegrationDepth &&
        v<=tlfea::contact::MaxQ4IntegrationDepth && deepest==(u>v?u:v) &&
        (backend!=tlfea::contact::Q4PlanarIntegrationBackend::ScalarDyadicSquares || (u==deepest && v==deepest));
}
// Execution metadata only. These checks do not certify contact integrals,
// owner/attempt identities, physical geometry or a completed trajectory.
inline bool ValidGuidedContactPartition(const tlfea::contact::Q4PlanarContactDiagnostics& d,
                                         tlfea::contact::Q4PlanarIntegrationBackend expected) noexcept {
    return d.valid && d.integration_backend==expected &&
        ValidGuidedContactDepths(expected,d.deepest_u,d.deepest_v,d.deepest_leaf) &&
        d.parent_count<=tlfea::contact::MaxQ4PlanarParents && d.covered_count<=d.parent_count &&
        (d.covered_count || d.deepest_leaf==0) &&
        d.leaves>=d.covered_count && d.leaves<=d.covered_count*tlfea::contact::MaxQ4IntegrationLeaves &&
        d.visited>=d.leaves && d.visited<=d.covered_count*tlfea::contact::MaxQ4IntegrationVisits;
}
inline bool ValidGuidedContactPartition(const tlfea::contact::Q4PlanarParentResult& p,
                                         tlfea::contact::Q4PlanarIntegrationBackend expected) noexcept {
    const auto& r=p.integration;
    if (p.integration_backend!=expected || !ValidGuidedContactDepths(expected,p.deepest_u,p.deepest_v,r.deepest_leaf))
        return false;
    if (!p.covered) return !r.valid && !p.deepest_u && !p.deepest_v && !r.leaf_count && !r.visited;
    return r.valid && r.leaf_count>0 && r.leaf_count<=tlfea::contact::MaxQ4IntegrationLeaves &&
        r.visited>=r.leaf_count && r.visited<=tlfea::contact::MaxQ4IntegrationVisits;
}
inline bool ValidGuidedContactPartition(const tlfea::contact::Q4PlanarContactDiagnostics& d,
                                         const tlfea::contact::Q4PlanarParentResult* parents,std::size_t count,
                                         tlfea::contact::Q4PlanarIntegrationBackend expected) noexcept {
    if (!ValidGuidedContactPartition(d,expected) || count!=d.parent_count || (!parents && count)) return false;
    std::uint32_t covered=0,leaves=0,visited=0,u=0,v=0;
    for (std::size_t p=0;p<count;++p) {
        const auto& parent=parents[p];
        if (!ValidGuidedContactPartition(parent,expected)) return false;
        if (!parent.covered) continue;
        ++covered; leaves+=parent.integration.leaf_count; visited+=parent.integration.visited;
        if (parent.deepest_u>u) u=parent.deepest_u;
        if (parent.deepest_v>v) v=parent.deepest_v;
    }
    return d.covered_count==covered && d.leaves==leaves && d.visited==visited && d.deepest_u==u && d.deepest_v==v;
}
} // namespace crash::case_data
