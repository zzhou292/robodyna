#include "Internal.h"

namespace crash::modelio::physical_scope {
struct PhysicalScope::Storage {
    Storage(const rigid::point_mass::Source& mass, const tied_shell::TiedShellDeclaration& tie,
            const type13::SourceType13& beam, const solid_source::VehicleSolidSource& solid)
        : masses(mass), tied(tie), beams(beam), solids(solid) {}
    rigid::point_mass::Source masses;
    tied_shell::TiedShellDeclaration tied;
    type13::SourceType13 beams;
    solid_source::VehicleSolidSource solids;
    Forecast forecast;
    Data data;
};
PhysicalScope PhysicalScope::Prepare(const rigid::point_mass::Source& masses,
    const tied_shell::TiedShellDeclaration& tied, const type13::SourceType13& beams,
    const solid_source::VehicleSolidSource& solids, Limits limits) {
    const auto forecast = Preflight(masses, tied, beams, solids, limits);
    auto next = std::make_shared<Storage>(masses, tied, beams, solids);
    const auto& canonical = tied.canonical().data();
    auto nodes = detail::Decode<SourceId>(canonical, "node_ids");
    detail::Require(nodes.size() == canonical.canonical_nodes && !nodes.empty() && nodes.front() &&
        std::adjacent_find(nodes.begin(), nodes.end(), std::greater_equal<SourceId>()) == nodes.end(),
        "Physical source canonical node identities are not strictly ordered");
    auto& data = next->data;
    data.spotwelds = detail::ReadSpotwelds(tied.data().sources, limits);
    detail::BuildRoles(masses, beams, solids, nodes, data);
    detail::BuildGroups(masses.rigid_source(), tied, nodes, data, limits);
    detail::BuildEvidence(canonical, solids, nodes, data, limits);
    data.owned_payload_bytes = detail::OwnedPayload(data, limits);
    detail::Require(data.owned_payload_bytes <= forecast.result_reservation,
                    "Physical census owned payload exceeded preflight");
    next->forecast = forecast;
    return PhysicalScope(std::move(next));
}
const Data& PhysicalScope::data() const noexcept { return storage_->data; }
const Forecast& PhysicalScope::forecast() const noexcept { return storage_->forecast; }
const rigid::point_mass::Source& PhysicalScope::point_mass_source() const noexcept { return storage_->masses; }
const tied_shell::TiedShellDeclaration& PhysicalScope::tied_source() const noexcept { return storage_->tied; }
const type13::SourceType13& PhysicalScope::type13_source() const noexcept { return storage_->beams; }
const solid_source::VehicleSolidSource& PhysicalScope::solid_source() const noexcept { return storage_->solids; }
} // namespace crash::modelio::physical_scope
