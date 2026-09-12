#include "Internal.h"
#include <optional>

namespace crash::modelio::physical_scope {
struct PhysicalScope::Storage {
    Storage(const rigid::point_mass::Source& mass, const tied_shell::TiedShellDeclaration& tie,
            const type13::SourceType13& beam, const solid_source::VehicleSolidSource& solid, const beam18::Source* structural)
        : masses(mass), tied(tie), beams(beam), solids(solid) {if(structural) structural_beams.emplace(*structural); }
    rigid::point_mass::Source masses;
    tied_shell::TiedShellDeclaration tied;
    type13::SourceType13 beams;
    solid_source::VehicleSolidSource solids;
    std::optional<beam18::Source> structural_beams;
    Forecast forecast;
    Data data;
};
PhysicalScope PhysicalScope::PrepareImpl(const rigid::point_mass::Source& masses,
    const tied_shell::TiedShellDeclaration& tied, const type13::SourceType13& beams,
    const solid_source::VehicleSolidSource& solids, const beam18::Source* structural, Limits limits) {
    const auto forecast = PreflightImpl(masses, tied, beams, solids, structural, limits);
    auto next = std::make_shared<Storage>(masses, tied, beams, solids, structural);
    const auto& canonical = tied.canonical().data();
    auto nodes = detail::Decode<SourceId>(canonical, "node_ids");
    detail::Require(nodes.size() == canonical.canonical_nodes && !nodes.empty() && nodes.front() &&
        std::adjacent_find(nodes.begin(), nodes.end(), std::greater_equal<SourceId>()) == nodes.end(),
        "Physical source canonical node identities are not strictly ordered");
    auto& data = next->data;
    data.spotwelds = detail::ReadSpotwelds(tied.data().sources, limits);
    detail::BuildRoles(masses, beams, solids, structural, nodes, data);
    detail::BuildGroups(masses.rigid_source(), tied, nodes, data, limits);
    detail::BuildEvidence(canonical, solids, structural, nodes, data, limits);
    data.owned_payload_bytes = detail::OwnedPayload(data, limits);
    detail::Require(data.owned_payload_bytes <= forecast.result_reservation,
                    "Physical census owned payload exceeded preflight");
    next->forecast = forecast;
    return PhysicalScope(std::move(next));
}
PhysicalScope PhysicalScope::Prepare(const rigid::point_mass::Source& mass,
    const tied_shell::TiedShellDeclaration& tied,const type13::SourceType13& type13,
    const solid_source::VehicleSolidSource& solids,Limits limits) {
    return PrepareImpl(mass,tied,type13,solids,nullptr,limits);
}
PhysicalScope PhysicalScope::PrepareVehicleSupports(const rigid::point_mass::Source& mass,
    const tied_shell::TiedShellDeclaration& tied,const type13::SourceType13& type13,
    const solid_source::VehicleSolidSource& solids,const beam18::Source& beams,Limits limits) {
    return PrepareImpl(mass,tied,type13,solids,&beams,limits);
}
const beam18::Source* PhysicalScope::structural_beam_source() const noexcept {
    return storage_->structural_beams ? &*storage_->structural_beams : nullptr;
}
const Data& PhysicalScope::data() const noexcept { return storage_->data; }
const Forecast& PhysicalScope::forecast() const noexcept { return storage_->forecast; }
const rigid::point_mass::Source& PhysicalScope::point_mass_source() const noexcept { return storage_->masses; }
const tied_shell::TiedShellDeclaration& PhysicalScope::tied_source() const noexcept { return storage_->tied; }
const type13::SourceType13& PhysicalScope::type13_source() const noexcept { return storage_->beams; }
const solid_source::VehicleSolidSource& PhysicalScope::solid_source() const noexcept { return storage_->solids; }
} // namespace crash::modelio::physical_scope
