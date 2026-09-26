#include "Components.h"

namespace crash::cases::vehicle_startup::physical_model {
namespace fe = tl::fea;
struct VehiclePhysicalModel::Storage {
    Storage(const modelio::physical_domain::VehiclePhysicalDomain& d, const VehicleShellBinding& s)
        : source(d), shells(s), components(d.source(), d.domain(), detail::WeldDeclaration()) {}
    modelio::physical_domain::VehiclePhysicalDomain source;
    VehicleShellBinding shells;
    detail::Components components;
    Forecast forecast;
};
VehiclePhysicalModel VehiclePhysicalModel::Prepare(const modelio::physical_domain::VehiclePhysicalDomain& source,
                                                  const VehicleShellBinding& shells, Limits limits) {
    using detail::Require;
    const auto forecast = Preflight(source, shells, limits);
    auto next = std::make_shared<Storage>(source, shells);
    detail::PrepareComponents(source, source.domain(), shells.shells(), limits, next->components);
    next->forecast = forecast;
    return VehiclePhysicalModel(std::move(next));
}
const modelio::physical_domain::VehiclePhysicalDomain& VehiclePhysicalModel::source_domain() const noexcept { return storage_->source; }
const VehicleShellBinding& VehiclePhysicalModel::shell_source() const noexcept { return storage_->shells; }
const modelio::point_mass::VehiclePointMassSource& VehiclePhysicalModel::point_masses() const noexcept { return storage_->components.masses; }
const modelio::type25::VehicleType25Source& VehiclePhysicalModel::welds() const noexcept { return storage_->components.welds; }
const fe::type13::Model& VehiclePhysicalModel::beams() const noexcept { return storage_->components.beams; }
const fe::solids::Model& VehiclePhysicalModel::solids() const noexcept { return storage_->components.solids; }
const fe::beam18::Model* VehiclePhysicalModel::structural_beams() const noexcept {
    return storage_->components.structural_beams.prepared() ? &storage_->components.structural_beams : nullptr;
}
const fe::NodalCoefficientLedger& VehiclePhysicalModel::coefficients() const noexcept { return storage_->components.ledger; }
const fe::NodalRigidGroupModel& VehiclePhysicalModel::plain_groups() const noexcept { return storage_->components.plain; }
const fe::NodalRigidAssemblyBinding& VehiclePhysicalModel::rigid_assembly() const noexcept { return storage_->components.rigid; }
const Forecast& VehiclePhysicalModel::forecast() const noexcept { return storage_->forecast; }
} // namespace crash::cases::vehicle_startup::physical_model
