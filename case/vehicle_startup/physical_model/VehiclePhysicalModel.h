#pragma once
#include "case/vehicle_startup/VehicleShellBinding.h"
#include "modelio/physical_domain/VehiclePhysicalDomain.h"
#include "modelio/point_mass/VehiclePointMassSource.h"
#include "modelio/type25/VehicleType25Source.h"
#include "lib_src/elements/solids/Model.h"
#include "lib_src/elements/beam18/Model.h"
#include "lib_src/constraints/NodalRigidAssemblyBinding.h"

namespace crash::cases::vehicle_startup::physical_model {
struct Limits {
    // Native caps include their retained upstream backing. The shell map alone
    // retains the300 MB shell binding; these are not incremental allocations.
    // Preflight deliberately overcharges shared ownership across module caps.
    std::size_t host_bytes = std::size_t{6} << 30;
    std::size_t shell_map_bytes = 512u << 20, beam_bytes = 16u << 20, solid_bytes = 32u << 20;
    std::size_t beam_contribution_bytes = 32u << 20, ledger_bytes = 512u << 20;
    std::size_t part_bytes = 512u << 20, plain_bytes = 8u << 20, rigid_binding_bytes = 512u << 20;
    std::size_t structural_beam_bytes = 32u << 20, structural_contribution_bytes = 32u << 20;
    // Explicit V4 construction ceiling, including its full-domain backing and
    // five-family temporary references. The V1 default remains32 MiB.
    static Limits ExtendedSolids() noexcept {
        Limits result;
        result.solid_bytes = 64u << 20;
        return result;
    }
    static Limits VehicleSupports() noexcept { return ExtendedSolids(); }
};
struct Forecast {
    std::size_t shell_source = 0, physical_source = 0, producer_source = 0;
    std::size_t native_reservation = 0, packing_bytes = 0, total_bytes = 0;
};
// Complete immutable startup mechanics on the explicit retained source domain.
// Owns all typed producers, their additive ledger and native PART/plain models.
// Material/failure execution catalogs, CIN, DOFs, CUDA state and case load-path
// admission are separate; this value creates no clock or force history.
class VehiclePhysicalModel {
  public:
    static Forecast Preflight(const modelio::physical_domain::VehiclePhysicalDomain&,
                              const VehicleShellBinding&, Limits = {});
    static VehiclePhysicalModel Prepare(const modelio::physical_domain::VehiclePhysicalDomain&,
                                        const VehicleShellBinding&, Limits = {});
    const modelio::physical_domain::VehiclePhysicalDomain& source_domain() const noexcept;
    const VehicleShellBinding& shell_source() const noexcept;
    const modelio::point_mass::VehiclePointMassSource& point_masses() const noexcept;
    const modelio::type25::VehicleType25Source& welds() const noexcept;
    const tl::fea::type13::Model& beams() const noexcept;
    const tl::fea::solids::Model& solids() const noexcept;
    const tl::fea::beam18::Model* structural_beams() const noexcept;
    const tl::fea::NodalCoefficientLedger& coefficients() const noexcept;
    const tl::fea::NodalRigidGroupModel& plain_groups() const noexcept;
    const tl::fea::NodalRigidAssemblyBinding& rigid_assembly() const noexcept;
    const Forecast& forecast() const noexcept;
    bool SharesStorage(const VehiclePhysicalModel& other) const noexcept {
        return storage_ == other.storage_;
    }
  private:
    struct Storage;
    explicit VehiclePhysicalModel(std::shared_ptr<const Storage> value) : storage_(std::move(value)) {}
    std::shared_ptr<const Storage> storage_;
};
} // namespace crash::cases::vehicle_startup::physical_model
