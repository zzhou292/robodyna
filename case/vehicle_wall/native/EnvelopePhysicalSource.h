#pragma once
#include "WallSource.h"
#include "case/vehicle_startup/physical_model/VehiclePhysicalModel.h"

namespace crash::cases::vehicle_wall::native {
struct EnvelopePhysicalLimits {
    std::size_t host_bytes = std::size_t{8} << 30;
    modelio::physical_scope::DomainEmbeddingLimits embedding;
    vehicle_startup::VehicleShellBindingLimits shells;
    vehicle_startup::physical_model::Limits components = vehicle_startup::physical_model::Limits::VehicleSupports();
};
struct EnvelopePhysicalForecast {
    std::size_t wall_source = 0, embedding = 0, shell_binding = 0;
    std::size_t contributor_sources = 0, native_components = 0, component_packing = 0;
    std::size_t fixed_bytes = 0, peak_bytes = 0;
    std::size_t embedding_prior_peak = 0, point_mass_current = 0, point_mass_chain_peak = 0;
    std::size_t type25_current = 0, type25_chain_peak = 0;
};
struct EnvironmentParent {
    std::uint64_t element_id = 0, part_id = 0, material_id = 0, section_id = 0;
    std::size_t qeph_index = 0, catalog_append_ordinal = 0;
    std::array<std::size_t,4> shell_nodes{}, domain_nodes{};
};
// Complete source coefficients and rigid models on the genuine combined domain.
// Vehicle references remain original provenance; the added parent has explicit
// environment origin. This is not a VehiclePhysicalModel and creates no owner,
// execution/failure catalog, CIN witness, contact arrays or runtime admission.
class EnvelopePhysicalSource {
  public:
    static EnvelopePhysicalForecast Preflight(const WallSource&,
        const vehicle_startup::VehicleShellReferences&, EnvelopePhysicalLimits = {});
    static EnvelopePhysicalSource Prepare(const WallSource&,
        const vehicle_startup::VehicleShellReferences&, EnvelopePhysicalLimits = {});
    const WallSource& wall() const noexcept;
    const modelio::physical_scope::DomainEmbedding& embedding() const noexcept;
    const tl::fea::NodalNodeDomain& domain() const noexcept;
    const vehicle_startup::VehicleShellReferences& vehicle_references() const noexcept;
    const tl::fea::ShellBatchBinding& shells() const noexcept;
    const EnvironmentParent& environment_parent() const noexcept;
    const modelio::point_mass::VehiclePointMassSource& point_masses() const noexcept;
    const modelio::type25::VehicleType25Source& welds() const noexcept;
    const tl::fea::type13::Model& beams() const noexcept;
    const tl::fea::solids::Model& solids() const noexcept;
    const tl::fea::beam18::Model& structural_beams() const noexcept;
    const tl::fea::NodalCoefficientLedger& coefficients() const noexcept;
    const tl::fea::NodalRigidGroupModel& plain_groups() const noexcept;
    const tl::fea::NodalRigidAssemblyBinding& rigid_assembly() const noexcept;
    const EnvelopePhysicalForecast& forecast() const noexcept;
  private:
    struct Data;
    explicit EnvelopePhysicalSource(std::shared_ptr<const Data> data) : data_(std::move(data)) {}
    std::shared_ptr<const Data> data_;
};
} // namespace crash::cases::vehicle_wall::native
