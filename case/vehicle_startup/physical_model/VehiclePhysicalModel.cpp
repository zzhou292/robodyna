#include "Internal.h"

namespace crash::cases::vehicle_startup::physical_model {
namespace fe = tl::fea;
struct VehiclePhysicalModel::Storage {
    Storage(const modelio::physical_domain::VehiclePhysicalDomain& d, const VehicleShellBinding& s)
        : source(d), shells(s), masses(modelio::point_mass::VehiclePointMassSource::Prepare(d.source(), d.domain())),
          welds(modelio::type25::VehicleType25Source::Prepare(d.source(), d.domain(), detail::WeldDeclaration())) {}
    modelio::physical_domain::VehiclePhysicalDomain source;
    VehicleShellBinding shells;
    modelio::point_mass::VehiclePointMassSource masses;
    modelio::type25::VehicleType25Source welds;
    fe::ShellNodeMap shell_map;
    fe::type13::Model beams;
    fe::solids::Model solids;
    fe::Type13NodeContributions beam_coefficients;
    fe::beam18::Model structural_beams;
    fe::Beam18NodeContributions structural_coefficients;
    fe::NodalCoefficientLedger ledger;
    fe::rigid::NodalRigidPartAssemblyModel parts;
    fe::NodalRigidGroupModel plain;
    fe::NodalRigidAssemblyBinding rigid;
    Forecast forecast;
};
VehiclePhysicalModel VehiclePhysicalModel::Prepare(const modelio::physical_domain::VehiclePhysicalDomain& source,
                                                  const VehicleShellBinding& shells, Limits limits) {
    using detail::Require;
    const auto forecast = Preflight(source, shells, limits);
    auto next = std::make_shared<Storage>(source, shells);
    const auto& domain = source.domain();
    auto shell_limits = fe::ShellNodeMapLimits::Vehicle(); shell_limits.max_host_bytes = limits.shell_map_bytes;
    const auto shell_report = next->shell_map.Initialize(shells.shells(), domain, shell_limits);
    Require(bool(shell_report), shell_report.message);
    detail::PrepareBeams(source.source().type13_source(), domain, limits.beam_bytes, next->beams);
    detail::PrepareSolids(source.source().solid_source(), domain, limits.solid_bytes, next->solids);
    fe::Type13ContributionLimits beam_limits; beam_limits.max_host_bytes = limits.beam_contribution_bytes;
    const auto beam_report = next->beam_coefficients.Initialize(next->beams, domain, beam_limits);
    Require(bool(beam_report), beam_report.message);
    if (const auto* structural = source.source().structural_beam_source()) {
        detail::PrepareStructuralBeams(*structural, domain, limits.structural_beam_bytes, next->structural_beams);
        fe::Beam18ContributionLimits structural_limits;
        structural_limits.max_host_bytes = limits.structural_contribution_bytes;
        const auto report = next->structural_coefficients.Initialize(next->structural_beams, structural_limits);
        Require(bool(report), report.message);
    }
    auto ledger_limits = fe::CoefficientLimits::Vehicle(); ledger_limits.max_host_bytes = limits.ledger_bytes;
    const fe::NodalCoefficientSourcesWithSolids input{{&next->shell_map, &next->welds.model(),
        &next->beam_coefficients}, &next->masses.contributions(), next->solids.contributions()};
    const auto ledger_report = next->structural_beams.prepared()
        ? next->ledger.InitializeWithBeams({input, &next->structural_coefficients}, ledger_limits)
        : next->solids.profile() == fe::solids::ModelProfile::ExtendedLaw44Law90
        ? next->ledger.InitializeWithExtendedSolids(input, ledger_limits)
        : next->ledger.InitializeWithSolids(input, ledger_limits);
    Require(bool(ledger_report), ledger_report.message);
    Require(!next->ledger.scope().uncovered_nodes, "Complete physical domain contains an uncovered coefficient node");
    fe::rigid::PartAssemblyLimits part_limits; part_limits.max_host_bytes = limits.part_bytes;
    const auto part_report = next->parts.Initialize(source.topology(), next->ledger, {1000, .001}, part_limits);
    if (!part_report) throw std::runtime_error("Original PART aggregate " + std::to_string(part_report.part) +
                                             " rejected: " + part_report.message);
    detail::PreparePlain(source, next->ledger, limits.plain_bytes, next->plain);
    fe::RigidBindingLimits rigid_limits; rigid_limits.max_host_bytes = limits.rigid_binding_bytes;
    const auto rigid_report = next->rigid.Initialize(next->parts, &next->plain, rigid_limits);
    Require(bool(rigid_report), rigid_report.message);
    next->forecast = forecast;
    return VehiclePhysicalModel(std::move(next));
}
const modelio::physical_domain::VehiclePhysicalDomain& VehiclePhysicalModel::source_domain() const noexcept { return storage_->source; }
const VehicleShellBinding& VehiclePhysicalModel::shell_source() const noexcept { return storage_->shells; }
const modelio::point_mass::VehiclePointMassSource& VehiclePhysicalModel::point_masses() const noexcept { return storage_->masses; }
const modelio::type25::VehicleType25Source& VehiclePhysicalModel::welds() const noexcept { return storage_->welds; }
const fe::type13::Model& VehiclePhysicalModel::beams() const noexcept { return storage_->beams; }
const fe::solids::Model& VehiclePhysicalModel::solids() const noexcept { return storage_->solids; }
const fe::beam18::Model* VehiclePhysicalModel::structural_beams() const noexcept {
    return storage_->structural_beams.prepared() ? &storage_->structural_beams : nullptr;
}
const fe::NodalCoefficientLedger& VehiclePhysicalModel::coefficients() const noexcept { return storage_->ledger; }
const fe::NodalRigidGroupModel& VehiclePhysicalModel::plain_groups() const noexcept { return storage_->plain; }
const fe::NodalRigidAssemblyBinding& VehiclePhysicalModel::rigid_assembly() const noexcept { return storage_->rigid; }
const Forecast& VehiclePhysicalModel::forecast() const noexcept { return storage_->forecast; }
} // namespace crash::cases::vehicle_startup::physical_model
