#include "Components.h"

namespace crash::cases::vehicle_startup::physical_model::detail {
void PrepareComponents(const modelio::physical_domain::VehiclePhysicalDomain& source,
    const fe::NodalNodeDomain& domain, const fe::ShellBatchBinding& shells,
    Limits limits, Components& next) {
    auto shell_limits = fe::ShellNodeMapLimits::Vehicle(); shell_limits.max_host_bytes = limits.shell_map_bytes;
    const auto shell_report = next.shell_map.Initialize(shells, domain, shell_limits);
    Require(bool(shell_report), shell_report.message);
    detail::PrepareBeams(source.source().type13_source(), domain, limits.beam_bytes, next.beams);
    detail::PrepareSolids(source.source().solid_source(), domain, limits.solid_bytes, next.solids);
    fe::Type13ContributionLimits beam_limits; beam_limits.max_host_bytes = limits.beam_contribution_bytes;
    const auto beam_report = next.beam_coefficients.Initialize(next.beams, domain, beam_limits);
    Require(bool(beam_report), beam_report.message);
    if (const auto* structural = source.source().structural_beam_source()) {
        detail::PrepareStructuralBeams(*structural, domain, limits.structural_beam_bytes, next.structural_beams);
        fe::Beam18ContributionLimits structural_limits;
        structural_limits.max_host_bytes = limits.structural_contribution_bytes;
        const auto report = next.structural_coefficients.Initialize(next.structural_beams, structural_limits);
        Require(bool(report), report.message);
    }
    auto ledger_limits = fe::CoefficientLimits::Vehicle(); ledger_limits.max_host_bytes = limits.ledger_bytes;
    const fe::NodalCoefficientSourcesWithSolids input{{&next.shell_map, &next.welds.model(),
        &next.beam_coefficients}, &next.masses.contributions(), next.solids.contributions()};
    const auto ledger_report = next.structural_beams.prepared()
        ? next.ledger.InitializeWithBeams({input, &next.structural_coefficients}, ledger_limits)
        : next.solids.profile() == fe::solids::ModelProfile::ExtendedLaw44Law90
        ? next.ledger.InitializeWithExtendedSolids(input, ledger_limits)
        : next.ledger.InitializeWithSolids(input, ledger_limits);
    Require(bool(ledger_report), ledger_report.message);
    Require(!next.ledger.scope().uncovered_nodes, "Complete physical domain contains an uncovered coefficient node");
    fe::rigid::PartAssemblyLimits part_limits; part_limits.max_host_bytes = limits.part_bytes;
    const auto part_report = next.parts.Initialize(source.topology(), next.ledger, {1000, .001}, part_limits);
    if (!part_report) throw std::runtime_error("Original PART aggregate " + std::to_string(part_report.part) +
                                             " rejected: " + part_report.message);
    detail::PreparePlain(source, domain, next.ledger, limits.plain_bytes, next.plain);
    fe::RigidBindingLimits rigid_limits; rigid_limits.max_host_bytes = limits.rigid_binding_bytes;
    const auto rigid_report = next.rigid.Initialize(next.parts, &next.plain, rigid_limits);
    Require(bool(rigid_report), rigid_report.message);
}
} // namespace crash::cases::vehicle_startup::physical_model::detail
