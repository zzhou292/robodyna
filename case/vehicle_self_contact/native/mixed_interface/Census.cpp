#include "Internal.h"
#include "modelio/vehicle_sections/VehicleSectionResolution.h"
#include <algorithm>
#include <cmath>
namespace crash::cases::vehicle_self_contact::native::mixed_interface::detail {
AdmissionCensus Census(const Initial& input) {
    AdmissionCensus result;
    const auto& physical = input.context().pre_correction().physical();
    const auto& ledger = physical.coefficients();
    const auto& domain = physical.source_domain().domain();
    const auto& rigid = physical.rigid_assembly();
    const auto& candidates = physical.source_domain().source().tied_source().data().slave_nodes;
    output::Require(ledger.prepared() && ledger.domain() && ledger.domain()->SharesStorage(domain) &&
        ledger.nodes().size() == domain.node_count() && domain.node_count() == input.geometry().nodes.size(),
        "Mixed census requires the identical complete physical ledger/domain");
    // Retained declaration contract: one row per canonical slave node, sorted
    // by original NID in GeometryCensus.cpp. Check it before binary lookup;
    // this is a cheap retained-array check, not a parser or source reconstruction.
    for (std::size_t i = 1; i < candidates.size(); ++i)
        output::Require(candidates[i-1].id < candidates[i].id,
            "Tied source candidate IDs are not sorted unique");
    result.nodes = domain.node_count();
    result.actual_rigid_binding_available = rigid.prepared();
    if (rigid.prepared()) {
        output::Require(rigid.coefficients() && rigid.coefficients()->Matches(ledger),
            "Mixed census rigid binding has a foreign physical ledger");
        result.rigid_groups = rigid.groups().size();
        result.rigid_members = rigid.members().size();
    }
    for (std::size_t n = 0; n < result.nodes; ++n) {
        const auto id = domain.nodes()[n].source_id;
        output::Require(input.geometry().nodes[n].source_id == id, "Mixed census source node order differs");
        const auto at = std::lower_bound(candidates.begin(), candidates.end(), id,
            [](const auto& candidate, auto value) { return candidate.id < value; });
        const bool tied_candidate = at != candidates.end() && at->id == id;
        result.tied_source_candidates += tied_candidate;
        const bool rigid_member = rigid.prepared() && rigid.FindMember(n);
        const auto mass = ledger.nodes()[n].coefficients.mass;
        if (!std::isfinite(mass)) ++result.nonfinite_mass;
        else if (mass > 0) ++result.positive_mass;
        else if (mass == 0) ++result.zero_mass;
        else ++result.negative_mass;
        if (!std::isfinite(mass) || mass <= 0) {
            const bool nonpositive = std::isfinite(mass) && mass <= 0;
            result.nonpositive_rigid_members += nonpositive && rigid_member;
            result.nonpositive_tied_candidates += nonpositive && tied_candidate;
            if (result.mass_example_count < result.mass_examples.size())
                result.mass_examples[result.mass_example_count++] = {id, std::uint32_t(n), mass, rigid_member, tied_candidate};
        }
    }
    const auto& references = physical.shell_source().references();
    const auto* resolved = references.resolution();
    output::Require(resolved && resolved->parents().size() == references.rows().size(),
        "Mixed census lacks complete source failure declarations");
    for (std::size_t i = 0; i < references.rows().size(); ++i) {
        const auto& row = references.rows()[i];
        const auto* declared = resolved->native_parent(i);
        output::Require(declared && row.part_index == resolved->parents()[i].part_index &&
            row.element_id == resolved->parents()[i].source_parent_id &&
            row.element_id == declared->source.source_parent_id && row.part_id == declared->source.source_part_id &&
            row.material_id == declared->source.material_id && row.section_id == declared->source.section_id,
            "Mixed census source failure parent association differs");
        const auto policy = static_cast<std::size_t>(declared->policy);
        using Family = vehicle_startup::ReferenceFamily;
        const auto family = row.family == Family::Qeph ? 0u : row.family == Family::T3 ? 1u : 2u;
        output::Require((row.family == Family::Qeph || row.family == Family::T3 || row.family == Family::Qbat) && policy < 3,
            "Mixed census source family/failure policy is unavailable");
        ++result.shell_failure_policies[family][policy];
    }
    return result;
}
}
