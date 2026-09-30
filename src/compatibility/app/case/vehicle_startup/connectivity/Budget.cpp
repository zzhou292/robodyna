#include "Internal.h"

namespace crash::cases::vehicle_startup::connectivity::detail {
namespace {
std::size_t Sum(std::initializer_list<std::size_t> values) {
    std::size_t result = 0;
    for (auto value : values) {
        Require(value <= SIZE_MAX-result, "Connectivity extent overflows");
        result += value;
    }
    return result;
}
}
Counts Extents(const VehiclePhysicalAttachments& source, const joints::VehicleJointModel* joints) {
    const auto& physical = source.physical();
    const auto& domain = physical.source_domain().domain();
    const auto& ledger = physical.coefficients();
    const auto& shells = physical.shell_source().shells();
    const auto& solid = physical.solids();
    const auto& rigid = physical.rigid_assembly();
    const auto& cin = source.attachments().model();
    const auto* structural = physical.structural_beams();
    Require(ledger.prepared() && ledger.domain()->SharesStorage(domain) &&
        ledger.shells()->Matches(shells,domain) && ledger.type13() &&
        ledger.type13()->Matches(physical.beams(),domain) && ledger.type25() &&
        ledger.type25()->Matches(physical.welds().model()) && ledger.solids() &&
        ledger.solids()->Matches(*solid.contributions()) && ledger.element_mass() &&
        ledger.element_mass()->Matches(physical.point_masses().contributions()) &&
        rigid.coefficients()->Matches(ledger) && cin.domain()->SharesStorage(domain) &&
        source.witnesses().runtime_mappable(), "Connectivity physical/CIN source authorities differ");
    Require(bool(structural) == bool(ledger.beam18()), "Connectivity structural beam source presence differs");
    if (structural)
        Require(structural->prepared() && structural->domain()->SharesStorage(domain) &&
            ledger.beam18()->Matches(*structural,domain), "Connectivity structural beam authority differs");
    if (joints) CheckJoints(source,*joints);
    Counts count;
    count.nodes = domain.node_count();
    auto& family = count.by_kind;
    family[static_cast<std::size_t>(Kind::Qeph)] = shells.qeph_count();
    family[static_cast<std::size_t>(Kind::T3)] = shells.t3_count();
    family[static_cast<std::size_t>(Kind::Qbat)] = shells.qbat_count();
    family[static_cast<std::size_t>(Kind::Solid18)] = solid.solid18().size();
    family[static_cast<std::size_t>(Kind::Solid24)] = solid.solid24().size();
    family[static_cast<std::size_t>(Kind::Solid6z)] = solid.solid6z().size();
    family[static_cast<std::size_t>(Kind::Solid18Law44)] = solid.solid18_law44().size();
    family[static_cast<std::size_t>(Kind::Solid18Law90)] = solid.solid18_law90().size();
    family[static_cast<std::size_t>(Kind::Beam18)] = structural ? structural->parents().size() : 0;
    family[static_cast<std::size_t>(Kind::Type13)] = physical.beams().connection_count();
    family[static_cast<std::size_t>(Kind::Type25)] = physical.welds().model().connection_count();
    family[static_cast<std::size_t>(Kind::PartRoot)] = rigid.parts()->topology()->root_count();
    Require(rigid.groups().size() >= family[static_cast<std::size_t>(Kind::PartRoot)],
            "Connectivity rigid root/group extent differs");
    family[static_cast<std::size_t>(Kind::PlainGroup)] =
        rigid.groups().size()-family[static_cast<std::size_t>(Kind::PartRoot)];
    family[static_cast<std::size_t>(Kind::Cin)] = cin.rows().count;
    family[static_cast<std::size_t>(Kind::PointMass)] = ledger.element_mass()->records().size();
    if (joints) {
        for (const auto& joint : joints->model().joints()) {
            using Native = tl::fea::type45::Kind;
            const auto kind = joint.property.kind == Native::Spherical ? Kind::SphericalJoint :
                joint.property.kind == Native::Revolute ? Kind::RevoluteJoint : Kind::CylindricalJoint;
            ++family[static_cast<std::size_t>(kind)]; // CheckJoints authenticated the closed kinds first.
        }
    }
    for (auto value : family) count.relations = Sum({count.relations,value});
    for (std::size_t kind = 0; kind < KindCount; ++kind) {
        const auto width = Width(static_cast<Kind>(kind));
        Require(!width || family[kind] <= SIZE_MAX/width, "Connectivity slot extent overflows");
        count.slots = Sum({count.slots,width*family[kind]});
    }
    count.slots = Sum({count.slots,rigid.members().size()});
    return count;
}
Forecast Budget(const VehiclePhysicalAttachments& source, const Counts& count,
                Limits limits, std::size_t fixed, const joints::VehicleJointModel* joints) {
    const Limits hard;
    Require(limits.nodes && limits.nodes <= hard.nodes && limits.relations &&
        limits.relations <= hard.relations && limits.slots && limits.slots <= hard.slots &&
        limits.extra_bytes && limits.extra_bytes <= hard.extra_bytes &&
        limits.report_bytes && limits.report_bytes <= hard.report_bytes &&
        limits.total_bytes && limits.total_bytes <= hard.total_bytes,
        "Connectivity limits exceed the explicit profile");
    Require(count.nodes && count.nodes <= limits.nodes && count.relations <= limits.relations &&
        count.slots <= limits.slots, "Connectivity complete source counts exceed caps");
    Forecast next;
    next.extents = count;
    next.retained_source_bound = source.forecast().total_bytes;
    // Both inputs are already prepared. CheckJoints proved shared physical
    // backing; charge only the immutable joint payload retained in addition.
    if (joints) next.retained_source_bound = Sum({next.retained_source_bound,joints->additional_owned_payload_bytes()});
    tl::util::BoundedArenaLayout owned(limits.extra_bytes), scratch(limits.extra_bytes);
    tl::util::ArenaRegion ignored;
    Require(owned.Append<std::byte>(fixed,ignored) &&
        owned.Append<std::uint64_t>(2*count.nodes,ignored) &&
        owned.Append<std::uint8_t>(count.nodes,ignored) &&
        owned.Append<Relation>(count.relations,ignored) &&
        owned.Append<std::uint32_t>(count.slots,ignored) &&
        scratch.Append<std::uint32_t>(count.nodes,ignored),
        "Connectivity graph payload or component scratch exceeds cap");
    next.owned_bytes = owned.bytes();
    next.component_scratch_bytes = scratch.bytes();
    next.report_reservation = limits.report_bytes;
    // Report construction follows component scratch retirement. The complete
    // source bound already includes its shared physical/CIN/canonical handles.
    next.extra_bytes = Sum({next.owned_bytes,std::max(next.component_scratch_bytes,next.report_reservation)});
    next.total_bytes = Sum({next.retained_source_bound,next.extra_bytes});
    Require(next.extra_bytes <= limits.extra_bytes && next.total_bytes <= limits.total_bytes,
            "Connectivity simultaneous source/graph/report reservation exceeds cap");
    return next;
}
} // namespace crash::cases::vehicle_startup::connectivity::detail
