#include "SourceAdmission.h"
#include "case/vehicle_self_contact/native/InitializerControlsSource.h"
#include "output/ArtifactIO.h"
#include <algorithm>
namespace crash::cases::vehicle_native_contact::detail {
namespace c = vehicle_self_contact::native::initial_controls;
namespace {
std::size_t Add(std::size_t a, std::size_t b) {
    output::Require(b <= SIZE_MAX - a, "Native vehicle source budget overflows");
    return a + b;
}
std::size_t Extra(std::size_t whole, std::size_t shared) {
    output::Require(whole >= shared, "Native vehicle owning source partition is inconsistent");
    return whole - shared;
}
}
SourceAdmission AdmitSources(const SourceInputs& in, std::size_t cap) {
    using output::Require;
    Require(cap && cap <= vehicle_wall::native::EnvelopeOwnerLimits{}.host_bytes,
            "Invalid complete native vehicle source cap");
    const auto& owner = in.owner;
    const auto& physical = owner.physical();
    const auto& mechanical = owner.execution_source().mechanical();
    const auto& main = in.self.main_source();
    const auto* embedding = in.self.embedding();
    Require(physical.prepared() && physical.domain() && embedding &&
                physical.domain()->SharesStorage(in.self.domain()) &&
                physical.domain()->SharesStorage(in.wall.domain()) &&
                physical.domain()->SharesStorage(embedding->domain()) &&
                mechanical.wall().SharesStorage(in.wall.wall()) &&
                in.controls.wall() && in.controls.wall()->SharesStorage(in.wall.wall()) &&
                main.SharesStorage(in.wall.vehicle_source()) && main.SharesStorage(in.controls.main()),
            "Native self/wall controls do not retain the actual common owner source");
    const auto& original = main.mixed().initial().context().pre_correction().physical();
    Require(embedding->original().Matches(original.source_domain().domain()) &&
                in.wall.embedding().original().Matches(embedding->original()) &&
                &original.shell_source().references().source().canonical().data() ==
                    &mechanical.vehicle_references().source().canonical().data(),
            "Native contact prefix/canonical source identity differs from the actual physical owner");
    SourceAdmission out;
    out.nodes = physical.domain()->node_count();
    Require(out.nodes && in.self.snapshot().node_count == out.nodes &&
                in.wall.starter().node_count == out.nodes && in.wall.nodes().size() == out.nodes &&
                in.self.snapshot().main_count == main.coefficients().size() &&
                in.wall.provenance().all_retained_vehicle_additional_nodes &&
                in.wall.secondary_nodes().size() == out.nodes &&
                in.wall.gap_report().completed &&
                in.wall.gap_report().status == tlfea::contact::radioss_type25::source_gaps::Status::Ok,
            "Native interface complete domain/topology/gap source extent is inconsistent");
    Require(in.controls.self_interface_id() && in.controls.wall_interface_id() &&
                in.controls.self_interface_id() != in.controls.wall_interface_id() &&
                in.controls.wall_interface_id() == in.wall.provenance().interface_id,
            "Native contact source identities are missing or repeated");
    std::size_t count = 0;
    std::uint32_t previous = 0;
    bool saw_self = false, saw_wall = false;
    for (const auto& row : in.controls.interfaces()) {
        Require(row.native_id && row.native_storage_ordinal > previous,
                "Complete native interface table is not in authenticated storage order");
        previous = row.native_storage_ordinal;
        if (row.kind != c::InterfaceKind::Type25) continue;
        Require(count < out.interfaces.size(), "Declared TYPE25 interface count exceeds this two-interface case");
        auto& ordered = out.interfaces[count++];
        ordered.native_id = row.native_id;
        ordered.native_storage_ordinal = row.native_storage_ordinal;
        if (row.native_id == in.controls.self_interface_id()) {
            Require(!saw_self && row.origin == c::InterfaceOrigin::OriginalDefinition,
                    "Original self interface is duplicated or declared as a replacement");
            saw_self = true;
            ordered.role = vehicle_dynamics::native_contact::Role::Self;
        } else {
            Require(!saw_wall && row.native_id == in.controls.wall_interface_id() &&
                        row.origin == c::InterfaceOrigin::DeclaredAdditionalInterface,
                    "Finite wall does not match the complete declared interface table");
            saw_wall = true;
            ordered.role = vehicle_dynamics::native_contact::Role::MeshWall;
        }
    }
    Require(count == 2 && saw_self && saw_wall, "Complete self and finite-wall interface pair is required");
    const auto& s = in.self.forecast();
    const auto& w = in.wall.forecast();
    const auto& controls = in.controls.forecast();
    const auto main_retained = main.forecast().retained_bytes;
    Require(s.upstream_retained == main_retained && controls.upstream_retained == main_retained,
            "Shared immutable main-source retained partition differs");
    out.owner_retained = owner.retained_host_upper_bound(cap);
    out.self_retained = s.retained_bytes;
    out.wall_incremental = w.own_retained;
    // Both discounts use exact owning SharesStorage proofs above and published
    // component partitions. The full owner and contact graphs stay separate.
    out.controls_incremental = Extra(controls.retained_bytes,
        Add(controls.upstream_retained, controls.wall_retained_bound));
    const auto self_extra = Extra(out.self_retained, main_retained);
    const auto contact = Add(Add(out.self_retained, out.wall_incremental), out.controls_incremental);
    out.retained_bytes = Add(out.owner_retained, contact);
    out.construction_peak = std::max({out.retained_bytes,
        Add(owner.forecast().peak_bytes, contact),
        Add(Add(Add(s.peak_bytes, out.owner_retained), out.wall_incremental), out.controls_incremental),
        Add(Add(w.complete_construction_bound, self_extra), out.controls_incremental),
        Add(Add(Add(controls.peak_bytes, out.owner_retained), self_extra), out.wall_incremental)});
    Require(out.construction_peak <= cap, "Complete native vehicle source coexistence exceeds the host cap");
    return out;
}
} // namespace crash::cases::vehicle_native_contact::detail
