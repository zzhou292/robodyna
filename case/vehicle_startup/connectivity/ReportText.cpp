#include "ReportText.h"
#include "output/ArtifactIO.h"
#include "chrono_thirdparty/rapidjson/writer.h"

namespace crash::cases::vehicle_startup::connectivity::detail {
namespace {
struct Sink {
    using Ch = char;
    std::string* bytes;
    std::size_t cap, count = 0;
    void Put(char c) {
        output::Require(count < cap, "Complete connectivity report exceeds byte cap");
        ++count;
        if (bytes) bytes->push_back(c);
    }
    void Flush() {}
};
void Render(Sink& sink, const Data& data, tl::util::ConstView<tl::fea::NodalDomainNode> nodes,
            const Forecast& forecast, const ReportIdentity& identity) {
    bool extended = identity.has_joints;
    std::size_t joint_count = 0;
    for (std::size_t kind = LegacyKindCount; kind < KindCount; ++kind) {
        extended |= data.counts.by_kind[kind] != 0;
        if (IsJoint(static_cast<Kind>(kind))) joint_count += data.counts.by_kind[kind];
    }
    const auto kinds = extended ? KindCount : LegacyKindCount;
    rapidjson::Writer<Sink> writer(sink);
    const auto number = [&](const char* key, std::size_t value) { writer.Key(key); writer.Uint64(value); };
    const auto text = [&](const char* key, const std::string& value) {
        writer.Key(key); writer.String(value.data(),value.size());
    };
    writer.StartObject();
    text("schema",extended ? "robo_dyna.original_physical_connectivity.v2" :
        "robo_dyna.original_physical_connectivity.v1");
    text("scope","Static weak incidence/potential transfer; no current activity, DOF rank or complete load-path admission");
    text("archive_sha256",identity.archive_sha256); text("member_sha256",identity.member_sha256);
    text("canonical_sha256",identity.canonical_sha256); text("tire_policy",identity.tire_policy);
    writer.Key("pending"); writer.StartArray();
    if (!identity.has_joints) writer.String("Required original joint operators; no joint edges are admitted here");
    else if (identity.joint_boundaries) writer.String("Original joint rows at explicitly omitted assembly boundaries");
    writer.String("All omitted assemblies and auxiliary/test-setup source dispositions");
    writer.String("Current CIN master activity/release and current geometry");
    writer.EndArray();
    writer.Key("kind_codes"); writer.StartArray();
    for (std::size_t kind = 0; kind < kinds; ++kind) writer.String(Name(static_cast<Kind>(kind)));
    writer.EndArray();
    text("role_codes","0 constitutive; 1 original rigid skin; 2 potential constraint transfer; 3 point-mass attribution");
    text("node_role_bits","1 element incidence; 2 rigid skin; 4 constraint support; 8 point-mass attribution");
    text("node_columns","original_nid,element_component_min_nid,transfer_component_min_nid,role_bits");
    text("relation_columns",extended ?
        "kind,role,source_id,part_id,source_row,ordered_domain_slots[,rigid_skin_root_index or joint_released_dof_mask]" :
        "kind,role,source_id,part_id,source_row,ordered_domain_slots[,rigid_skin_root_index]");
    text("relation_identity","structural EID, TYPE25 WID and ELEMENT_MASS use distinct namespaces; CIN source_id is declared master EID and source_row is retained CIN row");
    text("beam_support","N1/N2 only; release declarations retained by source model, N3 excluded");
    text("cin_support","secondary then four ordered masters, including repeated native triangle slot; directional support, not scalar interpolation");
    if (extended) {
        text("partition_semantics","Constraint support is made undirected only for weak potential-transfer labels; no directed reachability or constrained-DOF rank is claimed");
        text("joint_support","Only N1/N2 actual rigid-member endpoints; axis-only N3 excluded. Source row indexes the original joint declarations. Both complete rigid bodies remain separate constraint relations");
        text("joint_dof_bits","Tx,Ty,Tz,Rx,Ry,Rz = bits0..5 in the native current joint frame; a set bit is released, not a world-axis constraint");
        writer.Key("joint_released_dof_masks"); writer.StartObject();
        for (const auto kind : {Kind::SphericalJoint,Kind::RevoluteJoint,Kind::CylindricalJoint})
            number(Name(kind),ReleasedDofs(kind));
        writer.EndObject();
        writer.Key("prepared_joint_model"); writer.Bool(identity.has_joints);
        number("joint_source_instance_id",identity.joint_source_instance);
        number("omitted_joint_boundary_rows",identity.joint_boundaries);
    }
    writer.Key("counts"); writer.StartObject();
    const auto& c = data.counts;
    number("nodes",c.nodes); number("relations",c.relations); number("ordered_slots",c.slots);
    number("element_components",c.element_components); number("potential_transfer_components",c.transfer_components);
    number("rigid_skin_parents",c.rigid_skin_parents); number("mass_without_element_incidence",c.mass_without_element_incidence);
    number("joint_edges",joint_count);
    writer.Key("by_kind"); writer.StartArray();
    for (std::size_t kind = 0; kind < kinds; ++kind) writer.Uint64(c.by_kind[kind]);
    writer.EndArray();
    writer.EndObject();
    writer.Key("budget"); writer.StartObject();
    number("retained_source_bound",forecast.retained_source_bound); number("owned_graph_bytes",forecast.owned_bytes);
    number("component_scratch_bytes",forecast.component_scratch_bytes); number("report_reservation",forecast.report_reservation);
    number("extra_phase_max_bytes",forecast.extra_bytes); number("inclusive_total_bytes",forecast.total_bytes);
    writer.EndObject();
    writer.Key("nodes"); writer.StartArray();
    for (std::size_t n = 0; n < nodes.size(); ++n) {
        writer.StartArray(); writer.Uint64(nodes[n].source_id); writer.Uint64(data.element_label[n]);
        writer.Uint64(data.transfer_label[n]); writer.Uint(data.node_roles[n]); writer.EndArray();
    }
    writer.EndArray();
    writer.Key("relations"); writer.StartArray();
    for (const auto& row : data.relations) {
        writer.StartArray(); writer.Uint(static_cast<unsigned>(row.kind)); writer.Uint(static_cast<unsigned>(row.role));
        writer.Uint64(row.source_id); writer.Uint64(row.part_id); writer.Uint(row.source_row);
        writer.StartArray();
        for (std::size_t slot = row.slot_offset; slot < row.slot_offset+row.slot_count; ++slot)
            writer.Uint(data.slots[slot]);
        writer.EndArray();
        if (row.role == Role::RigidSkin) writer.Uint(row.rigid_root_index);
        else if (IsJoint(row.kind)) writer.Uint(ReleasedDofs(row.kind));
        writer.EndArray();
    }
    writer.EndArray();
    writer.EndObject();
}
}
std::string RenderReport(const Data& data, tl::util::ConstView<tl::fea::NodalDomainNode> nodes,
                         const Forecast& forecast, const ReportIdentity& identity, std::size_t cap) {
    output::Require(cap && cap <= Limits{}.report_bytes && data.element_label.size() == nodes.size() &&
        data.transfer_label.size() == nodes.size() && data.node_roles.size() == nodes.size(),
        "Connectivity report bounds or complete node layout differs");
    std::size_t joint_count = 0;
    for (const auto kind : {Kind::SphericalJoint,Kind::RevoluteJoint,Kind::CylindricalJoint})
        joint_count += data.counts.by_kind[static_cast<std::size_t>(kind)];
    output::Require(identity.has_joints == (joint_count != 0) &&
        (identity.has_joints ? identity.joint_source_instance != 0 :
            identity.joint_source_instance == 0 && identity.joint_boundaries == 0),
        "Connectivity report joint model presence/identity differs");
    // Count before allocation. The second pass uses exactly the same immutable
    // data and one pre-reserved string, avoiding JSON DOM and growth copies.
    Sink counted{nullptr,cap};
    Render(counted,data,nodes,forecast,identity);
    std::string bytes;
    bytes.reserve(counted.count);
    output::Require(bytes.capacity() <= cap, "Connectivity report capacity exceeds its reservation");
    Sink written{&bytes,cap};
    Render(written,data,nodes,forecast,identity);
    output::Require(written.count == counted.count, "Connectivity report changed during publication");
    return bytes;
}
} // namespace crash::cases::vehicle_startup::connectivity::detail
