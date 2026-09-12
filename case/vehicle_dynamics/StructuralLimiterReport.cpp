#include "StructuralLimiterReport.h"
#include "VehiclePhysicalDynamics.h"
#include "limiter/SourceRows.h"
#include "limiter/Identity.h"
#include "chrono_thirdparty/rapidjson/writer.h"

namespace crash::cases::vehicle_dynamics {
namespace {
struct Sink {
    using Ch = char;
    std::string* bytes = nullptr;
    std::size_t cap = 0, count = 0;
    void Put(char c) {
        output::Require(count < cap,"Complete structural limiter report exceeds byte cap");
        ++count;
        if(bytes) bytes->push_back(c);
    }
    void Flush() {}
};
void Render(Sink& sink, const vehicle_wall::VehicleWallSetup& setup,
            const StepObservation& step, const limiter::SourceNodes& nodes) {
    const auto& receipt = step.structural_limiter;
    const auto& value = receipt.values;
    const auto& physical = setup.execution().model();
    const auto& domain = physical.source_domain().domain();
    using Kind = tl::fea::NodalCinLimitKind;
    rapidjson::Writer<Sink> json(sink);
    const auto integer = [&](const char* key, std::uint64_t x) {json.Key(key);json.Uint64(x);};
    const auto real = [&](const char* key, double x) {
        json.Key(key);
        output::Require(json.Double(x),"Nonfinite limiter report value");
    };
    const auto text = [&](const char* key, const char* x) {json.Key(key);json.String(x);};
    const auto source_id = [&](const char* key, std::uint64_t x) {
        json.Key(key);
        if(x) json.Uint64(x); else json.Null();
    };
    json.StartObject();
    text("schema","robo_dyna.accepted_cin_structural_limiter.v1");
    text("scope","Post-CIN scalar surrogate admission; not a full nonlinear/contact stability theorem");
    text("coefficient_phase","Accepted pre-kick geometry after actual CIN transfer, before motion");
    integer("owner_id",receipt.owner_id); integer("source_instance_id",receipt.source_instance_id);
    integer("base_epoch",receipt.base_epoch); integer("accepted_epoch",step.base.epoch+1);
    integer("attempt",receipt.attempt); integer("cin_qualification_id",receipt.cin_qualification_id);
    integer("physical_nodes",domain.node_count());
    real("base_time_s",receipt.base_time_s); real("accepted_time_s",step.proposed_time);
    real("owner_fixed_dt_s",receipt.owner_fixed_dt_s); real("factor",value.factor);
    text("kind",value.kind == Kind::RigidTrace ? "rigid_trace" :
        value.kind == Kind::OrdinaryTranslation ? "ordinary_translation" :
        value.kind == Kind::OrdinaryRotation ? "ordinary_rotation" : "unbounded");
    json.Key("minimum_dt_s");
    if(value.kind == Kind::Unbounded) json.Null(); else json.Double(value.minimum_dt_s);
    source_id("source_node_id",receipt.source_node_id);
    if(value.kind != Kind::Unbounded) integer("physical_node_index",value.node);
    if(value.kind == Kind::RigidTrace) {
        integer("group_index",value.group);
        text("group_source_kind",receipt.source_kind == tl::fea::RigidBindingSourceKind::Part ? "part" : "nodal_group");
        source_id("source_group_id",receipt.source_group_id);
        source_id("source_node_set_id",receipt.source_node_set_id);
        real("aggregate_mass_kg",value.mass_kg);
        json.Key("principal_inertia_kg_m2");json.StartArray();
        json.Double(value.principal_inertia_kg_m2.x);json.Double(value.principal_inertia_kg_m2.y);
        json.Double(value.principal_inertia_kg_m2.z);json.EndArray();
        real("trace_upper_per_s2",value.trace_upper_per_s2);
    } else if(value.kind != Kind::Unbounded) {
        integer("translation_fixed_bits",value.translation_fixed_bits);
        integer("rotation_fixed",value.rotation_fixed);
        integer("rotation_present",value.rotation_present);
        real("post_transfer_mass_kg",value.mass_kg);
        real("post_transfer_inertia_kg_m2",value.inertia_kg_m2);
        real("post_transfer_translation_stiffness_n_per_m",value.translation_stiffness_n_per_m);
        real("post_transfer_rotation_stiffness_nm",value.rotation_stiffness_nm);
    }
    text("incidence_scope","Complete prepared source incidence on winning node/group and direct CIN donors; not current per-element stiffness shares or force attribution");
    text("thickness_scope","Shell thickness, density and modulus below are immutable prepared reference SI inputs, not current material thickness/tangent");
    text("unseparated_stiffness","Actual total includes any assembled wall and automatic TYPE45 stiffness; this report does not reconstruct their individual shares");
    json.Key("source_nodes");json.StartArray();
    for(const auto node : nodes.with_donors) {
        const auto& coefficient = physical.coefficients().nodes()[node].coefficients;
        json.StartObject();
        integer("physical_node_index",node);integer("original_nid",domain.nodes()[node].source_id);
        text("role",nodes.Direct(node) ? "winner_or_group_member" : "cin_secondary_donor");
        real("initial_ledger_mass_kg",coefficient.mass);
        real("initial_ledger_inertia_kg_m2",coefficient.isotropic_inertia);
        real("initial_shell_mass_kg",coefficient.shell.mass);
        real("initial_shell_physical_inertia_kg_m2",coefficient.shell.physical_inertia);
        real("initial_shell_added_inertia_kg_m2",coefficient.shell.added_inertia);
        json.EndObject();
    }
    json.EndArray();
    json.Key("cin_donor_rows");json.StartArray();
    const auto rows = setup.attachments().attachments().model().rows();
    for(std::size_t i = 0; i < rows.count; ++i) {
        const auto& row = rows[i];
        bool relevant = false;
        for(const auto node : row.master_domain_nodes) relevant |= nodes.Direct(node);
        if(!relevant) continue;
        json.StartObject();integer("row",i);
        integer("secondary_nid",domain.nodes()[row.secondary_domain_node].source_id);
        integer("declared_master_eid",row.master_source.element_id);
        integer("declared_master_pid",row.master_source.part_id);
        json.Key("ordered_master_nids");json.StartArray();
        for(const auto node : row.master_domain_nodes) json.Uint64(domain.nodes()[node].source_id);
        json.EndArray();json.EndObject();
    }
    json.EndArray();
    json.Key("incident_source_elements");json.StartArray();
    limiter::VisitRows(setup,nodes,[&](const char* family, std::uint64_t eid, std::uint64_t pid,
        std::uint64_t sid, std::uint64_t mid, double thickness, double density, double young) {
        json.StartObject();text("family",family);integer("source_element_id",eid);
        source_id("part_id",pid);source_id("section_or_property_id",sid);source_id("material_id",mid);
        if(thickness > 0) {
            real("reference_thickness_m",thickness);real("reference_density_kg_m3",density);
            real("reference_young_modulus_pa",young);
        }
        json.EndObject();
    });
    json.EndArray();json.EndObject();
}
}
std::string StructuralLimiterReport(const VehiclePhysicalDynamics& dynamics, std::size_t cap) {
    output::Require(cap && cap <= (8u << 20),"Invalid structural limiter report cap");
    const auto* setup = dynamics.wall_setup();
    output::Require(setup,"Structural source report requires the actual loaded source setup");
    const auto& step = dynamics.last_accepted_step();
    const auto accepted = dynamics.accepted();
    output::Require(limiter::MatchesAccepted(step,accepted),
        "Structural limiter is unavailable or differs from the actual accepted interval");
    const auto nodes = limiter::SelectNodes(*setup,step.structural_limiter);
    Sink count{nullptr,cap};
    Render(count,*setup,step,nodes);
    std::string result;
    result.reserve(count.count);
    Sink write{&result,cap};
    Render(write,*setup,step,nodes);
    output::Require(write.count == count.count,"Structural limiter report changed between passes");
    return result;
}
} // namespace crash::cases::vehicle_dynamics
