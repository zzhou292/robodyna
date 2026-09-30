#include "SourceAssemblyWallFields.h"

namespace crash::output::assembly::wall_fields {
void CheckContactPhase(const FrameView& v) {
    const auto& a=v.contact;Require(v.stamp&&v.bindings&&v.setup&&v.diagnostics&&a.diagnostics&&a.nodes&&a.parents&&a.wall_face,
        "Missing assembly contact fields");
    const auto& source=v.bindings->source().data();const auto& s=*v.stamp;const auto& c=*a.diagnostics;
    const auto& geometry=*v.setup->source_geometry();const auto& w=*geometry.weights();const auto& settings=*v.setup->settings();
    Require(s.epoch&&a.node_count==source.nodes.size()&&a.parent_count==source.parents.size()&&
        w.node_count()==a.node_count&&w.parent_count()==a.parent_count&&c.node_count==a.node_count&&c.parent_count==a.parent_count,
        "Incomplete assembly contact extents");
    const auto& q=v.diagnostics->shells.qeph;const auto& t=v.diagnostics->shells.t3;
    Require(c.valid&&c.phase==tlfea::contact::NodalWallDevicePhase::PreparedCandidate&&c.owner_id==s.owner_id&&
        c.base_epoch==s.reaction_base_epoch&&c.attempt&&c.attempt==q.attempt&&c.attempt==t.attempt&&
        c.time==s.time&&c.base_time==s.reaction_time&&c.velocity_time==s.velocity_time&&c.kick_dt==s.reaction_kick_dt&&
        c.base_velocity_time==q.base_velocity_time&&c.base_velocity_time==t.base_velocity_time&&
        c.scheme==s.temporal_scheme&&c.velocity_phase==s.velocity_phase&&c.configuration_id==settings.configuration_id&&
        c.qualification_id==settings.qualification_id&&c.wall_binding_id==settings.wall_binding_id,
        "Contact completed candidate is not the committed assembly interval");
}
} // namespace crash::output::assembly::wall_fields
