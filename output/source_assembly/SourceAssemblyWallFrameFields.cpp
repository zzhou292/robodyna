#include "SourceAssemblyWallFields.h"
#include "WallFieldValues.h"

namespace crash::output::assembly::wall_fields {
Document FrameDocument(const FrameView& v) {
    CheckFrame(v);const auto& s=*v.stamp;const auto& source=v.bindings->source().data();
    Document d;d.SetObject();String(d,"schema",WallFrameSchema);String(d,"kind",WallArtifactKind);
    String(d,"source_inventory_sha256",source.identity.sha256);Integer(d,"source_inventory_bytes",source.identity.bytes);
    Integer(d,"owner_id",s.owner_id);Integer(d,"run_id",v.surface->binding().identity.run);
    Integer(d,"topology_id",v.surface->binding().identity.topology);Integer(d,"source_instance_id",v.bindings->source_instance_id());
    Integer(d,"configuration_id",v.setup->settings()->configuration_id);Integer(d,"qualification_id",v.setup->settings()->qualification_id);
    Integer(d,"wall_binding_id",v.setup->settings()->wall_binding_id);Integer(d,"accepted_epoch",s.epoch);Number(d,"accepted_time_s",s.time);
    Child(d,"stamp",StampDocument(s));Document n;n.SetObject();
    String(n,"position_phase","accepted_endpoint");String(n,"velocity_phase",s.epoch?"previous_midpoint":"collocated");
    FiniteArray(n,"position_xyz_m",v.nodes.position_xyz,3*s.node_count);
    FiniteArray(n,"velocity_xyz_m_per_s",v.nodes.velocity_xyz,3*s.node_count);
    FiniteArray(n,"orientation_wxyz",v.nodes.orientation_wxyz,4*s.node_count);
    FiniteArray(n,"angular_velocity_xyz_rad_per_s",v.nodes.angular_velocity_xyz,3*s.node_count);Child(d,"nodal_fields",n);
    String(d,"stress_frame","native_corotational_shell_axes");
    String(d,"stress_semantics","Native three-thickness-point XX,YY,XY,YZ,ZX components in each current shell basis; not world-axis stress");
    Child(d,"sections",SectionFieldDocument(*v.surface,v.qeph,v.t3));Child(d,"diagnostics",DiagnosticsDocument(*v.diagnostics));
    if(s.epoch)Child(d,"contact",ContactDocument(v));else Put(d,"contact",Value());
    if(v.observe_force_stage) {
        if(s.epoch)Child(d,"force_stage_kinetic",ForceStageDocument(*v.force_stage));
        else Put(d,"force_stage_kinetic",Value());
    }
    return d;
}
} // namespace crash::output::assembly::wall_fields
