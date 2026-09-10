#include "SourceAssemblyWallFields.h"
#include "WallFieldValues.h"
#include "output/SurfaceBindingFields.h"
#include <cmath>

namespace crash::output::assembly::wall_fields {
Document ConfigurationDocument(const cases::source_assembly::SourceAssemblyBindings& b,
    const cases::source_assembly::SourceAssemblyWallSetup& setup,const dynamics::Config& c,
    const SourceAssemblySurface& surface,const WallArchiveRequest& r) {
    // The production adapter receives the initialized case's immutable Config.
    // This raw formatter checks representation/scope, not dynamics admission.
    Require(setup.initialized()&&std::isfinite(c.fixed_dt)&&c.fixed_dt>=1e-12&&b.materials().Matches(b.shells())&&
        setup.bindings()->shells().inventory()==b.shells().inventory()&&surface.source().data().identity.sha256==b.source().data().identity.sha256&&
        surface.binding().identity.run==r.run_id&&surface.binding().identity.topology==r.topology_id,
        "Configuration must retain the complete prepared assembly and explicit run");
    const auto plan=PlanWallArchive(r,b.source().data().identity.bytes,setup.placed_wall()->source_manifest()->size());
    const double horizon=static_cast<double>(r.steps)*c.fixed_dt;Require(std::isfinite(horizon)&&horizon>0,"Requested horizon cannot be represented");
    Document d;d.SetObject();String(d,"schema",WallConfigurationSchema);String(d,"kind",WallArtifactKind);
    String(d,"scope","Original six-part Yaris component, internal nodal rigid groups active, external connections explicitly released");
    Boolean(d,"shell_model",true);Boolean(d,"vehicle_model",false);
    if(c.observe_force_stage)Boolean(d,"observe_force_stage",true);
    String(d,"units","SI; physical geometry scale 1");
    Integer(d,"owner_id",surface.binding().identity.owner);Integer(d,"run_id",r.run_id);Integer(d,"topology_id",r.topology_id);
    Integer(d,"asset_id",r.asset_id);Integer(d,"source_instance_id",b.source_instance_id());
    Integer(d,"configuration_id",setup.settings()->configuration_id);Integer(d,"qualification_id",setup.settings()->qualification_id);
    Integer(d,"wall_binding_id",setup.settings()->wall_binding_id);String(d,"source_inventory_file","source-assembly-inventory.json");
    String(d,"source_inventory_sha256",b.source().data().identity.sha256);Integer(d,"source_inventory_bytes",b.source().data().identity.bytes);
    Number(d,"fixed_dt_s",c.fixed_dt);Number(d,"requested_horizon_s",horizon);Integer(d,"requested_steps",r.steps);Integer(d,"frame_every",r.frame_every);
    Integer(d,"archive_byte_cap",r.limits.total_bytes);Integer(d,"artifact_file_byte_cap",r.limits.file_bytes);Integer(d,"frame_cap",r.limits.frames);
    Integer(d,"forecast_bytes",plan.forecast_bytes);Integer(d,"forecast_frames",plan.frame_capacity);Integer(d,"forecast_files",plan.forecast_files);
    String(d,"stress_frame","native_corotational_shell_axes");
    String(d,"kinetic_policy","Native physical-node stored kinetic and ordinary-plus-aggregate stored kinetic remain separate; endpoint positions and midpoint velocities with lagged group axes; no collocated reconstruction");
    String(d,"work_policy","Native shell and viscous work counted once; plastic work is a diagnostic, reaction kick work is not dissipation; no independent global physical-energy certificate");
    String(d,"contact_result_policy","Initial frame has certified separation and null contact; later copied completed candidate belongs to the same accepted owner/material publication");
    AppendSurfaceBinding(d,surface.binding());Child(d,"input",InputDocument(b));Child(d,"wall_setup",SetupDocument(setup));
    Document e;e.SetObject();const auto& x=c.deformation;
    Number(e,"maximum_displacement_m",x.maximum_displacement);Number(e,"maximum_rotation_rad",x.maximum_rotation);
    Number(e,"maximum_rotation_increment_rad",x.maximum_rotation_increment);Number(e,"maximum_strain",x.maximum_strain);
    Number(e,"maximum_thickness_curvature",x.maximum_thickness_curvature);Number(e,"minimum_area_ratio",x.minimum_area_ratio);
    Number(e,"maximum_area_ratio",x.maximum_area_ratio);Number(e,"minimum_thickness_ratio",x.minimum_thickness_ratio);
    Number(e,"maximum_thickness_ratio",x.maximum_thickness_ratio);Number(e,"maximum_native_dt_fraction",x.maximum_native_dt_fraction);Child(d,"deformation_limits",e);
    Document storage;storage.SetObject();const auto& z=c.storage;
    Integer(storage,"max_nodes",z.max_nodes);Integer(storage,"max_parents",z.max_parents);Integer(storage,"max_host_bytes",z.max_host_bytes);
    Integer(storage,"max_device_bytes",z.max_device_bytes);Integer(storage,"owner_device_bytes",z.owner_device_bytes);
    Integer(storage,"qeph_device_bytes",z.qeph_device_bytes);Integer(storage,"t3_device_bytes",z.t3_device_bytes);
    Integer(storage,"publication_max_nodes",z.publication.max_nodes);Integer(storage,"publication_device_bytes",z.publication.max_device_bytes);
    Integer(storage,"publication_host_bytes",z.publication.max_host_bytes);Integer(storage,"contact_max_parents",z.contact.counts.parents);
    Integer(storage,"contact_max_nodes",z.contact.counts.nodes);Integer(storage,"contact_max_global_nodes",z.contact.counts.global_nodes);
    Integer(storage,"contact_device_bytes",z.contact.max_device_bytes);Integer(storage,"contact_host_bytes",z.contact.max_host_bytes);Child(d,"storage_limits",storage);
    AppendCsvLedgerSegments(d,&plan.intervals,1);return d;
}
} // namespace crash::output::assembly::wall_fields
