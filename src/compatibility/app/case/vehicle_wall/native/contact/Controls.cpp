#include "Values.h"
#include "output/ArtifactIO.h"
namespace crash::cases::vehicle_wall::native::wall_interface::detail {
Controls ResolveControls(const WallSource& wall,const VehicleSource& vehicle,Declaration declaration) {
    output::Require(declaration.profile==Profile::AllRetainedVehicleNodesToFixedMeshV1 &&
        declaration.topology_generation && declaration.source_generation && wall.ids().interface &&
        wall.declaration().profile==native::Profile::EnvelopeFixedElasticV1,
        "Explicit finite-wall contact declaration/identity is required");
    return DeclaredControls(vehicle.provenance().units,wall.declaration().wall_friction);
}
Controls DeclaredControls(n::UnitScale units,double friction) {
    Controls out;
    auto& config=out.runtime;
    config.units=units;
    config.response_mass=n::ResponseMassPolicy::AcceptedOwnerCoefficients;
    config.physical_source=n::PhysicalSourceProfile::CompleteBoundLedger;
    config.activity=n::ContactActivityPolicy::AllActivePrefix;
    auto& selection=config.lifecycle.selection;
    selection.gap_mode=1;
    selection.initial_penetration=5;
    selection.local_processor=1;
    selection.foreign_rows=false;
    selection.thermal=false;
    selection.gap_loading=false;
    auto& geometry=config.lifecycle.geometry;
    geometry.gap_mode=1;
    geometry.sharp=1;
    geometry.initial_penetration=5;
    geometry.damping_flag=1;
    geometry.adhesion=false;
    geometry.thermal=false;
    geometry.foreign_row=false;
    config.lifecycle.coefficient.stiffness_formulation=4;
    config.lifecycle.coefficient.mass_timestep_augmentation=0;
    config.lifecycle.minimum_coefficient=0;
    config.lifecycle.maximum_coefficient=n::native_constant::ep20*n::native_constant::ep10;
    config.lifecycle.neighbor_removal=2;
    config.lifecycle.optcd_response_precision=0;
    config.lifecycle.main_coefficient_domain=n::MainCoefficientDomain::Nonnegative;
    config.normal.stiffness_formulation=4;
    config.normal.damping_flag=1;
    config.normal.initial_penetration=5;
    config.normal.arithmetic_precision=8;
    config.normal.prescribed_contact_force=false;
    config.normal.adhesion=false;
    config.normal.damping_factor=.05;
    config.normal.engine.kdtint=0;
    config.normal.engine.idtmins=0;
    config.normal.engine.idtmins_int=0;
    config.friction.model=2;
    config.friction.formulation=10;
    config.friction.orthotropic=0;
    config.friction.converged=1;
    config.friction.thermal=0;
    config.friction.part_coefficients=0;
    config.friction.alpha=1;
    config.friction_coefficients.base=friction;
    // The six existing Darmstad extra coefficients remain exactly zero. This
    // declared profile uses the already-qualified response with constant mu.
    config.assembly.parallel_assembly=0;
    config.assembly.pinch=0;
    config.assembly.thermal=0;
    config.assembly.thermal_formulation=0;
    config.assembly.thermal_nodal_timestep=0;
    config.assembly.engine.kdtint=0;
    config.assembly.engine.idtmins=0;
    config.assembly.engine.idtmins_int=0;
    out.gaps.property_type=1;
    out.gaps.input_thickness_mode=0;
    out.gaps.level=1;
    out.gaps.gap_mode=1;
    out.gaps.free_edge_gap=0;
    out.gaps.contact_thickness_update=0;
    out.gaps.scale=1;
    out.gaps.maximum_secondary=n::native_constant::ep20*n::native_constant::ep10;
    out.gaps.maximum_main=n::native_constant::ep20*n::native_constant::ep10;
    out.search.level=1;
    out.search.gap_mode=1;
    out.search.neighbor_removal=2;
    out.search.initial_penetration=5;
    out.search.edge_mode=0;
    out.search.thermal_mode=0;
    out.search.curvature=0;
    out.search.partitions=1;
    out.search.initialization=n::search_startup::Initialization::SerialNative;
    out.search.gap_load_cards=n::search_startup::LoadCards::Absent;
    return out;
}

}
