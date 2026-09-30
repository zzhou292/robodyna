#include "AcceptedSelfContact.h"
#include "case/vehicle_dynamics/VehiclePhysicalDynamics.h"
#include "case/vehicle_self_contact/VehicleSelfContactStartup.h"

namespace crash::output::physical_run::detail {
void CaptureSelfContact(const cases::vehicle_dynamics::VehiclePhysicalDynamics& run,
    const records::Identity& id,Profile profile,Values& value) {
    const auto* setup=run.self_contact_setup();
    const auto* forecast=run.self_contact_forecast();
    const auto& step=run.last_accepted_step();
    const auto& self=step.self_contact;
    Require(bool(setup)==self.enabled && bool(forecast)==self.enabled &&
        profile.self_contact==self.enabled,"Committed self-contact presence/profile differs");
    if(!self.enabled)return;
    const auto& identity=forecast->identity;
    const auto& force=self.accepted_force;
    const auto& policy=self.policy_summary;
    Require(identity.source_id && identity.roster_entries==1 &&
        identity.owner_id==value.owner && identity.configuration_id==id.configuration &&
        identity.qualification_id==id.qualification && identity.physical_source_instance_id==id.source_instance &&
        identity.active_use_identity && force.active_use_identity==identity.active_use_identity &&
        force.valid && force.owner_id==value.owner && force.configuration_id==id.configuration &&
        force.qualification_id==id.qualification && force.base_epoch==value.stamp.base_epoch &&
        force.attempt==value.stamp.attempt && Bits(force.position_time)==Bits(value.stamp.base_time) &&
        Bits(force.velocity_time)==Bits(step.base.velocity_time) &&
        force.temporal_scheme==step.base.temporal_scheme && force.velocity_phase==step.base.velocity_phase &&
        force.temporal_scheme==tl::fea::NodalTemporalScheme::StaggeredHalfKickStart &&
        force.event_count<=forecast->transaction.accepted_event_capacity,
        "Committed self-contact force source/owner/accepted-base phase differs");
    Require(policy.complete && self.policy_outcomes==policy.outcomes &&
        policy.outcomes==self.candidate_facet_pairs && self.regularity_generation &&
        (force.event_count ? (force.first_source_order==0 && force.last_source_order==force.event_count-1) :
            (force.first_source_order==UINT64_MAX && force.last_source_order==UINT64_MAX)),
        "Committed self-contact candidate census or force-source coverage is incomplete");
    SelfContactValues out;
    out.source_id=identity.source_id;
    out.selected_parents=forecast->transaction.surface_parent_map_capacity;
    out.events=force.event_count;out.vertex_face_events=force.vertex_face_event_count;
    out.boundary_vertex_edge_events=force.boundary_vertex_edge_event_count;
    out.edge_edge_events=force.edge_edge_event_count;out.active_events=force.active_count;
    out.accepted_parent_pairs=self.accepted_broadphase_pairs;out.accepted_facet_pairs=self.accepted_facet_pairs;
    out.discovered_features=self.accepted_discovered_features;out.regularity_generation=self.regularity_generation;
    out.candidate_parent_pairs=self.candidate_broadphase_pairs;out.candidate_facet_pairs=self.candidate_facet_pairs;
    out.policy_outcomes=policy.outcomes;out.certified_separated=policy.certified_separated;
    out.same_rigid_exclusions=policy.excluded_same_rigid_group;
    out.local_intersections=policy.excluded_local_intersection;
    out.represented_vf=policy.represented_by_accepted_vf;out.represented_ee=policy.represented_by_accepted_ee;
    out.policy_digest=policy.digest;out.active_parents=self.active_parents;
    out.removing_parents=self.removing_parents;out.skipped_parents=self.skipped_parents;
    out.base_velocity_time=force.velocity_time;out.potential_j=force.potential_j;
    out.maximum_force_n=force.maximum_force_norm_n;out.maximum_sti_n_m=force.maximum_sti_diagonal_n_m;
    out.maximum_represented_stiffness_n_m=force.maximum_represented_stiffness_n_m;
    const auto vector=[](tlfea::contact::Vec3 v) {return std::array<double,3>{v.x,v.y,v.z};};
    out.endpoint_a_n=vector(force.endpoint_a_resultant_n);out.endpoint_b_n=vector(force.endpoint_b_resultant_n);
    out.equal_opposite_residual_n=vector(force.equal_opposite_residual_n);
    out.global_moment_n_m=vector(force.global_moment_n_m);
    CheckSelfContactValues(out);
    value.self_contact=out;
}
} // namespace crash::output::physical_run::detail
