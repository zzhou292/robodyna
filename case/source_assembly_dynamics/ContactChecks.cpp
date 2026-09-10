#include "State.h"
#include "lib_src/collision/Q4ContactBounds.h"
#include <algorithm>
#include <cmath>

namespace crash::cases::source_assembly_dynamics {
namespace {
bool Nonnegative(double x) noexcept { return std::isfinite(x)&&x>=0; }
bool Certificate(contact::Q4CertifiedIntegral c) noexcept {
    return Nonnegative(c.value)&&contact::nodal_wall_detail::Certificate(c);
}
bool Finite(contact::Vec3 v) noexcept { return std::isfinite(v.x)&&std::isfinite(v.y)&&std::isfinite(v.z); }
}
Report SourceAssemblyWallCase::Impl::CheckContact() {
    auto& next=candidate();auto& out=next.diagnostics;const auto& old=accepted().diagnostics;
    const auto& result=next.wall;const auto& d=result.diagnostics;const auto& settings=*setup.settings();
    const auto& weights=*setup.source_geometry()->weights();
    if(!d.valid||d.phase!=contact::NodalWallDevicePhase::PreparedCandidate||
       d.owner_id!=prepared.owner_id||d.base_epoch!=prepared.kinematics.base_epoch||d.attempt!=prepared.attempt||
       d.scheme!=prepared.temporal_scheme||d.velocity_phase!=prepared.velocity_phase||
       d.time!=prepared.proposed_time||d.velocity_time!=prepared.velocity_time||d.base_time!=prepared.base_time||
       d.base_velocity_time!=prepared.base_velocity_time||d.kick_dt!=prepared.kick_dt||
       d.configuration_id!=settings.configuration_id||d.qualification_id!=settings.qualification_id||
       d.wall_binding_id!=settings.wall_binding_id||d.node_count!=nodes()||d.parent_count!=quads()+triangles()||
       weights.node_count()!=nodes()||weights.parent_count()!=quads()+triangles()||weights.global_node_count()!=nodes())
        return Failure(Status::ComponentFailure,"Contact source, complete extent or completed interval identity disagrees");
    if(!Certificate(d.resultant)||!Certificate(d.potential)||!Certificate(base_contact.resultant)||!Certificate(base_contact.potential))
        return Failure(Status::EnvelopeFailure,"Contact global force/potential certificate is invalid");
    for(double x:{d.maximum_penetration,d.stiffness_rate_bound,d.base_potential,d.base_potential_error,
                  d.kick_work_roundoff,d.drift_work_roundoff,d.work_uncertainty,d.quadratic_work_upper,
                  d.wall_kick_impulse,d.wall_kick_impulse_error,d.wall_kick_moment_error.x,
                  d.wall_kick_moment_error.y,d.wall_kick_moment_error.z})
        if(!Nonnegative(x))return Failure(Status::EnvelopeFailure,"Contact interval bound is nonfinite or negative");
    for(double x:{d.surface_power,d.potential_increment,d.kick_work,d.drift_work,d.conservative_defect})
        if(!std::isfinite(x))return Failure(Status::EnvelopeFailure,"Contact interval work is nonfinite");
    if(!Finite(d.wall_reaction)||!Finite(d.wall_moment)||!Finite(d.wall_kick_moment)||
       d.maximum_penetration>settings.penalty.penetration_cap)
        return Failure(Status::EnvelopeFailure,"Contact resultant/moment/penetration is invalid");
    double upper=0;
    if(!contact::q4_bounds::AddScalar(d.quadratic_work_upper,d.work_uncertainty,true,&upper)||
       d.conservative_defect< -d.work_uncertainty||d.conservative_defect>upper)
        return Failure(Status::EnvelopeFailure,"Local contact force/potential work certificate failed",0,SIZE_MAX,d.conservative_defect,upper);
    for(std::size_t e=0;e<result.parents.size();++e) {
        const auto& p=result.parents[e];const auto& expected=weights.parent(static_cast<unsigned>(e));
        if(!p.valid||p.parent_element_id!=expected.parent_element_id||p.parent_face_id!=expected.parent_face_id||
           p.feature_id!=expected.feature_id||p.arity!=expected.arity||p.family!=expected.family||
           !Certificate(p.resultant)||!Certificate(p.potential)||p.resultant.error>settings.parent_force_error||
           p.potential.error>settings.parent_energy_error)
            return Failure(Status::SourceMismatch,"Contact parent identity or complete parent certificate is invalid",expected.parent_element_id);
        for(unsigned local=0;local<p.arity;++local)
            if(!Certificate(p.force[local])||p.force[local].error>settings.parent_force_error)
                return Failure(Status::EnvelopeFailure,"Contact parent nodal certificate is invalid",expected.parent_element_id);
    }
    const double wall_x=setup.placed_wall()->geometry()->wall_x();
    for(std::size_t n=0;n<nodes();++n) {
        const auto& p=result.nodes[n];
        if(!p.valid||p.node!=n||weights.node(static_cast<unsigned>(n)).node!=n||
           p.base_epoch!=prepared.kinematics.base_epoch||p.attempt!=prepared.attempt||p.fixed||
           !Certificate(p.force)||!Certificate(p.potential)||!Certificate(p.stiffness)||
           !Finite(p.force_world)||!Finite(p.wall_point)||!Finite(p.wall_reaction)||!Finite(p.wall_moment)||
           !std::isfinite(p.surface_power)||p.local_velocity_first_timestep!=0||
           p.force_world.x!=-p.force.value||p.force_world.y!=0||p.force_world.z!=0||
           p.wall_reaction.x!=p.force.value||p.wall_reaction.y!=0||p.wall_reaction.z!=0||p.wall_point.x!=wall_x||
           !std::binary_search(wall_faces.begin(),wall_faces.end(),result.wall_face[n]))
            return Failure(Status::ComponentFailure,"Contact unique node/finite mesh face/certificate is invalid",0,n);
        // Shares were already reduced and assembled once by the contributor.
        // Do not add a second parent-force sum to motion or applied-work inputs.
        if(p.force.value>0)++out.active_contact_nodes;
    }
    out.first_contact_epoch=old.first_contact_epoch;out.last_contact_epoch=old.last_contact_epoch;
    out.contact_intervals=old.contact_intervals;
    if(out.active_contact_nodes) {
        const auto epoch=old.stamp.epoch+1;
        if(!out.first_contact_epoch)out.first_contact_epoch=epoch;
        out.last_contact_epoch=epoch;++out.contact_intervals;
    }
    return Success();
}
} // namespace crash::cases::source_assembly_dynamics
