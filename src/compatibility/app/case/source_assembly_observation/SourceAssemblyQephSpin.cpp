#include "SourceAssemblyQephSpinInternal.h"
#include "lib_src/elements/qeph/QephHistory.h"

namespace crash::cases::source_assembly_observation {
namespace {
using Vec=tl::math::Vec3;
bool Finite(Vec v) noexcept{return std::isfinite(v.x)&&std::isfinite(v.y)&&std::isfinite(v.z);}
template<std::size_t N> bool Finite(const double (&v)[N]) noexcept {
    for(double x:v)if(!std::isfinite(x))return false;return true;
}
double Dot(Vec a,Vec b) noexcept{return a.x*b.x+a.y*b.y+a.z*b.z;}
Vec World(const tl::math::Matrix3& m,Vec v) noexcept {
    return {m.v[0]*v.x+m.v[1]*v.y+m.v[2]*v.z,m.v[3]*v.x+m.v[4]*v.y+m.v[5]*v.z,m.v[6]*v.x+m.v[7]*v.y+m.v[8]*v.z};
}
}
Report spin_detail::Parent(const QephSpinInput& in,const modelio::assembly::Parent& p,std::size_t local,bool candidate,QephSpinParent& out) noexcept {
    const auto& reference=in.bindings->shells().qeph_reference(p.family_index);
    const auto& force=(candidate?in.candidate_parents:in.parents)[p.family_index];
    const auto& section=(candidate?in.candidate_sections:in.sections)[p.family_index];const auto& k=force.kinematics;
    const auto fields=candidate?in.after:in.before;
    const auto epoch=in.base.epoch+(candidate?1:0);const auto time=candidate?in.prepared.proposed_time:in.base.time;
    if(!force.proposed_history.matches_reference(reference)||force.proposed_history.stamp().sample_index!=epoch||
       force.proposed_history.stamp().time!=time)
        return {Status::WrongIdentity,"Spin history lost its native reference/accepted stamp"};
    if(!fe::qeph::detail::ValidHistoryValues(force.proposed_history.data()))
        return {Status::NonfiniteResult,"Spin retained native history is invalid"};
    out.source_parent=p.source_id;out.source_part=p.part_id;out.source_material=p.material_id;
    out.source_section=p.section_id;out.source_curve=p.curve_id;out.source_index=p.index;out.family_index=p.family_index;out.local_node=local;
    out.force=force;out.section=section;
    for(unsigned l=0;l<4;++l) {
        out.source_nodes[l]=reference.input.node_ids[l];out.position[l]=detail::Vector(fields.position_xyz,p.nodes[l]);
        out.velocity[l]=detail::Vector(fields.velocity_xyz,p.nodes[l]);out.omega[l]=detail::Vector(fields.angular_velocity_xyz,p.nodes[l]);
        if(!Finite(out.position[l])||!Finite(out.velocity[l])||!Finite(out.omega[l])||
           !Finite(force.internal_force[l])||!Finite(force.internal_couple[l]))return {Status::NonfiniteResult,"Spin parent motion/load is nonfinite"};
    }
    for(const auto& point:section.history.point) {
        for(double value:point.stress)if(!std::isfinite(value))return {Status::NonfiniteResult,"Spin point stress is nonfinite"};
        if(!std::isfinite(point.plastic_strain)||!std::isfinite(point.filtered_rate_per_s))
            return {Status::NonfiniteResult,"Spin point history is nonfinite"};
    }
    if(!std::isfinite(section.cumulative_plastic_work_J))return {Status::NonfiniteResult,"Spin plastic-work diagnostic is nonfinite"};
    const auto& d=force.diagnostics;
    for(double value:{d.effective_thickness,d.native_sound_speed,d.membrane_viscosity,d.stabilization_viscosity,
                      d.translational_stiffness,d.rotational_stiffness,d.unscaled_element_dt,
                      d.internal_work_increment[0],d.internal_work_increment[1],d.hourglass_viscous_work_increment})
        if(!std::isfinite(value))return {Status::NonfiniteResult,"Spin force diagnostic is nonfinite"};
    // Startup contains initialized zero forces/history but has never evaluated
    // a native interval. Do not invent a kinematics packet or force stage there.
    if(!epoch)return detail::Success();
    if(k.sample_index!=epoch||k.base_time!=(candidate?in.base.time:in.base.reaction_time)||k.dt!=in.base.fixed_dt)
        return {Status::WrongIdentity,"Spin retained rate packet has a different originating interval"};
    if(!Finite(k.frame.v)||!Finite(k.nodal_factors)||!Finite(k.projection_inverse)||!Finite(k.projected_omega)||!Finite(k.regular_rate)||!Finite(k.hourglass_rate))
        return {Status::NonfiniteResult,"Spin native kinematics contains nonfinite entries"};
    for(unsigned l=0;l<4;++l)if(!Finite(k.local_position[l])||!Finite(k.local_normals[l])||!Finite(k.projection_columns[l]))
        return {Status::NonfiniteResult,"Spin native geometry contains nonfinite entries"};
    for(double value:{k.area,k.reciprocal_area,k.characteristic_length,k.raw_warpage_abs,k.effective_warpage})
        if(!std::isfinite(value))return {Status::NonfiniteResult,"Spin force/geometry diagnostic is nonfinite"};
    out.native_normal=World(k.frame,k.local_normals[local]);
    const auto w=out.omega[local],c=force.internal_couple[local],n=out.native_normal;
    const double length=std::hypot(std::hypot(n.x,n.y),n.z);
    if(!Finite(n)||std::abs(length-1)>1e-12)return {Status::InvalidMetric,"Spin native normal is not finite/unit"};
    out.normal_spin=Dot(w,n);out.normal_internal_couple=Dot(c,n);
    const Vec wt{w.x-out.normal_spin*n.x,w.y-out.normal_spin*n.y,w.z-out.normal_spin*n.z};
    out.tangent_spin=std::hypot(std::hypot(wt.x,wt.y),wt.z);
    out.normal_internal_power=out.normal_spin*out.normal_internal_couple;
    out.tangent_internal_power=Dot(c,wt);
    for(double value:{out.normal_spin,out.tangent_spin,out.normal_internal_couple,out.normal_internal_power,out.tangent_internal_power})
        if(!std::isfinite(value))return {Status::NonfiniteResult,"Spin projected diagnostic overflows"};
    out.has_native_kinematics=true;return detail::Success();
}
Report ObserveQephSpin(const QephSpinInput& in,QephSpinObservation* output) noexcept {
    spin_detail::Selection selected;auto report=spin_detail::Check(in,output,sizeof(*output),selected);if(!report)return report;
    QephSpinObservation next;next.base=in.base;next.attempt=in.prepared.attempt;
    next.source_instance=in.bindings->source_instance_id();next.source_node=in.source_node;next.global_node=selected.node;
    next.parent_count=selected.count;next.native=in.bindings->shells().nodes()[selected.node].native;
    next.position=detail::Vector(in.before.position_xyz,selected.node);next.velocity=detail::Vector(in.before.velocity_xyz,selected.node);
    next.omega=detail::Vector(in.before.angular_velocity_xyz,selected.node);next.applied_force=detail::Vector(in.applied_force_xyz,selected.node);
    next.applied_couple=detail::Vector(in.applied_couple_xyz,selected.node);
    for(unsigned a=0;a<4;++a)next.orientation[a]=in.before.orientation_wxyz[4*selected.node+a];
    if(!tl::math::UnitQuaternion({next.orientation[0],next.orientation[1],next.orientation[2],next.orientation[3]})||
       !Finite(next.applied_force)||!Finite(next.applied_couple))return {Status::NonfiniteResult,"Spin nodal orientation/load is invalid"};
    for(std::size_t i=0;i<selected.count;++i) {
        report=spin_detail::Parent(in,in.bindings->source().data().parents[selected.parents[i]],selected.locals[i],false,next.parents[i]);if(!report)return report;
        report=spin_detail::Parent(in,in.bindings->source().data().parents[selected.parents[i]],selected.locals[i],true,next.candidate_parents[i]);if(!report)return report;
        const auto c=next.parents[i].force.internal_couple[selected.locals[i]];
        next.negative_parent_couple_sum.x+=-c.x;next.negative_parent_couple_sum.y+=-c.y;next.negative_parent_couple_sum.z+=-c.z;
    }
    const auto a=next.applied_couple,b=next.negative_parent_couple_sum;
    next.assembly_couple_residual={a.x-b.x,a.y-b.y,a.z-b.z};
    if(!Finite(b)||!Finite(next.assembly_couple_residual))return {Status::NonfiniteResult,"Spin couple attribution overflows"};
    *output=next;return detail::Success();
}
}
