#include "AnalysisInternal.h"
#include <cmath>
namespace crash::benchmarks::assembly_spin {
namespace {
native::Vec3 Node(const Value& p,const char* key,std::size_t local) {
    const auto& a=Numbers(p,key,12);const auto i=static_cast<unsigned>(3*local);
    return {json::Real(a[i]),json::Real(a[i+1]),json::Real(a[i+2])};
}
double Dot(native::Vec3 a,native::Vec3 b){return a.x*b.x+a.y*b.y+a.z*b.z;}
double Norm(native::Vec3 a){return std::hypot(std::hypot(a.x,a.y),a.z);}
native::Vec3 Normal(const Value& p,std::size_t local) {
    const auto& k=json::Member(p,"retained_kinematics");const auto n=Node(k,"local_normals",local);const auto& m=Numbers(k,"frame_columns_row_major",9);
    return {json::Real(m[0u])*n.x+json::Real(m[1u])*n.y+json::Real(m[2u])*n.z,
            json::Real(m[3u])*n.x+json::Real(m[4u])*n.y+json::Real(m[5u])*n.z,
            json::Real(m[6u])*n.x+json::Real(m[7u])*n.y+json::Real(m[8u])*n.z};
}
}
Document LocalDiagnostic(const Context& c,const ParentSource& p,const Value& row,const Value& base,const Value& candidate,const layered::QephTrial& native_actual) {
    Document d;d.SetObject();const auto& stamp=json::Member(row,"enclosing_accepted_stamp");
    output::Integer(d,"enclosing_epoch",json::Unsigned(stamp,"epoch"));output::Number(d,"enclosing_time_s",json::Real(stamp,"time"));
    output::Integer(d,"source_parent_id",p.parent->source_id);output::Integer(d,"local_node",p.local);
    const auto w=Node(candidate,"omega_previous_midpoint_xyz_rad_s",p.local),couple=Node(candidate,"positive_internal_couple_xyz_N_m",p.local),n=Normal(candidate,p.local);
    const double spin=Norm(w),normal=Dot(w,n),torque=Dot(couple,n);
    const native::Vec3 tangent{w.x-normal*n.x,w.y-normal*n.y,w.z-normal*n.z};
    output::Number(d,"carried_spin_norm_rad_s",spin);output::Number(d,"normal_spin_rad_s",normal);
    output::Number(d,"normal_squared_spin_fraction",spin?normal*normal/(spin*spin):0);
    output::Number(d,"normal_internal_couple_N_m",torque);output::Number(d,"internal_couple_norm_N_m",Norm(couple));
    output::Number(d,"normal_carried_internal_power_W",normal*torque);output::Number(d,"tangent_carried_internal_power_W",Dot(couple,tangent));
    output::Number(d,"total_carried_internal_power_W",Dot(couple,w));
    const auto& metric=Numbers(row,"native",4);
    output::Number(d,"enclosing_carried_velocity_time_s",json::Real(stamp,"velocity_time"));
    output::Number(d,"native_isotropic_node_carried_rotational_K_J",.5*json::Real(metric[1u])*spin*spin);
    output::Number(d,"added_isotropic_node_carried_rotational_K_J",.5*json::Real(metric[3u])*spin*spin);
    // Frozen-state native sensitivity only. The actual trace and source are
    // immutable. Remove one parent's own projected normal component, retaining
    // exact x/v, other spins and old history. This is not an alternate trajectory.
    auto input=Interval(candidate,c.dt);auto& changed=input.omega_midpoint[p.local];changed=tangent;
    const auto old=History(p.reference,base);layered::QephTrial control;
    const auto status=layered::Evaluate(p.reference,old,input,p.law,control);
    output::Integer(d,"own_normal_projection_control_native_status",static_cast<unsigned>(status));
    if(status==native::Status::kSuccess) {
        const auto& actual=native_actual.shell;const auto& h=control.shell.proposed_history.data();
        double df=0,dm=0;for(unsigned i=0;i<4;++i){const auto f=control.shell.internal_force[i],m=control.shell.internal_couple[i];
            const auto af=actual.internal_force[i],am=actual.internal_couple[i];
            df=std::max(df,std::hypot(std::hypot(f.x-af.x,f.y-af.y),f.z-af.z));
            dm=std::max(dm,std::hypot(std::hypot(m.x-am.x,m.y-am.y),m.z-am.z));}
        output::Number(d,"own_normal_control_max_force_difference_N",df);output::Number(d,"own_normal_control_max_couple_difference_N_m",dm);
        const auto& ah=actual.proposed_history.data();
        output::Number(d,"own_normal_control_native_work_difference_J",static_cast<double>(
            (static_cast<long double>(h.internal_work[0])-ah.internal_work[0])+
            (static_cast<long double>(h.internal_work[1])-ah.internal_work[1])));
        output::Number(d,"own_normal_control_viscous_work_difference_J",h.hourglass_viscous_work-actual.proposed_history.data().hourglass_viscous_work);
    }
    const auto& k=json::Member(candidate,"retained_kinematics");output::Number(d,"effective_warpage_m",json::Real(k,"effective_warpage_m"));
    const auto& history=json::Member(candidate,"retained_history");const auto& old_history=json::Member(base,"retained_history");
    const auto& work=Numbers(history,"internal_work_J",2);output::FiniteArray(d,"native_internal_work_J",std::array<double,2>{json::Real(work[0u]),json::Real(work[1u])}.data(),2);
    output::Number(d,"native_viscous_work_J",json::Real(history,"hourglass_viscous_work_J"));
    output::Number(d,"native_viscous_work_increment_J",json::Real(history,"hourglass_viscous_work_J")-json::Real(old_history,"hourglass_viscous_work_J"));
    output::Number(d,"cumulative_plastic_work_J",json::Real(candidate,"cumulative_plastic_work_J"));
    double pla=0;for(const auto& point:json::Array(candidate,"points",3,3).GetArray())pla=std::max(pla,json::Real(point[5u]));
    output::Number(d,"maximum_point_PLA",pla);return d;
}
}
