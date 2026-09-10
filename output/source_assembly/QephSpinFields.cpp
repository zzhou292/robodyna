#include "QephSpinTrace.h"
#include "SourceAssemblyWallFields.h"
#include "WallFieldValues.h"
#include "case/source_assembly_observation/SourceAssemblyObservationInternal.h"
#include "lib_src/solvers/NodalTrialIdentity.h"

namespace crash::output::assembly {
namespace {
using namespace wall_fields;
void Vector(Document& d,const char* name,tl::math::Vec3 v){const double a[]{v.x,v.y,v.z};FiniteArray(d,name,a,3);}
template<std::size_t N> void Vectors(Document& d,const char* name,const std::array<tl::math::Vec3,N>& v) {
    double a[3*N];for(std::size_t n=0;n<N;++n){a[3*n]=v[n].x;a[3*n+1]=v[n].y;a[3*n+2]=v[n].z;}FiniteArray(d,name,a,3*N);
}
void Vectors(Document& d,const char* name,const tl::math::Vec3 (&v)[4]) {
    std::array<tl::math::Vec3,4> a;for(unsigned n=0;n<4;++n)a[n]=v[n];Vectors(d,name,a);
}
Document Parent(const cases::source_assembly_observation::QephSpinParent& p) {
    Document d;d.SetObject();Integer(d,"source_element_id",p.source_parent);Integer(d,"source_part_id",p.source_part);
    Integer(d,"source_material_id",p.source_material);Integer(d,"source_section_id",p.source_section);Integer(d,"source_curve_id",p.source_curve);
    Integer(d,"source_parent_index",p.source_index);Integer(d,"family_index",p.family_index);Integer(d,"local_node",p.local_node);
    Value ids(rapidjson::kArrayType);for(auto id:p.source_nodes)ids.PushBack(id,d.GetAllocator());Put(d,"source_nodes",std::move(ids));
    Vectors(d,"position_endpoint_xyz_m",p.position);Vectors(d,"velocity_previous_midpoint_xyz_m_s",p.velocity);
    Vectors(d,"omega_previous_midpoint_xyz_rad_s",p.omega);
    const auto& f=p.force;const auto& h=f.proposed_history.data();const auto& k=f.kinematics;
    Vectors(d,"positive_internal_force_xyz_N",f.internal_force);Vectors(d,"positive_internal_couple_xyz_N_m",f.internal_couple);
    Document hist;hist.SetObject();Number(hist,"time_s",f.proposed_history.stamp().time);Integer(hist,"epoch",f.proposed_history.stamp().sample_index);
    FiniteArray(hist,"FOR_Pa",h.stress,5);FiniteArray(hist,"FOR_G_Pa",h.material_stress,5);FiniteArray(hist,"MOM_Pa",h.bending_stress,3);
    FiniteArray(hist,"HOURG_native_mixed_units",h.stabilization,12);FiniteArray(hist,"STRA_native",h.strain_curvature,8);
    Number(hist,"reported_thickness_m",h.thickness);FiniteArray(hist,"internal_work_J",h.internal_work,2);
    Number(hist,"hourglass_viscous_work_J",h.hourglass_viscous_work);Number(hist,"active",h.active);Child(d,"retained_history",hist);
    Document rates;rates.SetObject();Boolean(rates,"available",p.has_native_kinematics);
    if(p.has_native_kinematics) {
        Number(rates,"origin_base_time_s",k.base_time);Number(rates,"origin_dt_s",k.dt);Integer(rates,"origin_endpoint_epoch",k.sample_index);
        FiniteArray(rates,"frame_columns_row_major",k.frame.v,9);Vectors(rates,"local_position_m",k.local_position);Vectors(rates,"local_normals",k.local_normals);
        Number(rates,"area_m2",k.area);Number(rates,"raw_warpage_abs_m",k.raw_warpage_abs);Number(rates,"effective_warpage_m",k.effective_warpage);
        Boolean(rates,"planar",k.planar);FiniteArray(rates,"projected_omega_rad_s",k.projected_omega,8);
        FiniteArray(rates,"regular_rate_native",k.regular_rate,8);FiniteArray(rates,"hourglass_rate_native",k.hourglass_rate,6);
        FiniteArray(rates,"projection_inverse_native",k.projection_inverse,6);Vectors(rates,"projection_columns_native",k.projection_columns);
        Vector(rates,"selected_native_normal_world",p.native_normal);Number(rates,"normal_spin_rad_s",p.normal_spin);Number(rates,"tangent_spin_rad_s",p.tangent_spin);
        Number(rates,"normal_positive_internal_couple_N_m",p.normal_internal_couple);
        Number(rates,"normal_carried_internal_power_W",p.normal_internal_power);Number(rates,"tangent_carried_internal_power_W",p.tangent_internal_power);
    }
    Child(d,"retained_kinematics",rates);
    Document diag;diag.SetObject();const auto& a=f.diagnostics;
    Number(diag,"effective_thickness_m",a.effective_thickness);Number(diag,"native_sound_speed_m_s",a.native_sound_speed);
    Number(diag,"membrane_viscosity_native",a.membrane_viscosity);Number(diag,"stabilization_viscosity_native",a.stabilization_viscosity);
    Number(diag,"translational_stiffness_N_per_m",a.translational_stiffness);Number(diag,"rotational_stiffness_N_m_per_rad",a.rotational_stiffness);
    Number(diag,"unscaled_element_dt_s",a.unscaled_element_dt);FiniteArray(diag,"internal_work_increment_J",a.internal_work_increment,2);
    Number(diag,"hourglass_viscous_work_increment_J",a.hourglass_viscous_work_increment);Child(d,"retained_force_diagnostics",diag);
    Value points(rapidjson::kArrayType);for(const auto& pt:p.section.history.point) {
        const double row[]{pt.stress[0],pt.stress[1],pt.stress[2],pt.stress[3],pt.stress[4],pt.plastic_strain,pt.filtered_rate_per_s};
        points.PushBack(FiniteArray(d,row,7),d.GetAllocator());
    }
    String(d,"point_columns","XX_Pa,YY_Pa,XY_Pa,YZ_Pa,ZX_Pa,PLA,filtered_rate_per_s");Put(d,"points",std::move(points));
    Number(d,"cumulative_plastic_work_J",p.section.cumulative_plastic_work_J);return d;
}
}
Document QephSpinDocument(const cases::source_assembly_observation::QephSpinObservation& v) {
    Require(v.completed&&v.parent_count&&v.parent_count<=cases::source_assembly_observation::MaxSpinParents,
            "Spin record has no completed incident-parent observation");
    Require(v.base.epoch!=UINT64_MAX&&v.enclosing.owner_id==v.base.owner_id&&v.enclosing.epoch==v.base.epoch+1&&
        v.enclosing.time==v.base.time+v.base.fixed_dt&&v.enclosing.reaction_base_epoch==v.base.epoch&&
        v.enclosing.reaction_time==v.base.time,"Spin record enclosing acceptance has a different phase");
    namespace observation=cases::source_assembly_observation;
    tl::fea::rigid::ObservationPhase base_phase,enclosing_phase;
    Require(bool(observation::detail::AcceptedPhase(v.base,base_phase))&&
        bool(observation::detail::AcceptedPhase(v.enclosing,enclosing_phase))&&v.attempt&&v.source_instance&&v.source_node&&
        v.base.node_count==v.enclosing.node_count&&v.base.fixed_dt==v.enclosing.fixed_dt&&
        tl::fea::SameRigidGroupInfo(v.base.rigid_groups,v.enclosing.rigid_groups)&&v.global_node<v.base.node_count,
        "Spin record complete stamp/source association is invalid");
    Document d;d.SetObject();String(d,"record","accepted_force_stage");Integer(d,"attempt",v.attempt);
    Child(d,"base_stamp",StampDocument(v.base));Child(d,"enclosing_accepted_stamp",StampDocument(v.enclosing));
    Integer(d,"source_instance_id",v.source_instance);Integer(d,"source_node_id",v.source_node);Integer(d,"global_node",v.global_node);
    Integer(d,"complete_incident_qeph_parent_count",v.parent_count);Integer(d,"incident_t3_parent_count",0);
    String(d,"phase","retained_accepted_force_at_base_time_with_carried_midpoint_motion");
    Vector(d,"position_endpoint_m",v.position);Vector(d,"velocity_previous_midpoint_m_s",v.velocity);Vector(d,"omega_previous_midpoint_rad_s",v.omega);
    FiniteArray(d,"orientation_endpoint_wxyz",v.orientation.data(),4);Vector(d,"actual_assembled_force_N",v.applied_force);
    Vector(d,"actual_assembled_couple_N_m",v.applied_couple);Vector(d,"negative_parent_couple_sum_N_m",v.negative_parent_couple_sum);
    Vector(d,"assembly_couple_residual_N_m",v.assembly_couple_residual);
    const double mass[]{v.native.mass,v.native.isotropic_inertia,v.native.physical_inertia,v.native.added_inertia};
    String(d,"native_columns","mass_kg,total_J_kg_m2,physical_J_kg_m2,added_J_kg_m2");FiniteArray(d,"native",mass,4);
    Value parents(rapidjson::kArrayType);for(std::size_t i=0;i<v.parent_count;++i) {
        auto p=Parent(v.parents[i]);Value copy;copy.CopyFrom(p,d.GetAllocator());parents.PushBack(copy,d.GetAllocator());
    }
    Put(d,"parents",std::move(parents));Value candidate(rapidjson::kArrayType);
    for(std::size_t i=0;i<v.parent_count;++i) {
        auto p=Parent(v.candidate_parents[i]);Value copy;copy.CopyFrom(p,d.GetAllocator());candidate.PushBack(copy,d.GetAllocator());
    }
    Put(d,"enclosing_candidate_parents",std::move(candidate));return d;
}
}
