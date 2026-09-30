#include "AnalysisInternal.h"
namespace crash::benchmarks::assembly_spin {
namespace {
void Scalar(Metrics& metrics,const std::string& name,const char* units,double expected,double actual,std::uint64_t source,std::size_t i=0) {
    auto& m=metrics[name];m.units=units;m.Add(expected,actual,i,source);
}
template<class A> void Values(Metrics& m,const Value& json,const char* key,const A& a,std::size_t n,const char* units,std::uint64_t id) {
    const auto& actual=Numbers(json,key,n);for(std::size_t i=0;i<n;++i)Scalar(m,key,units,a[i],json::Real(actual[static_cast<unsigned>(i)]),id,i);
}
void Nodes(Metrics& m,const Value& json,const char* key,const std::array<native::Vec3,4>& nodes,const char* units,std::uint64_t id) {
    std::array<double,12> values;for(unsigned i=0;i<4;++i){values[3*i]=nodes[i].x;values[3*i+1]=nodes[i].y;values[3*i+2]=nodes[i].z;}
    Values(m,json,key,values,12,units,id);
}
}
void Compare(Metrics& m,const ParentSource& p,const Value& row,const layered::QephTrial& expected) {
    const auto id=p.parent->source_id;const auto& force=expected.shell;
    Nodes(m,row,"positive_internal_force_xyz_N",force.internal_force,"N",id);Nodes(m,row,"positive_internal_couple_xyz_N_m",force.internal_couple,"N m",id);
    const auto& h=json::Member(row,"retained_history");const auto& v=force.proposed_history.data();
    Values(m,h,"FOR_Pa",v.stress,5,"Pa",id);Values(m,h,"FOR_G_Pa",v.material_stress,5,"Pa",id);Values(m,h,"MOM_Pa",v.bending_stress,3,"Pa",id);
    const auto& hg=Numbers(h,"HOURG_native_mixed_units",12);for(unsigned i=0;i<12;++i) {
        const bool bending=i==2||i==3||i==8||i==9;
        Scalar(m,bending?"HOURG_bending":"HOURG_other",bending?"Pa/m":"Pa",v.stabilization[i],json::Real(hg[i]),id,i);
    }
    const auto& strain=Numbers(h,"STRA_native",8);for(unsigned i=0;i<8;++i)
        Scalar(m,i<5?"STRA_strain":"STRA_curvature",i<5?"1":"1/m",v.strain_curvature[i],json::Real(strain[i]),id,i);
    Values(m,h,"internal_work_J",v.internal_work,2,"J",id);
    Scalar(m,"reported_thickness_m","m",v.thickness,json::Real(h,"reported_thickness_m"),id);
    Scalar(m,"hourglass_viscous_work_J","J",v.hourglass_viscous_work,json::Real(h,"hourglass_viscous_work_J"),id);
    Scalar(m,"active","1",v.active,json::Real(h,"active"),id);
    const auto& k=json::Member(row,"retained_kinematics");const auto& a=force.kinematics;
    Values(m,k,"frame_columns_row_major",a.frame.v,9,"1",id);Nodes(m,k,"local_position_m",a.local_position,"m",id);Nodes(m,k,"local_normals",a.local_normals,"1",id);
    Values(m,k,"projected_omega_rad_s",a.projected_omega,8,"rad/s",id);
    Values(m,k,"projection_inverse_native",a.projection_inverse,6,"native mixed",id);Nodes(m,k,"projection_columns_native",a.projection_columns,"native mixed",id);
    Values(m,k,"nodal_factors",a.nodal_factors,2,"1",id);
    const auto& rate=Numbers(k,"regular_rate_native",8);for(unsigned i=0;i<8;++i)
        Scalar(m,i<5?"regular_membrane_shear_rate":"regular_curvature_rate",i<5?"1/s":"1/(m s)",a.regular_rate[i],json::Real(rate[i]),id,i);
    const auto& hr=Numbers(k,"hourglass_rate_native",6);for(unsigned i=0;i<6;++i)
        Scalar(m,i==2||i==3?"hourglass_bending_rate":"hourglass_other_rate",i==2||i==3?"1/s":"m/s",a.hourglass_rate[i],json::Real(hr[i]),id,i);
    const char* key[]{"area_m2","reciprocal_area_per_m2","characteristic_length_m","raw_warpage_abs_m","effective_warpage_m"};
    const char* unit[]{"m2","1/m2","m","m","m"};const double value[]{a.area,a.reciprocal_area,a.characteristic_length,a.raw_warpage_abs,a.effective_warpage};
    for(unsigned i=0;i<5;++i)Scalar(m,key[i],unit[i],value[i],json::Real(k,key[i]),id);
    json::Flag(k,"planar",a.planar);
    const auto& diag=json::Member(row,"retained_force_diagnostics");const auto& d=force.diagnostics;
    const char* dk[]{"effective_thickness_m","native_sound_speed_m_s","membrane_viscosity_native","stabilization_viscosity_native",
        "translational_stiffness_N_per_m","rotational_stiffness_N_m_per_rad","unscaled_element_dt_s","hourglass_viscous_work_increment_J"};
    const char* du[]{"m","m/s","native","native","N/m","N m","s","J"};
    const double dv[]{d.effective_thickness,d.native_sound_speed,d.membrane_viscosity,d.stabilization_viscosity,
        d.translational_stiffness,d.rotational_stiffness,d.unscaled_element_dt,d.hourglass_viscous_work_increment};
    for(unsigned i=0;i<8;++i)Scalar(m,dk[i],du[i],dv[i],json::Real(diag,dk[i]),id);
    Values(m,diag,"internal_work_increment_J",d.internal_work_increment,2,"J",id);
    const auto& points=json::Array(row,"points",3,3);for(unsigned l=0;l<3;++l)for(unsigned i=0;i<7;++i)
        Scalar(m,i<5?"point_stress":i==5?"point_PLA":"point_filtered_rate",i<5?"Pa":i==5?"1":"1/s",expected.points[l][i],json::Real(points[l][i]),id,7*l+i);
}
}
