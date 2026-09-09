#pragma once
#include "ResponseData.h"

namespace tl::qualification::qeph::response::detail {
// One field-order definition serves the dictionary, observation and reader.
// Names are constructed only while the immutable dictionary is initialized.
template<class Emit>
void Visit(const Model& m,const State& s,const Results& results,
           const std::array<double,18>& vs,const std::array<double,18>& ws,
           const std::array<double,18>& angle,Emit emit) {
  const double moment=Young*Thickness*Thickness*Thickness*Theta/Side;
  const double force=BendingScale()*Delta/(Side*Side),couple=BendingScale()*Theta;
  for(unsigned n=0;n<m.nodes;++n) {
    for(unsigned a=0;a<3;++a) emit("node.position",n,a,s.x[3*n+a],"m",Delta,true);
    for(unsigned a=0;a<4;++a) emit("node.quaternion_wxyz",n,a,s.q[4*n+a],"1",1.,false);
    for(unsigned a=0;a<3;++a) emit("node.carried_velocity",n,a,s.v[3*n+a],"m/s",1.,false);
    for(unsigned a=0;a<3;++a) emit("node.carried_omega",n,a,s.omega[3*n+a],"rad/s",1.,false);
    for(unsigned a=0;a<3;++a) emit("node.synchronous_velocity",n,a,vs[3*n+a],"m/s",Delta/Pulse,true);
    for(unsigned a=0;a<3;++a) emit("node.synchronous_omega",n,a,ws[3*n+a],"rad/s",Theta/Pulse,true);
    for(unsigned a=0;a<3;++a) emit("node.rotation_vector",n,a,angle[3*n+a],"rad",Theta,true);
  }
  for(unsigned e=0;e<m.cells;++e) {
    const auto& r=results[e]; const auto& h=r.proposed_history.data(); const auto& k=r.kinematics;
    for(unsigned i=0;i<5;++i) emit("element.total_stress",e,i,h.stress[i],"Pa",Young*Theta,true);
    for(unsigned i=0;i<5;++i) emit("element.material_stress",e,i,h.material_stress[i],"Pa",Young*Theta,true);
    for(unsigned i=0;i<3;++i) emit("element.bending_stress",e,i,h.bending_stress[i],"Pa",moment/(Thickness*Thickness),true);
    for(unsigned i=0;i<12;++i) {
      const bool inverse_length=i==2||i==3||i==8||i==9;
      emit("element.hourglass",e,i,h.stabilization[i],inverse_length?"Pa/m":"Pa",Young*Theta/(inverse_length?Side:1.),true);
    }
    for(unsigned i=0;i<8;++i) emit("element.strain_curvature",e,i,h.strain_curvature[i],i<5?"1":"1/m",Theta/(i<5?1.:Side),true);
    emit("element.reported_thickness",e,0,h.thickness,"m",Thickness*Theta,true);
    for(unsigned i=0;i<2;++i) emit("element.internal_work",e,i,h.internal_work[i],"J",1.,false);
    emit("element.viscous_work",e,0,h.hourglass_viscous_work,"J",1.,false);
    emit("element.active",e,0,h.active,"1",1.,false);
    for(unsigned i=0;i<4;++i) {
      const double f[]{r.internal_force[i].x,r.internal_force[i].y,r.internal_force[i].z};
      const double c[]{r.internal_couple[i].x,r.internal_couple[i].y,r.internal_couple[i].z};
      for(unsigned a=0;a<3;++a) emit("element.cached_force",e,3*i+a,f[a],"N",force,true);
      for(unsigned a=0;a<3;++a) emit("element.cached_couple",e,3*i+a,c[a],"N*m",couple,true);
    }
    const auto& d=r.diagnostics;
    const double diag[]{d.effective_thickness,d.native_sound_speed,d.membrane_viscosity,d.stabilization_viscosity,
      d.translational_stiffness,d.rotational_stiffness,d.unscaled_element_dt,d.internal_work_increment[0],
      d.internal_work_increment[1],d.hourglass_viscous_work_increment};
    const char* units[]{"m","m/s","1","1","N/m","N*m","s","J","J","J"};
    for(unsigned i=0;i<10;++i) emit("element.force_diagnostic",e,i,diag[i],units[i],1.,false);
    for(unsigned i=0;i<9;++i) emit("element.frame",e,i,k.frame.v[i],"1",1.,false);
    const double geom[]{k.area,k.reciprocal_area,k.characteristic_length,k.nodal_factors[0],k.nodal_factors[1],k.raw_warpage_abs,k.effective_warpage};
    const char* geom_units[]{"m^2","1/m^2","m","1","1","m","m"};
    for(unsigned i=0;i<7;++i) emit("element.geometry",e,i,geom[i],geom_units[i],1.,false);
    emit("element.planar",e,0,k.planar,"1",1.,false);
    for(unsigned i=0;i<4;++i) {
      const double p[]{k.local_position[i].x,k.local_position[i].y,k.local_position[i].z};
      const double n[]{k.local_normals[i].x,k.local_normals[i].y,k.local_normals[i].z};
      const double b[]{k.projection_columns[i].x,k.projection_columns[i].y,k.projection_columns[i].z};
      for(unsigned a=0;a<3;++a) emit("element.local_position",e,3*i+a,p[a],"m",1.,false);
      for(unsigned a=0;a<3;++a) emit("element.local_normal",e,3*i+a,n[a],"1",1.,false);
      for(unsigned a=0;a<3;++a) emit("element.projection_column",e,3*i+a,b[a],"native-coordinate",1.,false);
    }
    for(unsigned i=0;i<6;++i) emit("element.projection_inverse",e,i,k.projection_inverse[i],"native-coordinate",1.,false);
    for(unsigned i=0;i<8;++i) emit("element.projected_omega",e,i,k.projected_omega[i],"rad/s",1.,false);
    for(unsigned i=0;i<8;++i) emit("element.regular_rate",e,i,k.regular_rate[i],i<5?"1/s":"1/(m*s)",1.,false);
    for(unsigned i=0;i<6;++i) emit("element.hourglass_rate",e,i,k.hourglass_rate[i],i==2||i==3?"1/s":"m/s",1.,false);
  }
}
} // namespace tl::qualification::qeph::response::detail
