// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Rig.h"
#include "../solid_resident/ResultValues.h"
#include "../law90_solid18_force/TestSupport.h"
#include <cstring>
#include <iomanip>
#include <ostream>
namespace controlled_resident_test::snapshot {
struct Writer {
  std::ostream& out;std::string prefix;
  void U(const std::string& name,std::uint64_t value){out<<prefix<<'.'<<name<<" U "<<std::hex<<std::setw(16)<<std::setfill('0')<<value<<std::dec<<'\n';}
  void D(const std::string& name,double value){std::uint64_t bits;std::memcpy(&bits,&value,sizeof(bits));out<<prefix<<'.'<<name<<" D "<<std::hex<<std::setw(16)<<std::setfill('0')<<bits<<std::dec<<'\n';}
  template<class Values>void Series(const std::string& name,const Values& values){unsigned n=0;for(auto value:values)D(name+'.'+std::to_string(n++),value);}
};
template<class H>void Material(Writer& w,const std::string& name,const H& value){
  w.Series(name+".stress",value.stress_pa);w.Series(name+".rho_eint_qvis",std::array<double,3>{value.density_kg_m3,value.internal_energy_density_j_m3,value.bulk_pressure_pa});
}
inline void Global(Writer& w,const std::string& name,const fe::solid18::total_strain::GlobalHistory& value){Material(w,name,value);w.D(name+".scalar_rate",value.scalar_rate_per_s);}
template<class R>void Cache(Writer& w,const R& r){
  unsigned n=0;for(auto force:r.cache.rhs_force_n){w.Series("force."+std::to_string(n++),std::array<double,3>{force.x,force.y,force.z});}
  w.Series("stiffness",std::array<double,2>{r.cache.stiffness.translation_n_m,r.cache.stiffness.rotation_nm});
  w.U("epoch",r.stamp.sample_index);w.D("time",r.stamp.time_s);
}
template<class R>void Controlled(Writer& w,const R& r){
  Cache(w,r);const auto& u=r.history_units;w.Series("native_units",std::array<double,3>{u.length_m,u.mass_kg,u.time_s});
  w.U("history_profile",unsigned(r.history.profile()));const auto& d=r.cache.response;
  w.Series("response",std::array<double,8>{d.material_raw_stiffness_n_m,d.hourglass_raw_stiffness_n_m,d.after_distortion_raw_stiffness_n_m,
    d.material_dt_s,d.material_work_j,d.hourglass_work_j,d.distortion_energy_j,d.distortion_work_j});
  w.Series("response_units",std::array<double,3>{d.units.length_m,d.units.mass_kg,d.units.time_s});
}
inline void H24(Writer& w,const s::ProfiledResult24& r){
  Controlled(w,r);const auto* h=r.history.native();ASSERT_NE(h,nullptr);
  w.Series("native_history",solid_resident_test::RubberHistory(h->values.material,h->values.controlled_hourglass.force_n));
  w.D("native_distortion_energy",h->distortion_energy);Material(w,"si_material",r.material_si);
}
inline void Foam(Writer& w,const s::ProfiledResult18Law90& r){
  Controlled(w,r);const auto* h=r.history.native();ASSERT_NE(h,nullptr);
  Global(w,"native_global",h->values.global);Global(w,"si_global",r.global_si);
  w.D("native_distortion_energy",h->distortion_energy);
  for(unsigned ip=0;ip<8;++ip){const auto& p=h->values.point[ip];const auto name="point."+std::to_string(ip);
    std::array<double,20> values{};law90_force_test::PackHistory(p,values.data());w.Series(name+".native_history",values);
    for(unsigned k=0;k<3;++k)w.U(name+".cursor."+std::to_string(k),p.point.cursor[k]);
    Material(w,name+".si_material",r.material_si[ip]);}
}
inline void Diagnostics(Writer& w,const s::BatchDiagnostics& d){
#define UFIELD(name) w.U(#name,d.name)
  UFIELD(source_instance_id);UFIELD(owner_id);UFIELD(configuration_id);UFIELD(qualification_id);UFIELD(epoch);UFIELD(base_epoch);UFIELD(attempt);
  UFIELD(valid);UFIELD(has_completed_interval);UFIELD(accepted_force_assembled);w.U("phase",unsigned(d.phase));
#undef UFIELD
  w.Series("phase_times",std::array<double,5>{d.time,d.base_time,d.velocity_time,d.base_velocity_time,d.kick_dt});
  for(unsigned i=0;i<5;++i)w.U("parent_count."+std::to_string(i),d.parent_count[i]);
  w.Series("native_work",d.native_internal_work_increment_j);w.Series("hourglass_work",d.physical_hourglass_work_increment_j);
  w.Series("distortion_work",d.distortion_work_increment_j);
  w.Series("other_work_dt",std::array<double,4>{d.plastic_work_increment_j,d.internal_kick_work_j,d.internal_drift_work_j,d.minimum_native_dt_s});
}
inline void Emit(std::ostream& out,const std::string& sample,const Results& r,const s::BatchDiagnostics& diagnostics){
  for(unsigned p=0;p<r.h24.size();++p){Writer w{out,sample+".h24."+std::to_string(p)};H24(w,r.h24[p]);}
  for(unsigned p=0;p<r.foam.size();++p){Writer w{out,sample+".foam."+std::to_string(p)};Foam(w,r.foam[p]);}
  auto legacy=[&](const auto& rows,const char* name){for(unsigned p=0;p<rows.size();++p){Writer w{out,sample+'.'+name+'.'+std::to_string(p)};Cache(w,rows[p]);}};
  legacy(r.old18,"law36");legacy(r.old6z,"s6");legacy(r.rear,"law44");Writer w{out,sample+".diagnostics"};Diagnostics(w,diagnostics);
}
} // namespace controlled_resident_test::snapshot
