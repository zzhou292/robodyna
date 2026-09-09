#include "ResponseSampleValidation.h"
#include "ResponseSamples.h"
#include "lib_src/elements/qeph/QephHistory.h"
#include <algorithm>
#include <cmath>

namespace tl::qualification::qeph::response {
bool SameSample(const Sample& a,const Sample& b) noexcept {
  return a.epoch==b.epoch&&a.time==b.time&&a.carried_velocity_time==b.carried_velocity_time&&a.kick_dt==b.kick_dt&&
    a.interval_available==b.interval_available&&a.values==b.values&&a.carried_kinetic==b.carried_kinetic&&
    a.synchronous_kinetic==b.synchronous_kinetic&&a.source_work==b.source_work&&a.external_work==b.external_work&&a.residual==b.residual;
}
bool ValidateSampleFields(const Run& run,const Sample& sample,std::string& error) {
  // This verifies retained observation arithmetic, not omitted physical steps.
  // PreparePrescribedHistory packages observed values; it evaluates no force.
  bool found=true;
  auto value=[&](const char* group,unsigned entity,unsigned component) {
    const std::string name=std::string(group)+"["+std::to_string(entity)+"]["+std::to_string(component)+"]";
    for(unsigned i=0;i<run.fields.size();++i) if(run.fields[i].name==name) return sample.values[i];
    found=false; return 0.;
  };
  State state; Results results;
  for(unsigned n=0;n<run.model.nodes;++n) {
    for(unsigned a=0;a<3;++a) {
      state.x[3*n+a]=value("node.position",n,a); state.v[3*n+a]=value("node.carried_velocity",n,a);
      state.omega[3*n+a]=value("node.carried_omega",n,a);
    }
    for(unsigned a=0;a<4;++a) state.q[4*n+a]=value("node.quaternion_wxyz",n,a);
  }
  for(unsigned e=0;e<run.model.cells;++e) {
    port::HistoryValues h;
    for(unsigned i=0;i<5;++i) { h.stress[i]=value("element.total_stress",e,i); h.material_stress[i]=value("element.material_stress",e,i); }
    for(unsigned i=0;i<3;++i) h.bending_stress[i]=value("element.bending_stress",e,i);
    for(unsigned i=0;i<12;++i) h.stabilization[i]=value("element.hourglass",e,i);
    for(unsigned i=0;i<8;++i) h.strain_curvature[i]=value("element.strain_curvature",e,i);
    h.thickness=value("element.reported_thickness",e,0); h.active=value("element.active",e,0);
    for(unsigned i=0;i<2;++i) h.internal_work[i]=value("element.internal_work",e,i);
    h.hourglass_viscous_work=value("element.viscous_work",e,0);
    if(port::PreparePrescribedHistory(run.model.reference[e],h,{sample.time,sample.epoch},results[e].proposed_history)!=port::Status::kSuccess) {
      error="Invalid retained source history"; return false;
    }
    for(unsigned i=0;i<4;++i) {
      results[e].internal_force[i]={value("element.cached_force",e,3*i),value("element.cached_force",e,3*i+1),value("element.cached_force",e,3*i+2)};
      results[e].internal_couple[i]={value("element.cached_couple",e,3*i),value("element.cached_couple",e,3*i+1),value("element.cached_couple",e,3*i+2)};
    }
    results[e].kinematics.area=value("element.geometry",e,0);
  }
  Sample expected; Limits limits;
  if(!found||!Observe(run.model,H0/run.config.refinement,sample.epoch,state,results,sample.external_work,expected,limits,error)) return false;
  if(sample.carried_kinetic!=expected.carried_kinetic||sample.synchronous_kinetic!=expected.synchronous_kinetic||sample.source_work!=expected.source_work) {
    error="Retained fields do not reproduce global kinetic/source-work partitions"; return false;
  }
  for(unsigned i=0;i<run.fields.size();++i) {
    const auto& name=run.fields[i].name;
    if(name.rfind("node.synchronous_",0)==0||name.rfind("node.rotation_vector",0)==0)
      if(sample.values[i]!=expected.values[i]) { error="Retained endpoint reconstruction mismatch"; return false; }
  }
  const auto& a=run.observed;
  if(a.displacement_over_side<limits.displacement_over_side||a.rotation_angle<limits.rotation_angle||a.strain<limits.strain||
      a.thickness_curvature<limits.thickness_curvature||a.minimum_area_ratio>limits.minimum_area_ratio||a.maximum_area_ratio<limits.maximum_area_ratio||
      a.minimum_thickness_ratio>limits.minimum_thickness_ratio||a.maximum_thickness_ratio<limits.maximum_thickness_ratio) {
    error="All-endpoint summary omits a retained sample bound"; return false;
  }
  if(sample.epoch==0) {
    if(state.x!=run.model.initial_position) { error="Initial geometry changed"; return false; }
    for(unsigned n=0;n<run.model.nodes;++n) {
      for(unsigned c=0;c<3;++c) if(state.v[3*n+c]!=0||state.omega[3*n+c]!=0) { error="Initial state is not rest"; return false; }
      if(state.q[4*n]!=1||state.q[4*n+1]!=0||state.q[4*n+2]!=0||state.q[4*n+3]!=0) { error="Initial orientation changed"; return false; }
    }
    if(sample.external_work!=0) { error="Nonzero initial external work"; return false; }
    for(unsigned e=0;e<run.model.cells;++e) {
      const auto& h=results[e].proposed_history.data();
      for(double x:h.stress) if(x!=0) { error="Initial total stress is not zero"; return false; }
      for(double x:h.material_stress) if(x!=0) { error="Initial material stress is not zero"; return false; }
      for(double x:h.bending_stress) if(x!=0) { error="Initial moment is not zero"; return false; }
      for(double x:h.stabilization) if(x!=0) { error="Initial stabilization is not zero"; return false; }
      for(double x:h.strain_curvature) if(x!=0) { error="Initial strain/curvature is not zero"; return false; }
      if(h.thickness!=Thickness||h.active!=1||h.internal_work[0]!=0||h.internal_work[1]!=0||h.hourglass_viscous_work!=0) {
        error="Initial source thickness/work/activity changed"; return false;
      }
      for(unsigned i=0;i<4;++i) {
        const auto f=results[e].internal_force[i],c=results[e].internal_couple[i];
        if(f.x!=0||f.y!=0||f.z!=0||c.x!=0||c.y!=0||c.z!=0) { error="Initial force/couple cache is not zero"; return false; }
      }
    }
  }
  return true;
}
} // namespace tl::qualification::qeph::response
