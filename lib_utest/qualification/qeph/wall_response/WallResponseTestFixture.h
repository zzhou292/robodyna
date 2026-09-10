#pragma once
#include "WallResponseData.h"
#include "lib_src/elements/qeph/QephHistory.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace tl::qualification::qeph::wall_response::test {
inline Config ConfigFor(unsigned cells,unsigned refinement=1) {
  return {cells,refinement,H0,std::string(64,'a')};
}
inline void RefreshContact(const Model& m,EndpointInput& out) {
  const auto& f=m.fields(); std::array<double,MaxNodes> inverse{}; std::array<std::uint8_t,MaxNodes> fixed{};
  for(unsigned n=0;n<f.nodes;++n)inverse[n]=1/f.mass[n];
  contact::NodalWallResult result;
  if(contact::EvaluateNodalWallContact(m.screened().weights(),{out.state.x.data(),f.nodes,3,1},
      {out.state.v.data(),f.nodes,3,1},{inverse.data(),fixed.data(),f.nodes,1,contact::TranslationMassModel::kIsotropicLumped},
      m.screened().law(),out.epoch+1,&result).status!=contact::NodalWallStatus::Ok)throw std::runtime_error("Synthetic contact refresh rejected");
  out.contact.node_count=f.nodes;out.contact.attempt=out.epoch+1;out.contact.base_epoch=out.epoch?out.epoch-1:0;out.contact.candidate=out.epoch!=0;
  out.contact.resultant=result.resultant;out.contact.potential=result.potential;
  for(unsigned n=0;n<f.nodes;++n) {out.contact.nodes[n]=result.nodes[n];out.contact.nodes[n].base_epoch=out.contact.base_epoch;out.contact.nodes[n].row.base_epoch=out.contact.base_epoch;}
}
// Synthetic observation data only. This packages a prescribed continuous
// spring sample; it neither integrates a trajectory nor calls native force.
// Depth scaling intentionally perturbs the recorded displacement independently
// of velocity for comparison/energy negative controls.
inline EndpointInput EndpointAt(const Model& m,const Config& config,std::uint64_t epoch,double depth_factor=1) {
  if(!m.prepared()||!ValidConfig(config)||epoch>Steps(config)||!std::isfinite(depth_factor)||depth_factor<=0)
    throw std::runtime_error("Invalid synthetic wall response endpoint");
  EndpointInput out; const auto& f=m.fields(); const double h=Step(config);
  out.epoch=epoch;out.time=epoch*h;out.carried_velocity_time=epoch?out.time-h/2:0;out.kick_dt=epoch?(epoch==1?h/2:h):0;
  out.state.x=f.initial_position;
  std::array<double,MaxNodes> inverse{}; std::array<std::uint8_t,MaxNodes> fixed{};
  long double impulse=0;
  for(unsigned n=0;n<f.nodes;++n) {
    const long double speed=wr::ImpactSpeed,omega=std::sqrt(static_cast<long double>(m.screened().mass_rates()[n].value));
    const long double tau=static_cast<long double>(out.time)-wr::InitialGap/speed,pi=std::acos(-1.L);
    long double x=-wr::InitialGap+speed*out.time,v=speed;
    if(tau>0) {
      if(tau<pi/omega) {x=depth_factor*speed/omega*std::sin(omega*tau);v=speed*std::cos(omega*tau);}
      else {x=-speed*(tau-pi/omega);v=-speed;}
    }
    out.state.x[3*n]=static_cast<double>(x);out.state.q[4*n]=1;
    const auto& k=m.screened().touching().nodes[n].stiffness;
    const double force=k.value*std::max(0.,out.state.x[3*n]);
    out.state.v[3*n]=static_cast<double>(v+(epoch?.5L*h*force/f.mass[n]:0));
    inverse[n]=1/f.mass[n]; impulse+=f.mass[n]*(speed-v);
  }
  contact::NodalWallResult contact_result;
  const auto report=contact::EvaluateNodalWallContact(m.screened().weights(),{out.state.x.data(),f.nodes,3,1},
    {out.state.v.data(),f.nodes,3,1},{inverse.data(),fixed.data(),f.nodes,1,contact::TranslationMassModel::kIsotropicLumped},
    m.screened().law(),epoch+1,&contact_result);
  if(report.status!=contact::NodalWallStatus::Ok)throw std::runtime_error("Synthetic wall contact rejected");
  out.contact.node_count=f.nodes;out.contact.attempt=epoch+1;out.contact.base_epoch=epoch?epoch-1:0;out.contact.candidate=epoch!=0;
  out.contact.resultant=contact_result.resultant;out.contact.potential=contact_result.potential;
  for(unsigned n=0;n<f.nodes;++n) {
    out.contact.nodes[n]=contact_result.nodes[n];out.contact.nodes[n].base_epoch=out.contact.base_epoch;
    out.contact.nodes[n].row.base_epoch=out.contact.base_epoch;
  }
  // Analytic fractional entry can precede the first discrete kick; do not
  // invent a negative opposite-wall impulse to remove that phase uncertainty.
  out.wall_impulse=std::max(0.,static_cast<double>(impulse-(epoch?.5L*h*out.contact.resultant.value:0)));
  for(unsigned e=0;e<f.cells;++e) {
    if(port::InitializeHistory(f.reference[e],{out.time,epoch},out.elements[e].proposed_history)!=port::Status::kSuccess)
      throw std::runtime_error("Synthetic numeric history rejected");
    out.elements[e].kinematics.area=epoch?f.reference[e].area:0;
  }
  return out;
}
} // namespace tl::qualification::qeph::wall_response::test
