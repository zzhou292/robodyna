#pragma once
#include "WallResponseTransaction.h"
#include "lib_src/collision/Q4ContactBounds.h"
#include <algorithm>
#include <cmath>

namespace tl::qualification::qeph::wall_response::runtime_detail {
inline State StateOf(const c::Snapshot& s) {
  State out;
  std::copy_n(s.x.begin(),out.x.size(),out.x.begin()); std::copy_n(s.v.begin(),out.v.size(),out.v.begin());
  std::copy_n(s.omega.begin(),out.omega.size(),out.omega.begin()); std::copy_n(s.q.begin(),out.q.size(),out.q.begin());
  return out;
}
inline Results ResultsOf(const c::PortResults& result) { return {result[0],result[1]}; }
inline ContactState ContactOf(const contact::NodalWallDeviceResults& result) {
  ContactState out; const auto& d=result.diagnostics;
  out.base_epoch=d.base_epoch; out.attempt=d.attempt; out.node_count=d.node_count;
  out.candidate=d.phase==contact::NodalWallDevicePhase::PreparedCandidate;
  out.resultant=d.resultant; out.potential=d.potential;
  // Caller checks the immutable one/two-cell extent before this fixed copy.
  std::copy_n(result.nodes,out.node_count,out.nodes.begin()); return out;
}
inline bool AddImpulse(double a,double ae,double b,double be,double& value,double& error) {
  namespace qb=contact::q4_bounds;
  if(!std::isfinite(a)||a<0||!std::isfinite(ae)||ae<0||
     !std::isfinite(b)||b<0||!std::isfinite(be)||be<0) return false;
  Interval x,y,sum;
  if(!qb::AddScalar(a,-ae,false,&x.lower)||!qb::AddScalar(a,ae,true,&x.upper)||
     !qb::AddScalar(b,-be,false,&y.lower)||!qb::AddScalar(b,be,true,&y.upper)) return false;
  x.lower=std::max(0.,x.lower); y.lower=std::max(0.,y.lower);
  contact::Q4CertifiedIntegral result;
  if(!qb::Add(x,y,&sum)||!qb::Certify(a+b,sum,&result)) return false;
  value=result.value; error=result.error; return true;
}
inline std::array<double,10> Ledgers(const StepStage& s) {
  return {s.source.kick_ratio,s.source.momentum_ratio,s.source.angular_ratio,s.source.internal_work_ratio,
          s.source.source_work_ratio,s.contact.work_ratio,s.contact.impulse_ratio,s.contact.defect_ratio,
          s.regular_maximum,s.hourglass_maximum};
}
} // namespace tl::qualification::qeph::wall_response::runtime_detail
