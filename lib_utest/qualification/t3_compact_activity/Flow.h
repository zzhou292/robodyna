// SPDX-License-Identifier: MIT
#pragma once
#include "SerialValues.h"
namespace t3_compact_test {
using Phase=m::ActivityPhase;using Error=m::ActivityError;
inline t::BatchReport Setup(sp::SetupReport r) {
  return {r.status==sp::SetupStatus::Success?t::BatchStatus::Success:
      r.status==sp::SetupStatus::NonfiniteResult?t::BatchStatus::NonfiniteResult:t::BatchStatus::InvalidInput,r.message};
}
inline t::BatchReport Serial(Fixture& f,std::vector<std::uint8_t>* output=nullptr) {
  auto r=t::BatchReport{t::BatchStatus::Success,"OK"};
  if(f.has_point){r=Setup(serial::Points(f));if(r.status!=t::BatchStatus::Success)return r;}
  r=Setup(serial::Mixed(f));if(r.status!=t::BatchStatus::Success)return r;
  r=Setup(serial::Failure(f));if(r.status!=t::BatchStatus::Success)return r;
  r=serial::Force(f);if(r.status!=t::BatchStatus::Success)return r;
  r=serial::Roles(f);if(r.status!=t::BatchStatus::Success)return r;
  if(f.has_point){r=serial::PointIdentity(f);if(r.status!=t::BatchStatus::Success)return r;}
  if(output){output->resize(f.size());for(std::size_t i=0;i<f.size();++i)(*output)[i]=
      f.laws[i]==Law::Law44Nip1?(f.points[i].point.failure.history.point_active?1:0):(f.failure[i].active?1:0);}
  return r;
}
inline std::uint32_t HostPhase(const Fixture& f,Phase phase,std::vector<std::uint8_t>& flags,bool reverse) {
  flags.assign(f.size(),0);std::uint32_t first=m::NoActivityFailure;
  for(std::size_t cursor=0;cursor<f.size();++cursor) {
    const auto p=reverse?f.size()-1-cursor:cursor;Error error=Error::None;
    switch(phase) {
      case Phase::OnePoint:error=m::CheckPointActivity(f.laws[p],f.points[p],f.parameters[p],f.time);break;
      case Phase::Mixed:error=m::CheckMixedActivity(f.laws[p],f.plastic[p],f.elastic[p],f.has_point,f.execution);
        if(error==Error::None)flags[p]=static_cast<std::uint8_t>(f.laws[p]);break;
      case Phase::Failure:error=m::CheckFailureActivity(f.laws[p],f.policies[p],f.failure[p],f.plastic[p],f.has_point,f.time);
        if(error==Error::None)flags[p]=f.laws[p]==Law::Law44Nip1?(f.points[p].point.failure.history.point_active?1:0):(f.failure[p].active?1:0);break;
      case Phase::Force:error=m::CheckForceActivity(f.elements[p].reference,f.forces[p],f.laws[p],f.force_time,f.force_epoch);
        if(error==Error::None)flags[p]=static_cast<std::uint8_t>(f.forces[p].proposed_history.data().active);break;
      case Phase::PointIdentity:error=m::CheckPointIdentity(f.elements[p].reference,f.forces[p],f.points[p],f.parameters[p],f.laws[p],f.time,f.epoch);break;
    }
    if(error!=Error::None)first=std::min(first,m::ActivityKey(static_cast<std::uint32_t>(p),error));
  }
  return first;
}
template<class ReadPhase> t::BatchReport Candidate(Fixture& f,ReadPhase read,std::vector<std::uint8_t>* output=nullptr) {
  std::vector<std::uint8_t> packet,roles,failure;
  const auto call=[&](Phase phase,std::vector<std::uint8_t>& flags){return m::ActivityErrorReport(phase,read(phase,flags),f.size());};
  auto r=t::BatchReport{t::BatchStatus::Success,"OK"};
  if(f.has_point){r=call(Phase::OnePoint,packet);if(r.status!=t::BatchStatus::Success)return r;}
  r=call(Phase::Mixed,roles);if(r.status!=t::BatchStatus::Success)return r;
  r=call(Phase::Failure,failure);if(r.status!=t::BatchStatus::Success)return r;
  r=call(Phase::Force,packet);if(r.status!=t::BatchStatus::Success)return r;
  for(std::size_t p=0;p<f.size();++p) {
    if(roles[p]!=static_cast<std::uint8_t>(f.laws[p]))return {t::BatchStatus::NonfiniteResult,"Mapped T3 typed section role differs",static_cast<std::uint32_t>(p)};
    if(packet[p]!=(failure[p]?1:0))return {t::BatchStatus::NonfiniteResult,"Mapped T3 force and failure activity differ",static_cast<std::uint32_t>(p)};
  }
  if(f.has_point){r=call(Phase::PointIdentity,packet);if(r.status!=t::BatchStatus::Success)return r;}
  if(output)*output=failure;return r;
}
} // namespace t3_compact_test
