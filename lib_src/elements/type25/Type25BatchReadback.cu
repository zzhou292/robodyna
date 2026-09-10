// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Type25BatchStorage.h"
#include <cstring>

namespace tl::fea::type25 {
bool Batch::Impl::OutputDisjoint(const void* output,std::size_t bytes) const noexcept {
  using trial_identity::Disjoint;
  if(!Disjoint(output,bytes,this,sizeof(*this))||!Disjoint(output,bytes,staging.get(),config.element_count*sizeof(Evaluation)))return false;
  const auto& m=*source;
  return Disjoint(output,bytes,m.connections(),m.connection_count()*sizeof(ConnectionInput))&&
    Disjoint(output,bytes,m.properties(),m.property_count()*sizeof(PropertyInput))&&
    Disjoint(output,bytes,m.references(),m.connection_count()*sizeof(Reference))&&
    Disjoint(output,bytes,m.initial_histories(),m.connection_count()*sizeof(History))&&
    Disjoint(output,bytes,m.endpoint_mass(),2*m.connection_count()*sizeof(EndpointMass))&&
    Disjoint(output,bytes,combined->nodes().data(),combined->node_count()*sizeof(NodalMassNode));
}
BatchReport Batch::CopyAcceptedResults(const NodalStamp& expected,Evaluation* output,std::size_t capacity,BatchDiagnostics* diagnostics) {
  if(!impl_)return {BatchStatus::NotInitialized,"TYPE25 batch is not initialized"};
  auto& s=*impl_;
  if(!s.bound)return {BatchStatus::NotBound,"TYPE25 initial live-source binding is required"};
  if(!trial_identity::SameStamp(expected,s.accepted_stamp))return {BatchStatus::StaleTrial,"TYPE25 accepted identity differs"};
  if(capacity<s.config.element_count||capacity>SIZE_MAX/sizeof(Evaluation))return {BatchStatus::ResourceLimit,"TYPE25 result capacity is invalid"};
  const auto bytes=s.config.element_count*sizeof(Evaluation);using trial_identity::Disjoint;
  if(!s.OutputDisjoint(output,bytes)||!s.OutputDisjoint(diagnostics,sizeof(*diagnostics))||
     !Disjoint(output,bytes,diagnostics,sizeof(*diagnostics))||!Disjoint(output,bytes,&expected,sizeof(expected))||
     !Disjoint(diagnostics,sizeof(*diagnostics),&expected,sizeof(expected))||!Disjoint(output,bytes,this,sizeof(*this))||
     !Disjoint(diagnostics,sizeof(*diagnostics),this,sizeof(*this)))
    return {BatchStatus::InvalidInput,"TYPE25 result output is missing, overlapping or overflowing"};
  const auto r=s.ReadResults(s.accepted);if(r.status!=BatchStatus::Success)return r;
  std::memcpy(output,s.staging.get(),bytes);*diagnostics=s.accepted_diagnostics;return {BatchStatus::Success,"OK"};
}
BatchReport Batch::CopyPreparedResults(const BatchDiagnostics& expected,Evaluation* output,std::size_t capacity) {
  if(!impl_)return {BatchStatus::NotInitialized,"TYPE25 batch is not initialized"};
  auto& s=*impl_;
  if(!s.bound)return {BatchStatus::NotBound,"TYPE25 initial live-source binding is required"};
  if(!s.pending||!batch_detail::SameDiagnostics(expected,s.candidate_diagnostics))return {BatchStatus::StaleTrial,"TYPE25 prepared identity differs"};
  if(capacity<s.config.element_count||capacity>SIZE_MAX/sizeof(Evaluation))return {BatchStatus::ResourceLimit,"TYPE25 result capacity is invalid"};
  const auto bytes=s.config.element_count*sizeof(Evaluation);
  if(!s.OutputDisjoint(output,bytes)||!trial_identity::Disjoint(output,bytes,&expected,sizeof(expected))||
     !trial_identity::Disjoint(output,bytes,this,sizeof(*this)))
    return {BatchStatus::InvalidInput,"TYPE25 result output overlaps input or owned data"};
  const auto r=s.ReadResults(s.trial);if(r.status!=BatchStatus::Success)return r;
  std::memcpy(output,s.staging.get(),bytes);return {BatchStatus::Success,"OK"};
}
} // namespace tl::fea::type25
