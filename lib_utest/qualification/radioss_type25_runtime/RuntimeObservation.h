// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/collision/radioss_type25/runtime/Storage.h"
#include "lib_src/solvers/NodalTrialIdentity.h"
#include <vector>
namespace tlfea::contact::radioss_type25::runtime_qualification {
struct Observation {
  std::uint64_t force_base_epoch=0,attempt=0;
  std::vector<lifecycle::Occurrence> occurrences;
  std::vector<NativeRawGeometryResult> geometry;
  std::vector<NativeFrictionResult> response;
  std::vector<std::uint32_t> cohort_ends;
};
// Read-only owning qualification seam. No setter, callback, receipt factory,
// physical publication or production dependency exists here.
class Access {
 public:
  static bool Read(const Transaction& value,tl::fea::FENodalState& owner,
      const tl::fea::NodalTrialToken& token,const tl::fea::NodalAssemblyView& view,Observation* output) {
    if(!value.impl_||!output)return false;const auto& p=*value.impl_;
    if(!p.usable||p.owner!=&owner||p.phase!=Transaction::Impl::Phase::Assembled||
       view.attempt!=p.assembly_view.attempt||view.owner_id!=p.assembly_view.owner_id||
       !tl::fea::trial_identity::SameStamp(owner.accepted(),p.assembly_stamp)||
       owner.AuthenticateAssemblyView(token,view).status!=tl::fea::NodalStatus::Ok)return false;
    const auto count=std::size_t(p.control.required_candidates),kept=std::size_t(p.control.kept);
    const auto cohorts=kept/p.source.force_packet_size+(kept%p.source.force_packet_size!=0);
    std::vector<lifecycle::Occurrence> raw(count);std::vector<NativeRawGeometryResult> geometry(count);
    std::vector<NativeFrictionResult> response(count);std::vector<std::uint32_t> order(count);
    Observation result;result.force_base_epoch=p.assembly_stamp.epoch;result.attempt=view.attempt;
    result.cohort_ends.resize(cohorts);
    // Every vector is alive until the explicit stream drains, including errors.
    auto error=cudaGetLastError();
    const auto copy=[&](void* to,const void* from,std::size_t bytes){if(error==cudaSuccess&&bytes)error=cudaMemcpyAsync(to,from,bytes,cudaMemcpyDeviceToHost,p.stream);};
    copy(raw.data(),p.device.occurrences,count*sizeof(raw[0]));copy(geometry.data(),p.device.geometry,count*sizeof(geometry[0]));
    copy(response.data(),p.device.responses,count*sizeof(response[0]));copy(order.data(),p.device.sorted_slots,count*sizeof(order[0]));
    copy(result.cohort_ends.data(),p.device.cohort_ends,cohorts*sizeof(std::uint32_t));
    const auto drained=cudaStreamSynchronize(p.stream);if(error!=cudaSuccess||drained!=cudaSuccess)return false;
    result.occurrences.reserve(count);result.geometry.reserve(count);result.response.reserve(count);
    for(auto slot:order){if(slot>=count)return false;result.occurrences.push_back(raw[slot]);result.geometry.push_back(geometry[slot]);result.response.push_back(response[slot]);}
    *output=std::move(result);return true;
  }
};
} // namespace tlfea::contact::radioss_type25::runtime_qualification
