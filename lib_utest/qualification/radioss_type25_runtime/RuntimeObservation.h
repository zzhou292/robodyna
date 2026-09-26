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
struct NormalObservation {
  tl::fea::NativeContactPublicationSnapshot accepted;
  std::uint64_t force_base_epoch=0,attempt=0;
  std::vector<StoredNormal> face;
  std::vector<startup::NormalReference> references;
  std::vector<std::uint32_t> active,tags;
};
struct InitializationObservation {
  tl::fea::NativeContactPublicationSnapshot accepted;
  std::vector<NativeGeometryHistory> rows[2];
  std::vector<lifecycle::Secondary> secondary[2];
  std::vector<lifecycle::Main> mains;
};
// Read-only owning qualification seam. No setter, callback, receipt factory,
// physical publication or production dependency exists here.
class Access {
 public:
  static std::size_t FixedHostBytes() noexcept {return sizeof(Transaction)+sizeof(Transaction::Impl);}
  static bool ReadCandidateReport(const Transaction& value,std::uint64_t attempt,candidates::Report* output) {
    if(!value.impl_||!output||!attempt)return false;
    for(const auto& inventory:value.impl_->inventory) {
      const auto report=inventory.last_report();
      if(report.stamp.attempt==attempt){*output=report;return true;}
    }
    return false;
  }
  static bool ReadInitialization(const Transaction& value,InitializationObservation* output) {
    if(!value.impl_||!output)return false;const auto& p=*value.impl_;
    const auto accepted=p.state.Accepted(*p.owner);
    if(!p.usable||!accepted.available||p.phase!=Transaction::Impl::Phase::Idle||p.owner->accepted().epoch)return false;
    InitializationObservation next;next.accepted=accepted;const auto rows=p.source.selection.secondary_count;
    next.mains.resize(p.source.selection.main_count);auto error=cudaGetLastError();
    const auto copy=[&](void* to,const void* from,std::size_t bytes){if(error==cudaSuccess&&bytes)error=cudaMemcpyAsync(to,from,bytes,cudaMemcpyDeviceToHost,p.stream);};
    for(unsigned slab=0;slab<2;++slab) {
      next.rows[slab].resize(rows);next.secondary[slab].resize(rows);
      copy(next.rows[slab].data(),p.device.history[slab],rows*sizeof(NativeGeometryHistory));
      copy(next.secondary[slab].data(),p.device.secondary[slab],rows*sizeof(lifecycle::Secondary));
    }
    copy(next.mains.data(),p.device.source.mains,next.mains.size()*sizeof(lifecycle::Main));
    const auto drained=cudaStreamSynchronize(p.stream);if(error!=cudaSuccess||drained!=cudaSuccess)return false;
    *output=std::move(next);return true;
  }
  static bool ReadAcceptedNormals(const Transaction& value,NormalObservation* output) {
    if(!value.impl_||!output)return false;const auto& p=*value.impl_;
    const auto accepted=p.state.Accepted(*p.owner);
    if(!p.usable||!p.device.normal.shape.enabled||!accepted.available)return false;
    NormalObservation next;next.accepted=accepted;
    next.force_base_epoch=accepted.force_phase_available?accepted.force_base_stamp.epoch:0;
    if(!CopyNormals(p,accepted.selectors.history,false,next))return false;
    *output=std::move(next);return true;
  }
  static bool ReadAttemptNormals(const Transaction& value,tl::fea::FENodalState& owner,
      const tl::fea::NodalTrialToken& token,const tl::fea::NodalAssemblyView& view,NormalObservation* output) {
    if(!value.impl_||!output)return false;const auto& p=*value.impl_;
    if(!p.usable||!p.normal_ready||!p.device.normal.shape.enabled||p.owner!=&owner||
       p.phase!=Transaction::Impl::Phase::Assembled||view.attempt!=p.assembly_view.attempt||
       !tl::fea::trial_identity::SameStamp(owner.accepted(),p.assembly_stamp)||
       owner.AuthenticateAssemblyView(token,view).status!=tl::fea::NodalStatus::Ok)return false;
    NormalObservation next;next.force_base_epoch=p.assembly_stamp.epoch;next.attempt=view.attempt;
    if(!CopyNormals(p,p.trial_selectors.history,true,next))return false;
    *output=std::move(next);return true;
  }
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
 private:
  static bool CopyNormals(const Transaction::Impl& p,unsigned slab,bool masks,NormalObservation& out) {
    out.face.resize(4*p.source.selection.main_count);out.references.resize(p.source.selection.normal_count);
    if(masks){out.active.resize(p.source.selection.main_count);out.tags.resize(p.source.selection.node_count);}
    auto error=cudaGetLastError();
    const auto copy=[&](void* to,const void* from,std::size_t bytes){if(error==cudaSuccess&&bytes)error=cudaMemcpyAsync(to,from,bytes,cudaMemcpyDeviceToHost,p.stream);};
    copy(out.face.data(),p.device.normal.face[slab],out.face.size()*sizeof(StoredNormal));
    copy(out.references.data(),p.device.normal.references[slab],out.references.size()*sizeof(startup::NormalReference));
    if(masks){copy(out.active.data(),p.device.normal.active,out.active.size()*sizeof(std::uint32_t));
      copy(out.tags.data(),p.device.normal.tags,out.tags.size()*sizeof(std::uint32_t));}
    const auto drained=cudaStreamSynchronize(p.stream);return error==cudaSuccess&&drained==cudaSuccess;
  }
};
} // namespace tlfea::contact::radioss_type25::runtime_qualification
