// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"
#include "Packing.h"
#include "../search/Ranges.h"
#include "lib_src/solvers/NodalTrialIdentity.h"
#include <cstring>
namespace tlfea::contact::radioss_type25::candidates {
namespace {
bool Separate(const void* a,std::size_t bytes,const void* b,std::size_t extent) noexcept {
  return !bytes||!extent||tl::fea::trial_identity::Disjoint(a,bytes,b,extent);
}
bool VectorField(VectorView view,std::size_t nodes,const void* arena,std::size_t bytes) noexcept {
  std::size_t extent=0;return search::detail::VectorSpan(view,nodes,extent)&&Separate(view.data,extent,arena,bytes);
}
}
Status Inventory::Impl::Check(const Current& in) const noexcept {
  if(!usable)return Status::Unusable;
  if(in.stamp.source.source!=source.stamp.source||in.stamp.source.topology!=source.stamp.topology||
     !in.stamp.activity||!in.stamp.gaps||!in.stamp.geometry||!in.stamp.attempt||!in.stamp.reference)
    return Status::StaleReference;
  const auto bytes=layout.forecast.device_bytes;
  if(!VectorField(in.positions,source.physical_nodes,arena,bytes)||
     !VectorField(in.velocities,source.physical_nodes,arena,bytes))return Status::InvalidInput;
  const double* arrays[]{in.secondary_stiffness,in.secondary_gaps,in.main_stiffness,in.main_gaps,in.main_curvature};
  for(unsigned i=0;i<5;++i) {
    const auto count=i<2?source.secondaries:source.mains;
    if(!search::detail::Span(arrays[i],count)||!Separate(arrays[i],count*sizeof(double),arena,bytes))return Status::InvalidInput;
  }
  namespace d=detail;namespace v=tl::math::fixed3;
  const auto lo=in.domain.minimum,hi=in.domain.maximum;
  if((in.domain_policy!=DomainPolicy::Bounded&&in.domain_policy!=DomainPolicy::AllFinite)||
     (in.domain_policy==DomainPolicy::Bounded&&
      (!v::Finite(lo)||!v::Finite(hi)||lo.x>hi.x||lo.y>hi.y||lo.z>hi.z))||
     !d::Nonnegative(in.margin)||!tl::math::Finite(in.gap_load)||!d::Nonnegative(in.drad)||
     !d::Nonnegative(in.stored_motion)||!d::Nonnegative(in.previous_dt))return Status::InvalidInput;
  return Status::Ok;
}
Status Inventory::Impl::Fence(cudaError_t error) noexcept {
  if(error==cudaSuccess)error=cudaMemcpyAsync(&control,device.control,sizeof(control),cudaMemcpyDeviceToHost,stream);
  const auto drained=cudaStreamSynchronize(stream);++report.host_fences;
  if(error!=cudaSuccess||drained!=cudaSuccess){usable=false;pending=false;return Status::DeviceFailure;}
  report.active_secondaries=control.active;report.envelope_encounters=control.encounters;
  report.tasks=control.tasks;report.pairs=control.pairs;
  static_assert(sizeof(double)==sizeof(control.maximum_gap_bits));
  std::memcpy(&report.maximum_secondary_gap,&control.maximum_gap_bits,sizeof(double));
  if(control.failure!=~0ull) {
    report.failure_row=control.failure>>8;return static_cast<Status>(control.failure&255);
  }
  return Status::Ok;
}
Status Inventory::Stage(const Current& in) noexcept {
  if(!impl_)return Status::NotInitialized;
  auto& p=*impl_;p.pending=false;p.report={};p.report.stamp=in.stamp;
  if(p.sequence==UINT64_MAX){p.usable=false;p.report.status=Status::ResourceLimit;return p.report.status;}
  ++p.sequence;
  auto status=p.Check(in);if(status!=Status::Ok){p.report.status=status;return status;}
  Current native=in;
  if(p.device.si) {
    if(in.domain_policy==DomainPolicy::Bounded) {
      native.domain.minimum=tl::math::fixed3::Divide(in.domain.minimum,p.device.length);
      native.domain.maximum=tl::math::fixed3::Divide(in.domain.maximum,p.device.length);
    }
    native.margin/=p.device.length;native.gap_load/=p.device.length;native.drad/=p.device.length;
    native.stored_motion/=p.device.length;native.previous_dt/=p.device.time;
    status=p.Check(native);if(status!=Status::Ok){p.report.status=Status::NonfiniteResult;return p.report.status;}
  }
  auto error=cudaGetLastError();
  if(error==cudaSuccess)error=detail::BuildRanges(p.device,native,p.stream);
  p.report.own_kernel_launches=2+(p.source.secondaries?1:0)+(p.source.mains?1:0);
  p.report.sort_calls=p.source.secondaries?1:0;p.report.scan_calls=1;
  status=p.Fence(error);
  if(status==Status::Ok&&p.control.tasks>p.limits.max_tasks)status=Status::ResourceLimit;
  if(status==Status::Ok) {
    error=detail::CountPairs(p.device,native,p.control.tasks,p.stream);
    p.report.own_kernel_launches+=2+(p.control.tasks?1:0);++p.report.scan_calls;
    status=p.Fence(error);
    if(status==Status::Ok&&p.control.pairs>p.limits.max_pairs)status=Status::ResourceLimit;
  }
  if(status==Status::Ok) {
    error=detail::FillPairs(p.device,native,p.control.tasks,p.control.pairs,p.stream);
    p.report.own_kernel_launches+=1+(p.control.tasks?1:0)+(p.control.pairs?1:0);
    p.report.sort_calls+=p.control.pairs?1:0;status=p.Fence(error);
  }
  p.report.status=status;p.pending=status==Status::Ok;return status;
}
InventoryView Inventory::view() const noexcept {
  InventoryView out;if(!impl_||!impl_->pending)return out;const auto& p=*impl_;
  out.owner_=p.identity;out.sequence_=p.sequence;out.pairs_=p.device.pairs;
  out.offsets_=p.device.secondary_offsets;out.count_=p.control.pairs;out.rows_=p.source.secondaries;
  out.stamp_=p.report.stamp;return out;
}
bool Inventory::IsCurrent(const InventoryView& v) const noexcept {
  return impl_&&impl_->pending&&v.owner_==impl_->identity&&v.sequence_==impl_->sequence;
}
void Inventory::Discard() noexcept {if(impl_)impl_->pending=false;}
Report Inventory::last_report() const noexcept {if(impl_)return impl_->report;Report out;out.status=Status::NotInitialized;return out;}
Forecast Inventory::allocations() const noexcept {return impl_?impl_->layout.forecast:Forecast{};}
} // namespace tlfea::contact::radioss_type25::candidates
