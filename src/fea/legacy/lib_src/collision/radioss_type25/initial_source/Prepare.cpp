// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Internal.h"
#include "../search_startup/Internal.h"
#include "../tied_removal/Internal.h"
#include "../current_normals/Types.h"
#include <algorithm>
#include <new>
#include <type_traits>
namespace tlfea::contact::radioss_type25::initial_source {
namespace {
struct Arena {void* data=nullptr;~Arena(){if(data)cudaFree(data);}};
struct Drain {cudaStream_t stream;~Drain(){if(stream)cudaStreamSynchronize(stream);}};
bool RetainedFits(const detail::Prepared& p,std::size_t bound,std::size_t owner_bytes) noexcept {
  tl::util::BoundedArenaLayout actual(bound);tl::util::ArenaRegion region;
  if(!actual.Append<std::byte>(owner_bytes,region))return false;
  const auto charge=[&](const auto& vector) {
    using Value=typename std::decay_t<decltype(vector)>::value_type;
    return actual.Append<Value>(vector.capacity(),region);
  };
#define V(field) if(!charge(p.field))return false
  V(positions);V(nodes);V(mains);V(secondary);V(references);V(main_gap);V(primary_extent);V(solids);
  V(solid_offsets);V(solid_incidence);V(support_solid);V(internal_main);
  V(removal_offsets);V(removal_nodes);V(final_offsets);V(final_mains);V(final_main_offsets);V(final_nodes);
  V(normal_offsets);V(normal_mains);V(topology);V(expanded_to_primary);V(primary_to_partner);
  V(primary_identities);V(raw_origins);V(raw_origin_to_primary);V(primary_roles);
  V(primary_corners);V(before_shell);V(final_support);
  V(sweep_mains);V(main_ranks);V(secondary_nodes);V(main_nodes);V(node_ids);V(codes);
#undef V
  return true;
}
Forecast Plan(const Input& in,Limits limits) noexcept {
  Forecast f;auto report=detail::Admit(in,limits,f);if(report.status!=Status::Ok){f.status=report.status;return f;}
  const auto n=in.contact.node_count,g=in.contact.main_count,s=in.contact.secondary_count,p=in.starter.primary_count;
  const bool mixed=in.mesh.profile==startup::Profile::MixedSurface;
  search_startup::detail::Layout gl;
  const auto gr=search_startup::detail::MakeLayout(n,p,s,limits.geometric,gl,in.auxiliary_rigid_primary_count,mixed?g:0);
  if(gr.status!=search_startup::Status::Ok){f.status=Status::ResourceLimit;return f;}
  f.geometric_output_bytes=gl.forecast.output_bytes;f.geometric_scratch_bytes=gl.forecast.scratch_bytes;
  if(in.tied_interface_count) {
    tied_removal::Input ti;ti.source.mesh=in.mesh;ti.source.main_count=g;ti.source.secondary_count=s;
    ti.source.auxiliary_rigid_primary_count=in.auxiliary_rigid_primary_count;ti.interfaces=in.tied_interfaces;ti.interface_count=in.tied_interface_count;
    const auto tf=tied_removal::PreflightComposed(ti,limits.tied);
    if(tf.status!=search_startup::Status::Ok){f.status=Status::ResourceLimit;return f;}
    f.tied_output_bytes=tf.output_bytes;f.tied_scratch_bytes=tf.scratch_bytes;
  }
  candidates::Limits sl;sl.max_tasks=limits.max_tasks;sl.max_pairs=limits.max_pairs;
  const auto removals=std::min(g*s,std::max(limits.geometric.max_removals,limits.tied.search.max_removals));
  std::size_t cub=0;
  if(candidates::detail::QueryStorageScratch({n,s,g,removals},sl,cub)!=cudaSuccess){f.status=Status::DeviceFailure;return f;}
  detail::Layout layout;const auto status=detail::MakeLayout(in,limits,cub,layout);
  if(status!=Status::Ok){f.status=status;return f;}
  const auto geometry=f;f=layout.forecast;
  f.geometric_output_bytes=geometry.geometric_output_bytes;f.geometric_scratch_bytes=geometry.geometric_scratch_bytes;
  f.tied_output_bytes=geometry.tied_output_bytes;f.tied_scratch_bytes=geometry.tied_scratch_bytes;
  // Count every simultaneously live source vector, then a second complete copy
  // bound for vector capacity/validation staging. Source input backing itself
  // is retained by the caller and belongs to its separate source forecast.
  tl::util::BoundedArenaLayout host(limits.max_host_bytes);tl::util::ArenaRegion region;
  const auto add=[&](std::size_t bytes){return host.Append<std::byte>(bytes,region);};
  tl::util::BoundedArenaLayout source(limits.max_host_bytes);tl::util::ArenaRegion r;
#define H(type,count) if(!source.Append<type>(count,r)){f.status=Status::ResourceLimit;return f;}
  H(Vector,n);H(detail::Node,n);H(detail::Main,g);H(detail::Secondary,s);H(detail::Reference,in.contact.normal_count);
  H(double,g+p);H(std::uint32_t,in.main_node_count);H(EightSlotSolid,in.solid_count);H(std::uint32_t,n+1);H(std::uint32_t,8*in.solid_count);
  H(std::uint32_t,2*g);H(std::uint64_t,g+1);H(std::uint32_t,2*removals);H(std::uint32_t,s+1);
  H(candidates::detail::MainEntry,g);H(std::uint32_t,g);H(std::uint32_t,s);H(std::uint64_t,n);H(int,n);
  H(std::uint32_t,g+1);H(std::uint32_t,removals);H(std::uint32_t,in.contact.normal_count+1);
  H(std::uint32_t,in.starter.normal_incidence_count);H(startup::Main,g);
  H(std::uint32_t,g+p);H(startup::PrimaryFaceIdentity,in.starter.primary_identity_count);
  H(startup::PrimaryFaceIdentity,in.starter.raw_origin_count);H(std::uint32_t,in.starter.raw_origin_count);
  H(startup::ShellSideRole,in.starter.primary_role_count);
  if(in.starter.post_gapm) {
    H(startup::PrimaryCornerPermutation,p);H(startup::PreShellSolidSupport,p);H(startup::PostGapmMainSupport,g);
  }
#undef H
  if(!add(source.bytes())||!add(source.bytes())||!add(sizeof(detail::Prepared)+sizeof(detail::Layout)+sizeof(Forecast)+sizeof(Limits)+sizeof(SeedIdentity)+1024)) {
    f.status=Status::ResourceLimit;return f;
  }
  f.retained_host_bytes=host.bytes();
  if(!add(sizeof(Input)+sizeof(DeviceSeed)+1024)||
      !add(f.geometric_output_bytes)||!add(f.geometric_scratch_bytes)||!add(f.tied_output_bytes)||!add(f.tied_scratch_bytes)||
      !add(s*sizeof(tied_removal::History))||!add(s*sizeof(search_startup::Secondary))||
      !add(in.interface_count*sizeof(std::uint64_t))||!add(in.solid_count*sizeof(std::pair<std::uint64_t,std::uint32_t>))||
      !add(n*sizeof(std::uint8_t))||!add(n*sizeof(std::uint64_t))||!add((n+1)*sizeof(std::uint32_t))||
      !add(mixed?current_normals::MixedSourceValidationBytes(p):0)) {f.status=Status::ResourceLimit;return f;}
  f.peak_host_bytes=host.bytes();f.temporary_host_bytes=f.peak_host_bytes-f.retained_host_bytes;return f;
}
Report Decode(const detail::Control& control,const candidates::detail::Control& sweep,const detail::Prepared& p) noexcept {
  Report r;r.status=Status::Ok;r.diagnostics=p.diagnostics;
  r.diagnostics.encounters=sweep.encounters;r.diagnostics.tasks=sweep.tasks;r.diagnostics.pairs=sweep.pairs;
  r.diagnostics.warm_before_tied=control.warm_before;r.diagnostics.warm_after_tied=control.warm_after;
  r.diagnostics.tied_reset=control.reset;r.diagnostics.warm_positive_main=control.warm_positive;
  r.diagnostics.warm_zero_main=control.warm_zero;r.diagnostics.warm_negative_main=control.warm_negative;
  r.diagnostics.changed_gap_corners=control.changed_gaps;r.diagnostics.solid_tagged_nodes=control.solid_nodes;
  r.diagnostics.large_secondaries=control.large_nodes;r.diagnostics.edge_average=control.edge_average;
  for(unsigned k=0;k<3;++k)r.diagnostics.grid[k]=control.grid[k];
  if(control.failure!=~0ull){r.status=static_cast<Status>(control.failure&255u);r.row=std::size_t(control.failure>>8);}
  else if(sweep.failure!=~0ull) {
    r.row=std::size_t(sweep.failure>>8);
    switch(static_cast<candidates::Status>(sweep.failure&255u)) {
      case candidates::Status::ResourceLimit:r.status=Status::ResourceLimit;break;
      case candidates::Status::NonfiniteResult:r.status=Status::NonfiniteResult;break;
      case candidates::Status::UnsupportedProfile:r.status=Status::UnsupportedProfile;break;
      case candidates::Status::DeviceFailure:r.status=Status::DeviceFailure;break;
      default:r.status=Status::InvalidInput;break;
    }
  }
  return r;
}
}
PreparedSource::PreparedSource() noexcept=default;
PreparedSource::~PreparedSource()=default;
PreparedSource::PreparedSource(PreparedSource&&) noexcept=default;
PreparedSource& PreparedSource::operator=(PreparedSource&&) noexcept=default;
bool PreparedSource::prepared() const noexcept{return bool(impl_);}
SeedIdentity PreparedSource::identity() const noexcept{return impl_?impl_->identity:SeedIdentity{};}
Forecast PreparedSource::forecast() const noexcept{return impl_?impl_->forecast:Forecast{};}
FinalRemovalView PreparedSource::removals() const noexcept {
  if(!impl_)return {};
  const auto& p=impl_->source;
  return {p.final_main_offsets.data(),p.final_nodes.empty()?nullptr:p.final_nodes.data(),
      {p.final_offsets.data(),p.final_offsets.size(),p.final_mains.empty()?nullptr:p.final_mains.data(),p.final_mains.size()},
      p.mains.size(),p.final_mains.size(),p.added_removals,p.native_nodes,
      p.diagnostics.mean_length,p.diagnostics.engine_margin,p.diagnostics.initial_margin,p.population,p.native_nodes_exact,
      p.primary_extent.data(),p.primary_extent.size()};
}
DeviceSeed::DeviceSeed() noexcept=default;
DeviceSeed::~DeviceSeed()=default;
DeviceSeed::DeviceSeed(DeviceSeed&&) noexcept=default;
DeviceSeed& DeviceSeed::operator=(DeviceSeed&&) noexcept=default;
bool DeviceSeed::prepared() const noexcept{return bool(impl_);}
SeedIdentity DeviceSeed::identity() const noexcept{return impl_?impl_->identity:SeedIdentity{};}
Diagnostics DeviceSeed::diagnostics() const noexcept{return impl_?impl_->diagnostics:Diagnostics{};}
Forecast Preflight(const Input& in,Limits limits) noexcept{return Plan(in,limits);}
Report PrepareSource(const Input& in,Limits limits,PreparedSource& output) noexcept {
  if(output.impl_)return {Status::AlreadyPrepared};
  const auto plan=Plan(in,limits);if(plan.status!=Status::Ok)return {plan.status};
  try {
    auto next=std::make_unique<PreparedSource::Impl>();
    auto report=detail::PrepareHost(in,limits,next->source);if(report.status!=Status::Ok)return report;
    if(!RetainedFits(next->source,plan.retained_host_bytes,sizeof(*next)+sizeof(PreparedSource)+512))return {Status::ResourceLimit};
    auto status=detail::MakeLayout(in,limits,plan.cub_bytes,next->layout);if(status!=Status::Ok)return {status};
    next->forecast=plan;next->limits=limits;
    next->identity={in.stamp,in.units,in.contact.node_count,in.starter.primary_count,in.contact.main_count,
        in.contact.secondary_count,in.contact.normal_count,in.phase,next->source.population,next->source.native_nodes_exact,in.engine_handoff};
    report.diagnostics=next->source.diagnostics; // GPU candidate counts remain unavailable until CountPairs.
    output.impl_=std::move(next);return report;
  } catch(const std::bad_alloc&) {return {Status::ResourceLimit};}
  catch(...) {return {Status::InvalidInput};}
}
Report Prepare(const Input& in,Limits limits,CUstream_st* stream,DeviceSeed& output) noexcept {
  if(output.impl_)return {Status::AlreadyPrepared};
  if(!stream)return {Status::InvalidInput};
  PreparedSource source;const auto report=PrepareSource(in,limits,source);
  if(report.status!=Status::Ok)return report;
  return Prepare(source,stream,output);
}
Report Prepare(const PreparedSource& prepared,CUstream_st* stream,DeviceSeed& output) noexcept {
  if(output.impl_)return {Status::AlreadyPrepared};
  if(!prepared.impl_)return {Status::NotPrepared};
  if(!stream)return {Status::InvalidInput};
  try {
    const auto& plan=*prepared.impl_;const auto& source=plan.source;
    const auto& in=source.descriptor;const auto& layout=plan.layout;const auto limits=plan.limits;
    Report report;
    Arena work,sweep;auto next=std::make_unique<DeviceSeed::Impl>();
    // Host source survives until Drain, which precedes every device allocation's
    // destruction on all return/exception paths.
    Drain drain{stream};
    auto e=cudaMalloc(&work.data,layout.forecast.source_device_bytes);
    if(e==cudaSuccess)e=cudaMalloc(&sweep.data,layout.forecast.sweep_device_bytes);
    if(e==cudaSuccess)e=cudaMalloc(&next->data,layout.seed.bytes);
    if(e!=cudaSuccess)return {Status::DeviceFailure};
    if((e=cudaMemsetAsync(work.data,0,layout.forecast.source_device_bytes,stream))!=cudaSuccess||
       (e=cudaMemsetAsync(sweep.data,0,layout.forecast.sweep_device_bytes,stream))!=cudaSuccess)return {Status::DeviceFailure};
    auto device=detail::Bind(work.data,sweep.data,next->data,in,source,layout,limits);
    detail::Control control;candidates::detail::Control counters;
    const auto read=[&]() {
      auto error=cudaMemcpyAsync(&control,device.control,sizeof(control),cudaMemcpyDeviceToHost,stream);
      if(error==cudaSuccess)error=cudaMemcpyAsync(&counters,device.sweep.control,sizeof(counters),cudaMemcpyDeviceToHost,stream);
      const auto waited=cudaStreamSynchronize(stream);return error==cudaSuccess?waited:error;
    };
    if(detail::Upload(source,device,stream)!=cudaSuccess||detail::PrepareOperands(device,stream)!=cudaSuccess||
        detail::BuildRanges(device,stream)!=cudaSuccess||read()!=cudaSuccess)return {Status::DeviceFailure};
    report=Decode(control,counters,source);if(report.status!=Status::Ok)return report;
    if(counters.tasks>limits.max_tasks){report.status=Status::ResourceLimit;return report;}
    if(detail::CountPairs(device,std::size_t(counters.tasks),stream)!=cudaSuccess||read()!=cudaSuccess)return {Status::DeviceFailure};
    report=Decode(control,counters,source);if(report.status!=Status::Ok)return report;report.counts_complete=true;
    if(counters.pairs>limits.max_pairs){report.status=Status::ResourceLimit;return report;}
    const auto tasks=std::size_t(counters.tasks),pairs=std::size_t(counters.pairs);
    if(detail::FillPairs(device,tasks,pairs,stream)!=cudaSuccess||detail::ProduceRows(device,pairs,stream)!=cudaSuccess||read()!=cudaSuccess)
      return {Status::DeviceFailure};
    report=Decode(control,counters,source);report.counts_complete=true;if(report.status!=Status::Ok)return report;
    next->bytes=layout.seed.bytes;next->layout=layout.seed;next->diagnostics=report.diagnostics;
    next->identity=plan.identity;
    output.impl_=std::move(next);return report;
  } catch(const std::bad_alloc&) {return {Status::ResourceLimit};}
  catch(...) {return {Status::DeviceFailure};}
}
}
