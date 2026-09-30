// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"
#include <algorithm>
#include <atomic>
#include <new>
namespace tlfea::contact::radioss_type25::candidates {
namespace {
std::atomic<std::uint64_t> next_identity{1};
std::uint64_t Identity() noexcept {
  auto value=next_identity.load(std::memory_order_relaxed);
  while(value&&value!=UINT64_MAX)
    if(next_identity.compare_exchange_weak(value,value+1,std::memory_order_relaxed))return value;
  return 0;
}
}
Inventory::Inventory()=default;Inventory::~Inventory()=default;
Inventory::Impl::~Impl(){if(stream)cudaStreamSynchronize(stream);if(arena)cudaFree(arena);}
Status Inventory::Preflight(const Source& source,Limits limits,Forecast& output) noexcept {
  auto status=detail::CheckSource(source,limits);if(status!=Status::Ok)return status;
  std::size_t scratch=0;
  if(detail::QueryScratch(source,limits,scratch)!=cudaSuccess)return Status::DeviceFailure;
  detail::Layout layout;status=detail::MakeLayout(source,limits,scratch,sizeof(Inventory)+sizeof(Impl),layout);
  if(status==Status::Ok)output=layout.forecast;return status;
}
Status Inventory::Initialize(const Source& source,Limits limits,cudaStream_t stream) noexcept try {
  if(impl_)return Status::AlreadyInitialized;
  if(!stream||stream==cudaStreamLegacy||stream==cudaStreamPerThread)return Status::InvalidInput;
  auto status=detail::CheckSource(source,limits);if(status!=Status::Ok)return status;
  std::size_t scratch=0;
  if(cudaGetLastError()!=cudaSuccess||detail::QueryScratch(source,limits,scratch)!=cudaSuccess)return Status::DeviceFailure;
  detail::Layout layout;
  status=detail::MakeLayout(source,limits,scratch,sizeof(Inventory)+sizeof(Impl),layout);if(status!=Status::Ok)return status;
  tl::util::BoundedArenaLayout h(limits.max_host_bytes);
  tl::util::ArenaRegion ids_region,mains_region,ranks_region,removal_region;
  if(!h.Append<std::uint64_t>(source.physical_nodes,ids_region)||
     !h.Append<detail::MainEntry>(source.mains,mains_region)||
     !h.Append<std::uint32_t>(source.mains,ranks_region)||
     !h.Append<std::uint32_t>(source.removals,removal_region))return Status::ResourceLimit;
  tl::util::HostArena upload;if(!upload.Initialize(h.bytes()))return Status::ResourceLimit;
  auto* ids=upload.Construct<std::uint64_t>(ids_region);
  auto* mains=upload.Construct<detail::MainEntry>(mains_region);
  auto* ranks=upload.Construct<std::uint32_t>(ranks_region);
  auto* removals=upload.Construct<std::uint32_t>(removal_region);
  if(!ids||!mains||!ranks||!removals)return Status::ResourceLimit;
  std::copy_n(source.node_ids,source.physical_nodes,ids);
  std::sort(ids,ids+source.physical_nodes);
  for(std::size_t i=1;i<source.physical_nodes;++i)if(ids[i]==ids[i-1])return Status::InvalidInput;
  for(std::size_t i=0;i<source.mains;++i){mains[i].source=source.main[i];ranks[i]=i;}
  std::sort(ranks,ranks+source.mains,[&](std::uint32_t a,std::uint32_t c) {
    const auto x=source.main[a].source_id,y=source.main[c].source_id;return x<y||(x==y&&a<c);
  });
  for(std::size_t i=0;i<source.mains;++i)mains[ranks[i]].rank=i;
  if(source.removals)std::copy_n(source.removal_nodes,source.removals,removals);
  for(std::size_t i=0;i<source.mains;++i)
    std::sort(removals+source.removal_offsets[i],removals+source.removal_offsets[i+1]);
  auto next=std::make_unique<Impl>();next->identity=Identity();if(!next->identity)return Status::ResourceLimit;
  next->source=source;next->limits=limits;next->layout=layout;next->stream=stream;
  auto error=cudaMalloc(&next->arena,layout.forecast.device_bytes);
  if(error!=cudaSuccess)return Status::DeviceFailure;
  next->device=detail::Bind(next->arena,layout,source,limits);
  const auto copy=[&](const void* data,const tl::util::ArenaRegion& r) {
    if(error==cudaSuccess&&r.bytes)error=cudaMemcpyAsync(tl::util::ArenaPointer<std::byte>(next->arena,r),
        data,r.bytes,cudaMemcpyHostToDevice,stream);
  };
  copy(source.node_ids,layout.ids);copy(source.constraint_codes,layout.codes);
  copy(source.secondary_nodes,layout.secondary);copy(mains,layout.mains);copy(ranks,layout.ranks);
  copy(source.removal_offsets,layout.removal_offsets);copy(removals,layout.removals);
  const auto drained=cudaStreamSynchronize(stream);
  if(error!=cudaSuccess||drained!=cudaSuccess)return Status::DeviceFailure;
  next->source.node_ids=nullptr;next->source.constraint_codes=nullptr;next->source.secondary_nodes=nullptr;
  next->source.main=nullptr;next->source.removal_offsets=nullptr;next->source.removal_nodes=nullptr;
  impl_=std::move(next);return Status::Ok;
} catch(const std::bad_alloc&) {return Status::ResourceLimit;}
} // namespace tlfea::contact::radioss_type25::candidates
