#include "ShellBatchPlasticityStorage.h"
#include <new>
#include <stdexcept>
#include <utility>

namespace tl::fea::shell_batch_plasticity_detail {
SetupReport HostStorage::InitializeCollection(const ShellBatchPlasticityBinding& catalog,
    const ShellBatchBinding& binding,ShellBindingFamily family,std::size_t count,
    std::size_t maximum_extra_device_bytes,std::size_t maximum_extra_host_bytes) try {
  if(device_||collection_||!catalog.Matches(binding)||!count||count>MaxShellResidentParents||
      (family!=ShellBindingFamily::Qeph&&family!=ShellBindingFamily::T3)||
      count!=(family==ShellBindingFamily::Qeph?binding.qeph_count():binding.t3_count()))
    return {SetupStatus::InvalidInput,"Plasticity catalog must match the complete joined native collection"};
  Layout layout; std::size_t host_bytes=0;
  if(!Forecast(count,catalog.curve_point_count(),catalog.host_bytes(),maximum_extra_device_bytes,
      maximum_extra_host_bytes,layout,host_bytes))
    return {SetupStatus::ResourceLimit,"Optional plastic section exceeds active byte budgets"};
  std::unique_ptr<ShellBatchPlasticityBinding> owned(new(std::nothrow) ShellBatchPlasticityBinding(catalog));
  util::HostArena arena;
  if(!owned||!arena.Initialize(layout.bytes)) return {SetupStatus::ResourceLimit,"Collection plasticity startup allocation failed"};
  auto* initial=layout.Construct(arena);
  if(!initial) return {SetupStatus::ResourceLimit,"Collection plasticity startup layout is invalid"};
  staging_.Resize(count);
  const auto& source=owned->data_;
  for(std::size_t i=0;i<source.point_count;++i) {
    initial->curve_x[i]=source.curve_x[i]; initial->curve_y[i]=source.curve_y[i];
  }
  std::unique_ptr<std::size_t[]> offsets(new(std::nothrow) std::size_t[count]{});
  if(!offsets) return {SetupStatus::ResourceLimit,"Collection plasticity curve-offset allocation failed"};
  for(std::size_t e=0;e<count;++e) {
    if(!owned->Parameters(family,e,&initial->parameters[e]))
      return {SetupStatus::InvalidInput,"Complete catalog does not resolve a native family parent"};
    offsets[e]=static_cast<std::size_t>(initial->parameters[e].curve.plastic_strain-source.curve_x.data());
    if(offsets[e]>source.point_count||initial->parameters[e].curve.count>source.point_count-offsets[e])
      return {SetupStatus::InvalidInput,"Resolved material curve exceeds the owned pool"};
  }
  DeviceStorage* candidate=nullptr;
  auto error=cudaMalloc(reinterpret_cast<void**>(&candidate),layout.bytes);
  if(error!=cudaSuccess) return {SetupStatus::DeviceFailure,"Optional plastic section allocation failed",error};
  const auto header=layout.Rebase(*initial,candidate);
  for(std::size_t e=0;e<count;++e) {
    auto& curve=initial->parameters[e].curve;
    curve.plastic_strain=header.curve_x+offsets[e];
    curve.yield_stress_pa=header.curve_y+offsets[e];
  }
  *initial=header;
  error=cudaMemcpy(candidate,arena.data(),layout.bytes,cudaMemcpyHostToDevice);
  if(error!=cudaSuccess) { cudaFree(candidate); return {SetupStatus::DeviceFailure,"Collection plastic section initialization copy failed",error}; }
  device_=candidate; device_header_=header; layout_=layout;
  element_count_=count; collection_=std::move(owned);
  return {SetupStatus::Success,"OK"};
} catch(const std::bad_alloc&) { return {SetupStatus::ResourceLimit,"Collection plasticity host allocation failed"}; }
  catch(const std::length_error&) { return {SetupStatus::ResourceLimit,"Collection plasticity host size overflow"}; }
} // namespace tl::fea::shell_batch_plasticity_detail
