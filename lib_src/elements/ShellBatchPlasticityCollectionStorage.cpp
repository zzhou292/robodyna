#include "ShellBatchPlasticityStorage.h"
#include <new>
#include <utility>

namespace tl::fea::shell_batch_plasticity_detail {
SetupReport HostStorage::InitializeCollection(const ShellBatchPlasticityBinding& catalog,
    const ShellBatchBinding& binding,ShellBindingFamily family,std::size_t count,
    std::size_t maximum_extra_device_bytes) {
  if(device_||collection_||!catalog.Matches(binding)||!count||count>MaxShellCollectionParents||
      (family!=ShellBindingFamily::Qeph&&family!=ShellBindingFamily::T3)||
      count!=(family==ShellBindingFamily::Qeph?binding.qeph_count():binding.t3_count()))
    return {SetupStatus::InvalidInput,"Plasticity catalog must match the complete joined native collection"};
  if(sizeof(DeviceStorage)>maximum_extra_device_bytes)
    return {SetupStatus::ResourceLimit,"Optional plastic section allocation exceeds batch byte cap"};
  std::unique_ptr<ShellBatchPlasticityBinding> owned(new(std::nothrow) ShellBatchPlasticityBinding(catalog));
  std::unique_ptr<DeviceStorage> initial(new(std::nothrow) DeviceStorage{});
  if(!owned||!initial) return {SetupStatus::ResourceLimit,"Collection plasticity startup allocation failed"};
  const auto& source=owned->data_;
  for(std::size_t i=0;i<source.point_count;++i) {
    initial->curve_x[i]=source.curve_x[i]; initial->curve_y[i]=source.curve_y[i];
  }
  std::array<std::size_t,MaxShellCollectionParents> offsets{};
  for(std::size_t e=0;e<count;++e) {
    if(!owned->Parameters(family,e,&initial->parameters[e]))
      return {SetupStatus::InvalidInput,"Complete catalog does not resolve a native family parent"};
    offsets[e]=static_cast<std::size_t>(initial->parameters[e].curve.plastic_strain-source.curve_x.data());
  }
  auto error=cudaMalloc(reinterpret_cast<void**>(&device_),sizeof(DeviceStorage));
  if(error!=cudaSuccess) return {SetupStatus::DeviceFailure,"Optional plastic section allocation failed",error};
  for(std::size_t e=0;e<count;++e) {
    auto& curve=initial->parameters[e].curve;
    curve.plastic_strain=device_->curve_x+offsets[e];
    curve.yield_stress_pa=device_->curve_y+offsets[e];
  }
  error=cudaMemcpy(device_,initial.get(),sizeof(DeviceStorage),cudaMemcpyHostToDevice);
  if(error!=cudaSuccess) return {SetupStatus::DeviceFailure,"Collection plastic section initialization copy failed",error};
  element_count_=count; collection_=std::move(owned);
  return {SetupStatus::Success,"OK"};
}
} // namespace tl::fea::shell_batch_plasticity_detail
