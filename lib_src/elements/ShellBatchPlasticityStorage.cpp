#include "ShellBatchPlasticityStorage.h"
#include <cstring>
#include <new>

namespace tl::fea::shell_batch_plasticity_detail {
namespace {
bool Same(double a,double b) noexcept { return std::memcmp(&a,&b,sizeof a)==0; }
bool Same(ReferenceMaterial a,ReferenceMaterial b) noexcept {
  return Same(a.young,b.young)&&Same(a.nu,b.nu)&&Same(a.rho,b.rho);
}
SetupReport Cuda(cudaError_t error,const char* message) noexcept {
  return {error==cudaSuccess?SetupStatus::Success:SetupStatus::DeviceFailure,message,error};
}
}
HostStorage::~HostStorage() { if(device_) cudaFree(device_); }
SetupReport HostStorage::Initialize(const ShellBatchPlasticityConfig& c,const ReferenceMaterial* references,
    std::size_t count,std::size_t maximum_extra_device_bytes) {
  if(device_||!references||!count||count>MaxShellCollectionParents||!c.material_id||!c.curve_id||
     !c.curve.plastic_strain||!c.curve.yield_stress_pa||c.curve.count<2||c.curve.count>MaxCurvePoints)
    return {SetupStatus::InvalidInput,"Plasticity requires explicit IDs, curve and bounded source references"};
  if(sizeof(DeviceStorage)>maximum_extra_device_bytes)
    return {SetupStatus::ResourceLimit,"Optional plastic section allocation exceeds batch byte cap"};
  std::unique_ptr<DeviceStorage> initial(new(std::nothrow) DeviceStorage{});
  if(!initial) return {SetupStatus::ResourceLimit,"Plastic section startup staging allocation failed"};
  material_id_=c.material_id; curve_id_=c.curve_id; curve_count_=c.curve.count;
  element_count_=count; material_=references[0]; rate_=c.rate;
  for(std::size_t i=0;i<curve_count_;++i) {
    curve_x_[i]=initial->curve_x[i]=c.curve.plastic_strain[i];
    curve_y_[i]=initial->curve_y[i]=c.curve.yield_stress_pa[i];
  }
  const material::TabulatedShellPlasticityCurve owned{initial->curve_x,initial->curve_y,c.curve.count};
  for(std::size_t e=0;e<count;++e) {
    // One source material declaration; coefficients still come from each
    // authenticated reference, never from a caller's replacement rho/E/nu.
    if(!Same(material_,references[e]))
      return {SetupStatus::InvalidInput,"One plastic material declaration cannot replace heterogeneous references"};
    const auto status=material::PrepareTabulatedShellPlasticity(references[e].young,references[e].nu,
        references[e].rho,owned,rate_,initial->parameters[e]);
    if(status!=material::TabulatedShellPlasticityStatus::Ok)
      return {SetupStatus::InvalidInput,"Source material or tabulated curve is not admissible"};
  }
  auto error=cudaMalloc(reinterpret_cast<void**>(&device_),sizeof(DeviceStorage));
  if(error!=cudaSuccess) return Cuda(error,"Optional plastic section allocation failed");
  for(std::size_t e=0;e<count;++e)
    initial->parameters[e].curve={device_->curve_x,device_->curve_y,c.curve.count};
  error=cudaMemcpy(device_,initial.get(),sizeof(DeviceStorage),cudaMemcpyHostToDevice);
  return Cuda(error,"Optional plastic section initialization copy failed");
}
cudaError_t HostStorage::Read(unsigned slab,std::size_t count,cudaStream_t stream) noexcept {
  if(!device_||slab>1||count!=element_count_) return cudaErrorInvalidValue;
  auto error=cudaMemcpyAsync(staging_.data(),device_->section[slab],count*sizeof(ShellBatchSectionState),
      cudaMemcpyDeviceToHost,stream);
  return error==cudaSuccess?cudaStreamSynchronize(stream):error;
}
bool HostStorage::SameMaterialScope(const HostStorage& b) const noexcept {
  if(collection_||b.collection_)
    return collection_&&b.collection_&&collection_->SameScope(*b.collection_);
  if(material_id_!=b.material_id_||curve_id_!=b.curve_id_||curve_count_!=b.curve_count_||
     !Same(material_,b.material_)||rate_.enabled!=b.rate_.enabled||
     !Same(rate_.cowper_symonds_c_per_s,b.rate_.cowper_symonds_c_per_s)||
     !Same(rate_.cowper_symonds_p,b.rate_.cowper_symonds_p)||
     !Same(rate_.cutoff_hz,b.rate_.cutoff_hz)) return false;
  const auto bytes=curve_count_*sizeof(double);
  return std::memcmp(curve_x_.data(),b.curve_x_.data(),bytes)==0&&
         std::memcmp(curve_y_.data(),b.curve_y_.data(),bytes)==0;
}
} // namespace tl::fea::shell_batch_plasticity_detail
