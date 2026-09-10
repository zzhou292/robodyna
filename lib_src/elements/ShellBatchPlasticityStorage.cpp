#include "ShellBatchPlasticityStorage.h"
#include "ShellMixedSectionStorage.h"
#include <cstring>
#include <new>
#include <stdexcept>

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
HostStorage::HostStorage()=default;
HostStorage::~HostStorage() { if(device_) cudaFree(device_); }
bool HostStorage::Forecast(std::size_t count,std::size_t points,std::size_t catalog_bytes,
    std::size_t device_cap,std::size_t host_cap,Layout& output,std::size_t& host_bytes) noexcept {
  Layout layout;
  if(!host_cap||host_cap>MaxVehicleShellResidentHostBytes||!layout.Initialize(count,points,device_cap)) return false;
  util::BoundedArenaLayout host(host_cap); util::ArenaRegion ignored;
  if(!host.Append<unsigned char>(sizeof(HostStorage),ignored)||!host.Append<unsigned char>(layout.bytes,ignored)||
     !host.Append<ShellBatchSectionState>(count,ignored)||!host.Append<double>(2*points,ignored)||
     !host.Append<std::size_t>(count,ignored)||!host.Append<unsigned char>(catalog_bytes,ignored)||
     !host.Append<unsigned char>(3*64,ignored)) return false;
  output=layout; host_bytes=host.bytes(); return true;
}
SetupReport HostStorage::Initialize(const ShellBatchPlasticityConfig& c,const ReferenceMaterial* references,
    std::size_t count,std::size_t maximum_extra_device_bytes,std::size_t maximum_extra_host_bytes) try {
  if(device_||mixed_||!references||!count||count>MaxVehicleShellResidentParents||!c.material_id||!c.curve_id||
     !c.curve.plastic_strain||!c.curve.yield_stress_pa||c.curve.count<2||c.curve.count>MaxCurvePoints)
    return {SetupStatus::InvalidInput,"Plasticity requires explicit IDs, curve and bounded source references"};
  Layout layout; std::size_t host_bytes=0;
  if(!Forecast(count,c.curve.count,0,maximum_extra_device_bytes,maximum_extra_host_bytes,layout,host_bytes))
    return {SetupStatus::ResourceLimit,"Optional plastic section exceeds active byte budgets"};
  util::HostArena arena;
  if(!arena.Initialize(layout.bytes)) return {SetupStatus::ResourceLimit,"Plastic section startup staging allocation failed"};
  auto* initial=layout.Construct(arena);
  if(!initial) return {SetupStatus::ResourceLimit,"Plastic section startup layout is invalid"};
  material_id_=c.material_id; curve_id_=c.curve_id; curve_count_=c.curve.count;
  element_count_=count; material_=references[0]; rate_=c.rate;
  curve_x_.Resize(curve_count_); curve_y_.Resize(curve_count_); staging_.Resize(count);
  for(std::size_t i=0;i<curve_count_;++i) {
    curve_x_[i]=initial->curve_x[i]=c.curve.plastic_strain[i];
    curve_y_[i]=initial->curve_y[i]=c.curve.yield_stress_pa[i];
  }
  const material::TabulatedShellPlasticityCurve owned{initial->curve_x,initial->curve_y,c.curve.count};
  for(std::size_t e=0;e<count;++e) {
    if(!Same(material_,references[e]))
      return {SetupStatus::InvalidInput,"One plastic material declaration cannot replace heterogeneous references"};
    const auto status=material::PrepareTabulatedShellPlasticity(references[e].young,references[e].nu,
        references[e].rho,owned,rate_,initial->parameters[e]);
    if(status!=material::TabulatedShellPlasticityStatus::Ok)
      return {SetupStatus::InvalidInput,"Source material or tabulated curve is not admissible"};
  }
  DeviceStorage* candidate=nullptr;
  auto error=cudaMalloc(reinterpret_cast<void**>(&candidate),layout.bytes);
  if(error!=cudaSuccess) return Cuda(error,"Optional plastic section allocation failed");
  const auto header=layout.Rebase(*initial,candidate);
  for(std::size_t e=0;e<count;++e)
    initial->parameters[e].curve={header.curve_x,header.curve_y,c.curve.count};
  *initial=header;
  error=cudaMemcpy(candidate,arena.data(),layout.bytes,cudaMemcpyHostToDevice);
  if(error!=cudaSuccess) { cudaFree(candidate); return Cuda(error,"Optional plastic section initialization copy failed"); }
  device_=candidate; device_header_=header; layout_=layout;
  return {SetupStatus::Success,"OK"};
} catch(const std::bad_alloc&) { return {SetupStatus::ResourceLimit,"Plastic section host allocation failed"}; }
  catch(const std::length_error&) { return {SetupStatus::ResourceLimit,"Plastic section host size overflow"}; }
cudaError_t HostStorage::Read(unsigned slab,std::size_t count,cudaStream_t stream) noexcept {
  if(!device_||slab>1||count!=element_count_) return cudaErrorInvalidValue;
  auto error=cudaMemcpyAsync(staging_.data(),device_header_.section[slab],count*sizeof(ShellBatchSectionState),
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
