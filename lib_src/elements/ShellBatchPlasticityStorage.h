#pragma once
#include "ShellBatchPlasticity.h"
#include "ShellBatchPlasticityBinding.h"
#include "ShellCollectionLimits.h"
#include <cuda_runtime_api.h>
#include <array>
#include <memory>
#include <type_traits>

namespace tl::fea::shell_batch_plasticity_detail {
constexpr std::size_t MaxCurvePoints=MaxShellPlasticityCurvePoints;
struct ReferenceMaterial { double young=0,nu=0,rho=0; };
struct DeviceStorage {
  double curve_x[MaxCurvePoints]{},curve_y[MaxCurvePoints]{};
  sections::PointParameters parameters[MaxShellCollectionParents]{};
  ShellBatchSectionState section[2][MaxShellCollectionParents]{};
};
static_assert(std::is_trivially_copyable_v<DeviceStorage>);
static_assert(sizeof(DeviceStorage)<128*1024,"Bounded optional plastic section allocation");
enum class SetupStatus { Success,InvalidInput,ResourceLimit,DeviceFailure };
struct SetupReport {
  SetupStatus status=SetupStatus::InvalidInput;
  const char* message="Invalid plasticity configuration";
  cudaError_t cuda_status=cudaSuccess;
};

// Host-owned declaration and readback staging exist only for the opt-in path.
// Two section arrays use the original shell slab index; this class has no
// accepted index, stamp, pending flag, commit, discard or independent clock.
class HostStorage {
 public:
  HostStorage()=default;
  ~HostStorage();
  HostStorage(const HostStorage&)=delete;
  HostStorage& operator=(const HostStorage&)=delete;
  SetupReport Initialize(const ShellBatchPlasticityConfig&,const ReferenceMaterial*,
      std::size_t count,std::size_t maximum_extra_device_bytes);
  SetupReport InitializeCollection(const ShellBatchPlasticityBinding&,const ShellBatchBinding&,
      ShellBindingFamily,std::size_t count,std::size_t maximum_extra_device_bytes);
  cudaError_t Read(unsigned slab,std::size_t count,cudaStream_t) noexcept;
  DeviceStorage* device() const noexcept { return device_; }
  const ShellBatchSectionState* staging() const noexcept { return staging_.data(); }
  bool SameMaterialScope(const HostStorage&) const noexcept;
 private:
  DeviceStorage* device_=nullptr;
  std::uint64_t material_id_=0,curve_id_=0;
  std::size_t curve_count_=0,element_count_=0;
  ReferenceMaterial material_{};
  material::TabulatedShellPlasticityRate rate_{};
  std::array<double,MaxCurvePoints> curve_x_{},curve_y_{};
  std::array<ShellBatchSectionState,MaxShellCollectionParents> staging_{};
  std::unique_ptr<ShellBatchPlasticityBinding> collection_; // New path only; full owned scope.
};
inline bool SameMaterialScope(const std::unique_ptr<HostStorage>& a,
    const std::unique_ptr<HostStorage>& b) noexcept {
  return (!a&&!b)||(a&&b&&a->SameMaterialScope(*b));
}
} // namespace tl::fea::shell_batch_plasticity_detail
