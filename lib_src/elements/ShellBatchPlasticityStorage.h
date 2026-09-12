#pragma once
#include "ShellBatchPlasticity.h"
#include "ShellBatchPlasticityBinding.h"
#include "ShellCollectionLimits.h"
#include "ShellPlasticArenaLayout.h"
#include "ShellBatchLayeredSection.h"
#include "ShellBatchFailureBinding.h"
#include "lib_utils/BoundedStartupArray.h"
#include <cuda_runtime_api.h>
#include <memory>
#include <type_traits>

namespace tl::fea { class ShellPhysicalBinding; }

namespace tl::fea::shell_batch_plasticity_detail {
constexpr std::size_t MaxCurvePoints=MaxShellPlasticityCurvePoints;
struct ReferenceMaterial { double young=0,nu=0,rho=0; };
enum class SetupStatus { Success,InvalidInput,ResourceLimit,DeviceFailure,NonfiniteResult };
struct SetupReport {
  SetupStatus status=SetupStatus::InvalidInput;
  const char* message="Invalid plasticity configuration";
  cudaError_t cuda_status=cudaSuccess;
};

// Host-owned declaration and readback staging exist only for the opt-in path.
// Two section arrays use the original shell slab index; this class has no
// accepted index, stamp, pending flag, commit, discard or independent clock.
class FailureHostStorage;
struct FailureDeviceStorage;
class MixedHostStorage;
struct MixedDeviceStorage;
class OnePointHostStorage;
struct OnePointDeviceStorage;
class HostStorage {
 public:
  HostStorage();
  ~HostStorage();
  HostStorage(const HostStorage&)=delete;
  HostStorage& operator=(const HostStorage&)=delete;
  SetupReport Initialize(const ShellBatchPlasticityConfig&,const ReferenceMaterial*,
      std::size_t count,std::size_t maximum_extra_device_bytes,std::size_t maximum_extra_host_bytes);
  SetupReport InitializeCollection(const ShellBatchPlasticityBinding&,const ShellBatchBinding&,
      ShellBindingFamily,std::size_t count,std::size_t maximum_extra_device_bytes,std::size_t maximum_extra_host_bytes,
      bool vehicle_shared_inventory=false);
  cudaError_t Read(unsigned slab,std::size_t count,cudaStream_t) noexcept;
  DeviceStorage* device() const noexcept { return device_; }
  std::size_t device_bytes() const noexcept;
  MixedDeviceStorage* mixed_device() const noexcept;
  OnePointDeviceStorage* one_point_device() const noexcept;
  bool one_point_sections() const noexcept { return bool(one_point_); }
  const ShellBatchPlasticityBinding* section_catalog() const noexcept { return Collection(); }
  bool heterogeneous_sections() const noexcept { return bool(mixed_); }
  SetupReport ReadSections(unsigned slab,std::size_t count,cudaStream_t,double time=0) noexcept;
  // Fresh material staging and failure-source preflight only. The mapped
  // activity caller must validate the actual device failure slab before use.
  SetupReport ReadActivitySections(unsigned slab,std::size_t count,cudaStream_t,double time) noexcept;
  // Mapped QEPH compact query: no typed union staging and no one-point role.
  SetupReport CheckActivitySectionSources(unsigned slab,std::size_t count) const noexcept;
  SetupReport CheckActivityFailureSources(unsigned slab,std::size_t count) const noexcept;
  FailureDeviceStorage* failure_device() const noexcept;
  const ShellBatchFailureState* failure_staging() const noexcept;
  std::size_t failure_device_bytes() const noexcept;
  bool failure_sections() const noexcept { return bool(failure_); }
  const ShellBatchFailureBinding* failure_binding() const noexcept;
  SetupReport InitializeFailureCollection(const ShellBatchFailureBinding&,const ShellBatchBinding&,
      ShellBindingFamily,std::size_t,std::size_t device_cap,std::size_t host_cap,
      const ShellBatchFailureLimits&,bool vehicle);
  // The caller retains and has already budgeted this exact physical source.
  // Rigid rows have no material or point-history availability.
  SetupReport InitializeMappedCollection(const ShellPhysicalBinding&,ShellBindingFamily,
      std::size_t,std::size_t device_cap,std::size_t host_cap,const ShellBatchFailureLimits&);
  static bool ForecastFailureSections(std::size_t count,std::size_t points,std::size_t binding_bytes,
      std::size_t device_cap,std::size_t host_cap,const ShellBatchFailureLimits&,std::size_t& host_bytes,
      bool one_point=false,std::size_t* device_bytes=nullptr) noexcept;
  const ShellBatchLayeredSection* section_staging() const noexcept;
  // Conservative payload peak: arena initialization + readback + retained
  // curve/catalog data + per-parent rebase offsets. No allocation or input read.
  static bool Forecast(std::size_t count,std::size_t points,std::size_t catalog_bytes,
      std::size_t device_cap,std::size_t host_cap,Layout& layout,std::size_t& host_bytes) noexcept;
  static bool ForecastSections(std::size_t count,std::size_t points,std::size_t catalog_bytes,
      std::size_t device_cap,std::size_t host_cap,std::size_t& host_bytes) noexcept;
  const ShellBatchSectionState* staging() const noexcept { return staging_.data(); }
  bool SameMaterialScope(const HostStorage&) const noexcept;
 private:
  SetupReport ReadSectionsBeforeFailure(unsigned slab,std::size_t count,cudaStream_t,double time) noexcept;
  SetupReport InitializeFailureCollectionImpl(const ShellBatchFailureBinding&,const ShellBatchBinding&,
      ShellBindingFamily,std::size_t,std::size_t,std::size_t,
      const ShellBatchFailureLimits&,bool vehicle,bool execution,bool retained_physical);
  DeviceStorage* device_=nullptr;
  DeviceStorage device_header_; // Rebased address values; never host-dereference device fields.
  Layout layout_;
  std::uint64_t material_id_=0,curve_id_=0;
  std::size_t curve_count_=0,element_count_=0;
  ReferenceMaterial material_{};
  material::TabulatedShellPlasticityRate rate_{};
  util::BoundedStartupArray<double,0> curve_x_,curve_y_;
  util::BoundedStartupArray<ShellBatchSectionState,0> staging_;
  std::unique_ptr<ShellBatchPlasticityBinding> collection_; // New path only; full owned scope.
  std::unique_ptr<FailureHostStorage> failure_;
  const ShellBatchPlasticityBinding* Collection() const noexcept;
  bool SameFailureScope(const HostStorage&) const noexcept;
  std::unique_ptr<MixedHostStorage> mixed_; // Explicit mode only, same caller-provided slab index.
  std::unique_ptr<OnePointHostStorage> one_point_; // Optional T3 payload, no selector.
  SetupReport InitializeSections(const ShellBatchPlasticityBinding&,const ShellBatchBinding&,
      ShellBindingFamily,std::size_t,std::size_t,std::size_t,bool);
};
inline bool SameMaterialScope(const std::unique_ptr<HostStorage>& a,
    const std::unique_ptr<HostStorage>& b) noexcept {
  return (!a&&!b)||(a&&b&&a->SameMaterialScope(*b));
}
} // namespace tl::fea::shell_batch_plasticity_detail
