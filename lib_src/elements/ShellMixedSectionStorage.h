#pragma once
#include "ShellBatchPlasticityStorage.h"
#include "ShellMixedSectionArenaLayout.h"

namespace tl::fea::shell_batch_plasticity_detail {
// Optional owner of one device allocation and startup-sized readback arrays.
// Source scope belongs to HostStorage; no accepted index, clock or commit here.
class MixedHostStorage {
 public:
  MixedHostStorage()=default;
  ~MixedHostStorage();
  MixedHostStorage(const MixedHostStorage&)=delete;
  MixedHostStorage& operator=(const MixedHostStorage&)=delete;
  static bool Forecast(std::size_t count,std::size_t points,std::size_t catalog_bytes,
      std::size_t device_cap,std::size_t host_cap,MixedLayout&,std::size_t& host_bytes,bool global=false) noexcept;
  SetupReport Initialize(const ShellBatchPlasticityBinding&,ShellBindingFamily,std::size_t,
      const MixedLayout&,bool execution=false);
  bool HasReadShape(unsigned slab,std::size_t count,const ShellBatchPlasticityBinding& catalog) const noexcept {
    return device_ && slab <= 1 && count == count_ && catalog.heterogeneous_sections();
  }
  ShellBindingFamily family() const noexcept { return family_; }
  SetupReport Read(unsigned slab,std::size_t count,cudaStream_t,const ShellBatchPlasticityBinding&,
      const ShellBatchOnePointSectionState* one_point=nullptr) noexcept;
  // Shape/source preflight only; the caller must validate the fresh selected slab.
  SetupReport CheckActivitySources(unsigned slab,std::size_t count,
      const ShellBatchPlasticityBinding&) const noexcept;
  MixedDeviceStorage* device() const noexcept { return device_; }
  std::size_t device_bytes() const noexcept { return device_?layout_.bytes:0; }
  const ShellBatchLayeredSection* staging() const noexcept { return output_.data(); }
 private:
  MixedDeviceStorage* device_=nullptr;
  MixedDeviceStorage header_;
  MixedLayout layout_;
  std::size_t count_=0;
  ShellBindingFamily family_=ShellBindingFamily::None;
  util::BoundedStartupArray<ShellBatchSectionState,0> plastic_;
  util::BoundedStartupArray<sections::ShellLayeredLaw1History,0> elastic_;
  util::BoundedStartupArray<ShellBatchLayeredSection,0> output_;
};
} // namespace tl::fea::shell_batch_plasticity_detail
