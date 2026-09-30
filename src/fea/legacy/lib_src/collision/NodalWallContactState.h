#pragma once
#include "NodalWallContactArena.h"

namespace tlfea::contact {
struct NodalWallContactDevice::Impl {
  ~Impl();
  nodal_wall_device_detail::PreparedModel prepared;
  nodal_wall_device_detail::Storage device_shadow;
  nodal_wall_device_detail::Storage* device=nullptr;
  NodalWallDeviceConfig config;
  nodal_wall_device_detail::Control control;
  NodalWallDiagnostics available;
  tl::fea::NodalAssemblyView base_view;
  tl::fea::NodalStamp base_stamp; // Authenticated accepted scope; no owner retained.
  double rate=0;
  std::uint64_t last_epoch=0,last_attempt=0,last_candidate_attempt=0;
  cudaStream_t stream=nullptr;
  bool usable=true,has_base=false,has_results=false,has_stream=false;
  nodal_wall_device_detail::Control* DeviceControl() const noexcept {
    return reinterpret_cast<nodal_wall_device_detail::Control*>(reinterpret_cast<unsigned char*>(device)+
        offsetof(nodal_wall_device_detail::Storage,control));
  }
  NodalWallDiagnostics* DeviceDiagnostics() const noexcept {
    return reinterpret_cast<NodalWallDiagnostics*>(reinterpret_cast<unsigned char*>(device)+
        offsetof(nodal_wall_device_detail::Storage,result)+offsetof(nodal_wall_device_detail::ActiveResults,diagnostics));
  }
  NodalWallDeviceReport Check(cudaError_t);
  NodalWallDeviceReport ReadControl(cudaStream_t);
  NodalWallDeviceReport ReadResults(const NodalWallDiagnostics&);
  NodalWallDeviceReport FailAssembly(const tl::fea::NodalAssemblyView&,NodalWallDeviceReport);
};
} // namespace tlfea::contact
