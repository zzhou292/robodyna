// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../NodalWallContactDevice.h"
#include <type_traits>
namespace tlfea::contact::nodal_wall_mapped {
inline constexpr unsigned ObserverThreads=128,ObserverMaxBlocks=256;
// Trivial named storage permits CUDA shared records without constructors or
// an inactive union/raw-byte overlay. Arithmetic uses the existing certificate.
struct ObserverIntegral {
  double value,lower,upper,error;
  TL_SURFACE_HD Q4CertifiedIntegral Get() const noexcept {return {value,lower,upper,error};}
  TL_SURFACE_HD void Set(Q4CertifiedIntegral x) noexcept {value=x.value;lower=x.lower;upper=x.upper;error=x.error;}
};
struct ObserverSummary {
  ObserverIntegral resultant,potential;
  double signed_sum[7]; // reaction XYZ, moment XYZ, surface power
  double maximum_penetration,maximum_term;
  bool serial;
};
static_assert(std::is_trivial<ObserverSummary>::value,"Shared summaries have no dynamic initialization");
static_assert(sizeof(ObserverSummary)==144,"Forecast the complete typed observer record");
struct ObserverScratch {
  ObserverSummary* data=nullptr;
  std::size_t count=0;
};
TL_SURFACE_HD inline unsigned ObserverBlocks(std::size_t nodes) noexcept {
  if(!nodes || nodes>MaxVehicleNodalWallDeviceNodes) return 0;
  const auto blocks=1+(nodes-1)/ObserverThreads;
  return static_cast<unsigned>(blocks>ObserverMaxBlocks?ObserverMaxBlocks:blocks);
}
} // namespace tlfea::contact::nodal_wall_mapped
