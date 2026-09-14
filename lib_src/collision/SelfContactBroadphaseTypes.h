#pragma once
#include "SurfaceContactTypes.h"
#include <cstddef>

namespace tlfea::contact {
enum class SelfContactBoundsMotion {
  Current,
  LinearNodalEndpoints,
  ConservativeSweptParentBounds,
};
enum class SelfContactBroadphaseStatus {
  Ok, InvalidInput, NotInitialized, AlreadyInitialized, ResourceLimit,
  InvalidBounds, PairCapacity, DeviceFailure
};
struct SelfContactBroadphaseReport {
  SelfContactBroadphaseStatus status = SelfContactBroadphaseStatus::Ok;
  const char* message = "OK";
  std::uint32_t parent = UINT32_MAX;
  std::uint64_t required_pairs = 0;
};
struct SelfContactBroadphaseLimits {
  std::size_t max_parents = 2048, max_nodes = 2048, max_pairs = 65536;
  std::size_t max_device_bytes = 64u << 20, max_host_bytes = 64u << 20;
};
struct SelfContactBroadphaseForecast {
  std::size_t parents = 0, nodes = 0, pair_capacity = 0;
  std::size_t sort_temp_bytes = 0, scan_temp_bytes = 0, pair_sort_temp_bytes = 0;
  std::size_t device_bytes = 0, retained_source_bytes = 0;
  std::size_t conservative_bounds_host_bytes = 0;
  std::size_t owned_host_bytes = 0, startup_scratch_bytes = 0, startup_host_bytes = 0;
};
struct SelfContactBroadphasePreflight {
  SelfContactBroadphaseReport report;
  SelfContactBroadphaseForecast forecast;
};
// Caller-computed outward bounds for one complete parent trajectory.  This
// mode is intentionally separation-only: broadphase overlap is not evidence
// of contact or crossing.
struct SelfContactSweptParentBounds {
  Vec3 lower;
  Vec3 upper;
};
struct SelfContactBroadphaseInput {
  VectorView current;
  VectorView endpoint; // Required only for the explicitly linear nodal sweep.
  SelfContactBoundsMotion motion = SelfContactBoundsMotion::Current;
  unsigned axis = 0;
  // Required only for ConservativeSweptParentBounds.  Host values are copied
  // into fixed startup-owned staging before the device sweep.
  const SelfContactSweptParentBounds* swept_parent_bounds = nullptr;
  std::size_t swept_parent_count = 0;
};
// Canonical unsigned key encodes two distinct S0 parent ordinals (a < b).
// Keys are strictly increasing, independent of the sweep axis. They carry no
// owner/attempt authentication and are not a mechanical feature/contact list.
using SelfContactPairKey = std::uint64_t;
TL_SURFACE_HD inline std::uint32_t FirstSurfaceParent(SelfContactPairKey key) { return key >> 32; }
TL_SURFACE_HD inline std::uint32_t SecondSurfaceParent(SelfContactPairKey key) { return key & UINT32_MAX; }
struct SelfContactBroadphasePairs {
  const SelfContactPairKey* device_keys = nullptr;
  std::uint64_t count = 0;
  bool complete = false;
};
} // namespace tlfea::contact
