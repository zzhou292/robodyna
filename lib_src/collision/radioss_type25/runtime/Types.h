// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../selection/lifecycle/Types.h"
#include "../assembly/DeviceTypes.h"
#include "../search/Types.h"
#include "../candidates/InventoryTypes.h"
namespace tlfea::contact::radioss_type25 {
namespace lifecycle=selection::lifecycle;
namespace runtime_detail {
// Preserve literal native ISKEW (selection treats1 as its global-axis branch).
// Without a full frame binder this transaction admits nonzero skew only for
// entirely free/fixed translations: those subspaces are basis independent.
TL_MATH_HOST_DEVICE inline bool SupportedConstraint(int code,int skew) noexcept {
  return code>=0&&code<=7&&skew>=0&&(skew==0||code==0||code==7);
}
}
enum class TransactionStatus { Ok,InvalidInput,UnsupportedProfile,ResourceLimit,
  NotInitialized,AlreadyInitialized,StaleAttempt,SourceMismatch,OwnerFailure,
  PublicationFailure,NumericalFailure,DeviceFailure,Unusable };
struct TransactionReport {
  TransactionStatus status=TransactionStatus::InvalidInput;
  const char* message="Invalid native contact transaction";
  std::size_t row=SIZE_MAX,occurrence=SIZE_MAX;
  selection::Status selection_status=selection::Status::Ok;
};
// Borrowed immutable startup input. Scalar contact fields are native working
// units. Physical nodes are in the actual ShellPhysicalBinding domain order.
// Captured source data is permitted in qualification fixtures, never a generic
// source producer. A successful transaction does not qualify its caller's parser.
struct FixedMainSource {
  std::uint64_t source_id=0,topology_generation=0;
  lifecycle::SourceView selection;
  std::size_t primary_main_count=0;
  const std::uint64_t* primary_parent_ids=nullptr; // Authentic physical shell IDs.
  const double* primary_curvature=nullptr;
  double margin=0,gap_load=0,drad=0;
  unsigned force_packet_size=0; // Actual native NVSIZ; never CUDA block size.
  int native_workers=0; // First numerical profile requires exactly one.
};
struct TransactionConfig {
  UnitScale units;
  lifecycle::Profile lifecycle;
  ResolvedNormalConfig normal;
  FrictionControls friction;
  NativeFrictionCoefficients friction_coefficients;
  assembly::Controls assembly;
};
inline candidates::Limits FixedMainInventoryLimits() noexcept {
  candidates::Limits limits;limits.max_pairs=65536;limits.max_tasks=65536;return limits;
}
struct TransactionLimits {
  // Raw pairs and optimized cache records have separate caps. Complete native
  // counts are admitted before fill; there is no nearest-K/truncation fallback.
  candidates::Limits inventory=FixedMainInventoryLimits();
  search::Limits maintenance;
  std::size_t optimized_candidates=65536,sliding_entries=65536;
  std::size_t max_device_bytes=std::size_t{1}<<30,max_host_bytes=std::size_t{128}<<20;
};
// Immutable descriptive startup identity; not a physical/accepted receipt.
struct TransactionSourceInfo {
  std::uint64_t source_id=0,topology_generation=0,source_generation=0;
  std::size_t nodes=0,secondaries=0,primary_mains=0,expanded_mains=0;
  bool available=false;
};
struct TransactionForecast {
  std::size_t device_bytes=0,host_bytes=0,startup_host_bytes=0;
  std::size_t runtime_device_bytes=0,inventory_device_bytes=0,maintenance_device_bytes=0,incidence_device_bytes=0;
  std::size_t raw_pair_capacity=0,optimized_capacity=0,sliding_capacity=0;
};
struct TransactionDiagnostics {
  std::uint64_t raw_candidates=0,optimized_candidates=0,kept_occurrences=0,active_forces=0;
  bool reference_rebuilt=false;
  // Native per-attempt sums in source occurrence order, converted once to SI.
  double elastic_energy=0,damping_work=0,friction_work=0;
};
} // namespace tlfea::contact::radioss_type25
