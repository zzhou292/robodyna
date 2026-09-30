// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../QephBatch.h"
#include "../../ShellBatchFailure.h"
#include "../../ShellBatchLayeredSection.h"
#include "../../ShellGlobalLaw1Profile.h"
#include "MaterialParameters.h"
#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace tl::fea::qeph {
// A diagnostic value record, never a checkpoint or a publishable trial. No
// pointer, string, vector or device view belongs to its serialized value scope.
struct RejectedReportValues {
  BatchStatus status=BatchStatus::InvalidInput;
  std::uint32_t element=UINT32_MAX,node=UINT32_MAX;
  Status element_status=Status::kSuccess;
  NodalStatus nodal_status=NodalStatus::Ok;
};
struct RejectedOwnerStamp {
  std::uint64_t owner_id=0,epoch=0;
  std::size_t node_count=0;
  double time=0,fixed_dt=0,velocity_time=0;
  NodalTemporalScheme temporal_scheme=NodalTemporalScheme::VelocityFirst;
  NodalVelocityPhase velocity_phase=NodalVelocityPhase::Collocated;
};
struct RejectedCandidateMetadata {
  RejectedReportValues original;
  RejectedOwnerStamp owner;
  BatchDiagnostics accepted,candidate; // Candidate.valid remains false.
};
enum class RejectedCandidateRoute : std::uint8_t {
  Unspecified,PlainForce,PlasticSection,MixedSection,FailureSection,RigidSkin
};
struct RejectedSourceIds {
  bool available=false;
  std::uint64_t parent=0,part=0,material=0,section=0;
};
struct RejectedCandidateInput {
  RejectedCandidateMetadata metadata;
  RejectedSourceIds source;
  QephBatchElement element; // nodes[] are physical owner-domain indices.
  // reference.input.node_ids retain their own original native reference meaning.
  ForceTrial accepted_force; // Complete incoming cache, not the failed output.
  PrescribedInterval interval; // Exact gathered prepared x/v/omega and clock.
  RejectedCandidateRoute route=RejectedCandidateRoute::Unspecified;
  bool mapped=false,has_mixed=false,has_failure=false;
  ShellSectionLaw law=ShellSectionLaw::Unspecified;
  ShellGlobalLaw1Profile global_law1;
  RejectedMaterialParameters plastic_parameters;
  double curve_strain[MaxShellPlasticityCurvePoints]{};
  double curve_stress_pa[MaxShellPlasticityCurvePoints]{};
  ShellBatchSectionState accepted_plastic;
  material::ShellElasticLaw1PointParameters elastic_parameters;
  sections::ShellLayeredLaw1History accepted_elastic;
  ShellFailurePolicy failure_policy=ShellFailurePolicy::None;
  sections::ConstantFailureParameters constant_failure;
  sections::ShellLayeredTab1Parameters tab1_failure;
  ShellBatchFailureState accepted_failure;
};
enum class RejectedCaptureStatus : std::uint8_t {
  Captured,InvalidInput,NoRejectedCandidate,StaleTrial,UnsupportedFailure,
  InvalidSource,ResourceLimit,DeviceFailure
};
struct RejectedCaptureReport {
  RejectedCaptureStatus status=RejectedCaptureStatus::InvalidInput;
  const char* message="Invalid rejected-candidate diagnostic request";
  cudaError_t cuda_status=cudaSuccess;
  bool metadata_available=false;
  RejectedCandidateMetadata metadata;
};
struct RejectedCandidateForecast {
  std::size_t retained_host_bytes=sizeof(RejectedCandidateInput);
  // Includes caller record, failure-atomic staging, header/parameter readback,
  // the returned report and local host replay scratch. App serialization is extra.
  std::size_t peak_host_bytes=64u<<10;
  std::size_t device_bytes=0;
};
constexpr RejectedCandidateForecast ForecastRejectedCandidateCapture() noexcept { return {}; }
struct RejectedReplayResult {
  Status operator_status=Status::kInvalidInput;
  bool force_available=false,mapped_result_checked=false,mapped_result_valid=false;
  ForceTrial force;
  ShellBatchSectionState plastic;
  sections::ShellLayeredLaw1History elastic;
  ShellBatchFailureState failure;
};
// false means malformed diagnostic shape, preserving output. true reports the
// actual existing operator outcome, including rejection. Failed output packets
// are not exposed. Mapped result validation is a separate reported outcome.
bool ReplayRejectedCandidate(const RejectedCandidateInput&,RejectedReplayResult*) noexcept;
static_assert(std::is_trivially_copyable_v<RejectedCandidateInput>);
static_assert(sizeof(RejectedCandidateInput)<=32u<<10,"Bounded diagnostic input");
} // namespace tl::fea::qeph
