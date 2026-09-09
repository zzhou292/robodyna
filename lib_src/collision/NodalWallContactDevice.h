#pragma once
#include "NodalWallContact.h"
#include "PlanarWallBox.h"
#include "lib_src/solvers/FENodalState.h"
#include <memory>

namespace tlfea::contact {
constexpr unsigned MaxNodalWallDeviceParents=2,MaxNodalWallDeviceNodes=8;
constexpr std::size_t MaxNodalWallDeviceBytes=256*1024;
struct NodalWallDeviceConfig {
  tl::fea::NodalStamp owner;
  std::uint64_t configuration_id=0,qualification_id=0,wall_binding_id=0;
  NodalWallConfig law;
  double exposed_clearance=1e-6;
  std::size_t max_device_bytes=MaxNodalWallDeviceBytes;
};
enum class NodalWallDeviceStatus {
  Ok,InvalidInput,NotInitialized,ResourceLimit,WrongOwner,StaleAttempt,
  InvalidMass,GeometryFailure,PointFailure,Accuracy,AssemblyFailure,
  NonFiniteArithmetic,DeviceFailure
};
struct NodalWallDeviceReport {
  NodalWallDeviceStatus status=NodalWallDeviceStatus::InvalidInput;
  const char* message="Invalid nodal wall request";
  std::uint32_t node=UINT32_MAX,parent=UINT32_MAX;
  NodalWallReport point;
};
enum class NodalWallDevicePhase { Unspecified,AcceptedBase,PreparedCandidate };
struct NodalWallDiagnostics {
  std::uint64_t owner_id=0,configuration_id=0,qualification_id=0,wall_binding_id=0;
  std::uint64_t base_epoch=0,attempt=0;
  NodalWallDevicePhase phase=NodalWallDevicePhase::Unspecified;
  tl::fea::NodalTemporalScheme scheme=tl::fea::NodalTemporalScheme::StaggeredHalfKickStart;
  tl::fea::NodalVelocityPhase velocity_phase=tl::fea::NodalVelocityPhase::Collocated;
  double time=0,velocity_time=0,base_time=0,base_velocity_time=0,kick_dt=0;
  Q4CertifiedIntegral resultant,potential;
  Vec3 wall_reaction,wall_moment;
  double surface_power=0,maximum_penetration=0,stiffness_rate_bound=0;
  double base_potential=0,base_potential_error=0,potential_increment=0;
  double kick_work=0,kick_work_roundoff=0,drift_work=0,drift_work_roundoff=0;
  double conservative_defect=0,work_uncertainty=0,quadratic_work_upper=0;
  double wall_kick_impulse=0,wall_kick_impulse_error=0;
  Vec3 wall_kick_moment,wall_kick_moment_error;
  std::uint32_t node_count=0,parent_count=0;
  bool valid=false;
};
struct NodalWallDeviceResults {
  NodalWallDiagnostics diagnostics;
  NodalWallParentResult parents[MaxNodalWallDeviceParents]{};
  NodalWallPointResult nodes[MaxNodalWallDeviceNodes]{};
  std::uint64_t wall_face[MaxNodalWallDeviceNodes]{};
};

// Stateless, fixed-plane contributor, one/two Q4 parents in the existing <=64
// node owner. Zero offset/friction/damping; no rotations, shell mass/history,
// clock, stream or commit owner. Initial owner must use staggered half-kick
// timing at epoch zero. Free XYZ/fully-fixed incident nodes only; this does not
// extend QEPH's all-free binding. Contact model is NodalWallContactModel.
//
// Initialize copies immutable weights, mass/masks, fixed positions, actual wall
// and a host-certified finite motion envelope. Subsequent calls use only the
// returned owner stream/views. All laws and additive destinations validate
// before any physical assembly write. Every failure requires coordinator
// discard; assembly failure is sticky when valid channels exist. Raw device
// views/IDs are provenance, not authentication against fabricated writes.
//
// Base numerical forces are retained only for this attempt's work. Candidate
// evaluation never changes them. CopyResults is precommit staged readback;
// only the case may publish it after the SAME owner/material commit. Numerical
// rejection invalidates result scratch, not previously copied accepted output.
// Detected CUDA errors poison this participant. No device/context recovery is
// promised. The private all-active rate is NOT written to legacy owner rows;
// the case must separately qualify the full contact/history recurrence.
class NodalWallContactDevice {
 public:
  NodalWallContactDevice(); ~NodalWallContactDevice();
  NodalWallContactDevice(const NodalWallContactDevice&)=delete;
  NodalWallContactDevice& operator=(const NodalWallContactDevice&)=delete;
  NodalWallDeviceReport Initialize(const NodalWallDeviceConfig&,PlanarWallView,
      const NodalWallWeights&,VectorView initial_positions,const double* inverse_mass,
      const std::uint8_t* translation_fixed_bits,PlanarWallBox admitted_motion);
  NodalWallDeviceReport AssembleAccepted(const tl::fea::NodalAssemblyView&,NodalWallDiagnostics*);
  NodalWallDeviceReport EvaluateCandidate(const tl::fea::NodalPreparedView&,NodalWallDiagnostics*);
  NodalWallDeviceReport CopyResults(const NodalWallDiagnostics&,NodalWallDeviceResults*);
  void DiscardTrial() noexcept;
  tl::fea::NodalAllocationInfo allocations() const noexcept;
  double stiffness_rate_bound() const noexcept;
 private:
  struct Impl; std::unique_ptr<Impl> impl_;
};
} // namespace tlfea::contact
