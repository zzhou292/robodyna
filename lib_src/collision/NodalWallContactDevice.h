#pragma once
#include "NodalWallContact.h"
#include "PlanarWallBox.h"
#include "lib_src/solvers/FENodalState.h"
#include "lib_src/elements/ShellCollectionLimits.h"
#include <memory>

namespace tlfea::contact {
constexpr unsigned MaxNodalWallDeviceParents=tl::fea::MaxShellCollectionParents;
constexpr unsigned MaxNodalWallDeviceNodes=tl::fea::MaxShellCollectionNodes;
constexpr std::size_t MaxNodalWallDeviceBytes=512*1024;
constexpr unsigned MaxActiveNodalWallDeviceParents=1024,MaxActiveNodalWallDeviceNodes=2048;
constexpr std::size_t MaxActiveNodalWallDeviceBytes=8*1024*1024;
constexpr std::size_t MaxNodalWallHostBytes=16*1024*1024;
constexpr unsigned MaxVehicleNodalWallDeviceParents=524288,MaxVehicleNodalWallDeviceNodes=524288;
constexpr std::size_t MaxVehicleNodalWallDeviceBytes=2ULL*1024*1024*1024;
constexpr std::size_t MaxVehicleNodalWallHostBytes=2ULL*1024*1024*1024;
enum class NodalWallDeviceProfile { Legacy,Vehicle };
struct NodalWallDeviceLimits {
  std::size_t parents=MaxNodalWallDeviceParents,nodes=MaxNodalWallDeviceNodes,
      global_nodes=MaxNodalWallDeviceNodes;
  NodalWallDeviceProfile profile=NodalWallDeviceProfile::Legacy;
  static constexpr NodalWallDeviceLimits Vehicle() noexcept {
    return {MaxVehicleNodalWallDeviceParents,MaxVehicleNodalWallDeviceNodes,MaxVehicleNodalWallDeviceNodes,
      NodalWallDeviceProfile::Vehicle};
  }
};
struct NodalWallDeviceConfig {
  tl::fea::NodalStamp owner;
  std::uint64_t configuration_id=0,qualification_id=0,wall_binding_id=0;
  NodalWallConfig law;
  double exposed_clearance=1e-6;
  std::size_t max_device_bytes=MaxNodalWallDeviceBytes;
  NodalWallDeviceLimits limits;
  std::size_t max_host_bytes=1024*1024;
};
enum class NodalWallDeviceStatus {
  Ok,InvalidInput,NotInitialized,ResourceLimit,WrongOwner,StaleAttempt,
  InvalidMass,GeometryFailure,PointFailure,Accuracy,AssemblyFailure,
  NonFiniteArithmetic,DeviceFailure,StepTooLarge,ParticipationFailure
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
// Caller-owned output ranges, each with its exact active extent. Every range
// must be disjoint from all others and from the expected diagnostics/view.
// Storage remains caller-owned; no allocation or accepted-state publication.
struct NodalWallDeviceResultView {
  NodalWallDiagnostics* diagnostics=nullptr;
  NodalWallParentResult* parents=nullptr;
  NodalWallPointResult* nodes=nullptr;
  std::uint64_t* wall_face=nullptr;
  std::size_t parent_capacity=0,node_capacity=0;
};

// Stateless, fixed-plane contributor. Defaults admit 128 parents/nodes; explicit
// legacy count/byte limits admit <=1024 parents and <=2048 global/incident nodes.
// Vehicle() explicitly admits <=524288 of each with separately supplied byte caps.
// Exactly Q4/4 and
// T3/3 family/arity pairs use the immutable owning A0/4 and A0/3 shares.
// Zero offset/friction/damping; no direct couple, shell mass/history,
// clock, stream or commit owner. Initial owner must use staggered half-kick
// timing at epoch zero. Free XYZ/fully-fixed incident nodes only; this does not
// extend either shell batch's all-free binding or admit mixed force-feedback
// dynamics. Contact model is NodalWallContactModel. Native shell mass/J stays
// separate from contact area; the caller supplies the actual union mass.
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
  // Rigid-group coupling requires these live-owner/token overloads. Raw
  // grouped views reject; native mass rates remain local diagnostics only.
  NodalWallDeviceReport AssembleAccepted(tl::fea::FENodalState&,const tl::fea::NodalAssemblyView&,NodalWallDiagnostics*);
  NodalWallDeviceReport EvaluateCandidate(const tl::fea::NodalPreparedView&,NodalWallDiagnostics*);
  NodalWallDeviceReport EvaluateCandidate(tl::fea::FENodalState&,const tl::fea::NodalTrialToken&,
      const tl::fea::NodalPreparedView&,NodalWallDiagnostics*);
  NodalWallDeviceReport CopyResults(const NodalWallDiagnostics&,NodalWallDeviceResults*);
  NodalWallDeviceReport CopyResults(const NodalWallDiagnostics&,const NodalWallDeviceResultView&);
  void DiscardTrial() noexcept;
  tl::fea::NodalAllocationInfo allocations() const noexcept;
  double stiffness_rate_bound() const noexcept;
 private:
  struct Impl; std::unique_ptr<Impl> impl_;
  NodalWallDeviceReport AssembleAcceptedImpl(tl::fea::FENodalState*,const tl::fea::NodalAssemblyView&,NodalWallDiagnostics*);
  NodalWallDeviceReport EvaluateCandidateImpl(tl::fea::FENodalState*,const tl::fea::NodalTrialToken*,
      const tl::fea::NodalPreparedView&,NodalWallDiagnostics*);
};
} // namespace tlfea::contact
