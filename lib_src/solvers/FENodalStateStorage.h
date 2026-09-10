#pragma once

#include "FENodalState.h"
#include <vector>

namespace tl::fea {
namespace nodal_detail {
struct RigidStorage;
enum class Phase { Idle, Assembling, Sealed, AwaitingValidation, Ready };
struct Control {
  stability::RowBounds rows;
  NodalAssemblyResult assembly;
  stability::StepLimit limit;
  NodalStatus status = NodalStatus::Ok;
  std::uint32_t node = UINT32_MAX;
};

// Shared velocity-first translation arithmetic for the legacy and optional
// rotational operations. Constraint bits are WORLD x/y/z; only trial buffers
// are written. The caller records the first invalid node and discards a failed
// attempt. reaction_xyz is optional, and records forces on the constrained node.
template<bool Capture=false>
TL_SURFACE_HD inline bool AdvanceTranslationNodeWithKick(
    const double* accepted, double* trial, const double* force, double inverse_mass,
    std::uint8_t fixed_bits, std::uint32_t node, std::uint32_t n, double h, double kick_dt,
    double* reaction_xyz = nullptr,double* acceleration_xyz = nullptr) {
  for (unsigned axis = 0; axis < 3; ++axis) {
    const auto j = 3*node+axis;
    const bool fixed = (fixed_bits & (1u << axis)) != 0;
    const double acceleration = fixed ? 0 : inverse_mass*force[axis*n+node];
    const double velocity = fixed ? 0 : accepted[3*n+j]+kick_dt*acceleration;
    const double position = fixed ? accepted[j] : accepted[j]+h*velocity;
    trial[3*n+j] = velocity; trial[j] = position;
    if (reaction_xyz) reaction_xyz[j] = fixed ? -force[axis*n+node] : 0;
    if constexpr(Capture) acceleration_xyz[j]=acceleration;
    if (!tlfea::contact::IsFinite(acceleration) || !tlfea::contact::IsFinite(velocity) ||
        !tlfea::contact::IsFinite(position)) return false;
  }
  return true;
}
TL_SURFACE_HD inline bool AdvanceTranslationNode(
    const double* accepted, double* trial, const double* force, double inverse_mass,
    std::uint8_t fixed_bits, std::uint32_t node, std::uint32_t n, double h,
    double* reaction_xyz = nullptr) {
  return AdvanceTranslationNodeWithKick(accepted,trial,force,inverse_mass,fixed_bits,node,n,h,h,reaction_xyz);
}
}  // namespace nodal_detail

// Private runtime storage shared by the non-owning advance operations. Optional
// rigid history is an 18-double/group tail of both existing state slabs; one
// accepted/trial swap publishes the physical nodes and groups together.
struct FENodalState::Impl {
  ~Impl();
  NodalReport Check(cudaError_t);
  NodalReport SynchronizeControl();
  NodalReport Reject(NodalStatus, const char*, std::uint32_t = UINT32_MAX);
  bool Matches(std::uint64_t owner, std::uint64_t epoch, std::uint64_t trial) const;
  NodalAssemblyView AcceptedAssemblySources() const noexcept;
  NodalReport AdvanceSealedNodal(std::uint64_t owner, std::uint64_t epoch, std::uint64_t trial,
                                const NodalStepAdmission&, NodalTemporalScheme, bool with_rigid_groups=false);
  cudaError_t LaunchRigidAdvance(double maximum_angle);
  NodalReport StageRigidSnapshot(const double* state);
  NodalReport StageNodalSnapshot(const double* state, bool prepared);
  NodalStateConfig config;
  NodalStamp stamp;
  NodalAllocationInfo allocation;
  std::uint64_t attempt = 0;
  std::uint64_t pending_qualification = 0;
  double candidate_time = 0;
  double candidate_velocity_time = 0, candidate_kick_dt = 0;
  bool usable = true;
  bool has_rotations = false, has_component_constraints = false;
  std::size_t state_values = 0;
  nodal_detail::Phase phase = nodal_detail::Phase::Idle;
  cudaStream_t stream = nullptr;
  double *accepted = nullptr, *trial = nullptr, *scratch = nullptr, *inverse = nullptr;
  std::uint8_t* fixed = nullptr;
  nodal_detail::Control* control = nullptr;
  nodal_detail::Control host_control;
  // Legacy slab: x3/v3. Extended slab: x3/v3/omega3/q4/reactionF3/reactionC3.
  // No allocation or host vector growth after startup.
  std::vector<double> staging;
  std::vector<std::uint8_t> constraint_staging;
  std::unique_ptr<nodal_detail::RigidStorage> rigid_groups;
};
}  // namespace tl::fea
