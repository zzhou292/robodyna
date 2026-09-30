#pragma once

#include "PlanarContactTypes.h"
#include "Q4ContactIntegrationTypes.h"

#include <array>

namespace tlfea::contact {
class PlanarWallGeometry;

constexpr std::uint32_t MaxQ4PlanarParents = 2;
constexpr double MaxQ4ProjectedAspectRatio = 1024;
constexpr double MinQ4SideRoundoffMultiple = 32;

struct PreparedQ4PlanarParent {
  SurfaceQ4 parent;
  Vec3 reference_projection[4]{};  // Copied wall X and exact accepted Y/Z.
  double projected_area = 0;
  Q4IntegralInterval area_enclosure;  // Exact-coordinate area roundoff scope.
  bool covered = false;
};
struct Q4PlanarReferenceView {
  const PreparedQ4PlanarParent* parents = nullptr;
  std::uint32_t parent_count = 0, global_node_count = 0;
  double wall_x = 0,wall_tolerance = 0;
};
struct Q4PreparedIntegration {
  bool covered = false;
  // Outside is successful geometry classification with an INVALID/default C2
  // input. The caller must skip integration; this is never a zero-force proof.
  Q4NormalIntegrationInput input;
};

// Borrowed host/device metadata copied by the later batch. No physical state,
// owner identity, clock, allocation, stream or wall-face pointer is retained.
// Reference metadata must originate from Q4PlanarGeometry::Initialize and remain
// immutable. Pointee lengths/lifetimes and nonaliasing are caller contracts.
// Mass arithmetic is checked using C1; C4 still owns actual state provenance.
// Mask 7 declares all translations fixed and requires zero normal velocity too.
TL_SURFACE_HD inline PlanarContactStatus ValidateQ4PlanarMotion(
    Q4PlanarReferenceView reference,const Q4SurfaceView& current,const Q4FixedYZMassView& mass) {
  if (!reference.parents || !reference.parent_count || reference.parent_count > MaxQ4PlanarParents ||
      !reference.global_node_count || !IsFinite(reference.wall_x) || !IsFinite(reference.wall_tolerance) ||
      reference.wall_tolerance <= 0) return PlanarContactStatus::NotInitialized;
  if (!current.parents || current.parent_count != reference.parent_count ||
      !current.positions.valid() || !current.velocities.valid() ||
      current.positions.node_count != reference.global_node_count ||
      current.velocities.node_count != reference.global_node_count || mass.node_count != reference.global_node_count)
    return PlanarContactStatus::InvalidInput;
  for (std::uint32_t p=0;p<reference.parent_count;++p) {
    const auto& saved=reference.parents[p]; const auto& parent=current.parents[p];
    const auto validation=q4_detail::ValidateParent(parent,reference.global_node_count);
    if (validation != Status::kOk || parent.feature_id != saved.parent.feature_id ||
        parent.parent_element_id != saved.parent.parent_element_id || parent.parent_face_id != saved.parent.parent_face_id ||
        parent.half_thickness != saved.parent.half_thickness) return PlanarContactStatus::UnsupportedGeometry;
    for (unsigned n=0;n<4;++n) {
      if (parent.nodes[n] != saved.parent.nodes[n]) return PlanarContactStatus::UnsupportedGeometry;
      const auto x=current.positions.at(parent.nodes[n]),v=current.velocities.at(parent.nodes[n]);
      if (!IsFinite(x) || !IsFinite(v)) return PlanarContactStatus::InvalidInput;
      if (x.y != saved.reference_projection[n].y || x.z != saved.reference_projection[n].z || v.y != 0 || v.z != 0)
        return PlanarContactStatus::UnsupportedMotion;
    }
    // Attempt 1 is only a local arithmetic probe, never an owner/trial receipt.
    NormalJacobian check;
    const auto status=BuildQ4NormalXJacobian(mass,parent,0,0,1,&check);
    if (status == Status::kUnsupportedInterpolation || status == Status::kNoDynamicDofs)
      return PlanarContactStatus::UnsupportedMotion;
    if (status != Status::kOk) return PlanarContactStatus::InvalidInput;
    for (unsigned n=0;n<4;++n)
      if (mass.translation_fixed_bits[parent.nodes[n]] == 7 && current.velocities.at(parent.nodes[n]).x != 0)
        return PlanarContactStatus::UnsupportedMotion;
  }
  return PlanarContactStatus::Ok;
}

// Validates ALL prepared parents before publishing one selected C2 input.
// Changing free normal X/velocity is supported; changed projected Y/Z is rejected.
// The C2 certificate uses projected_area as its defined FP64 reference measure.
// A claim against exact coordinate-derived area must also propagate the retained
// area_enclosure (all normal forces and potential scale linearly with area).
TL_SURFACE_HD inline PlanarContactStatus PrepareQ4PlanarIntegration(
    Q4PlanarReferenceView reference,const Q4SurfaceView& current,const Q4FixedYZMassView& mass,
    std::uint32_t parent_index,double stiffness_per_area,double max_penetration,std::uint64_t attempt,
    Q4PreparedIntegration* output) {
  if (!output || !IsFinite(stiffness_per_area) || stiffness_per_area <= 0 ||
      !IsFinite(max_penetration) || max_penetration <= 0 || !attempt)
    return PlanarContactStatus::InvalidInput;
  const auto status=ValidateQ4PlanarMotion(reference,current,mass);
  if (status != PlanarContactStatus::Ok) return status;
  if (parent_index >= reference.parent_count) return PlanarContactStatus::InvalidInput;
  const auto& saved=reference.parents[parent_index];
  Q4PreparedIntegration candidate;
  candidate.covered=saved.covered;
  if (candidate.covered)
    candidate.input={current,mass,parent_index,attempt,reference.wall_x,saved.projected_area,stiffness_per_area,max_penetration};
  *output=candidate;
  return PlanarContactStatus::Ok;
}

// Startup-only fixed-capacity preparation. Axis-aligned YZ rectangles use
// C1 natural order (+,+),(-,+),(-,-),(+,-). Shear, rotated projected rectangles,
// interior overlap and unresolved aspect/length scales are outside this gate.
// Exact edge adjacency is allowed, including actually shared physical nodes.
// Both display diagonals must yield identical finite-wall coverage. Their
// triangles never define the FE interpolation or mechanical contact forces.
class Q4PlanarGeometry {
 public:
  PlanarContactReport Initialize(const PlanarWallGeometry& wall,const Q4SurfaceView& reference,
                                const Q4FixedYZMassView& mass,double exposed_clearance);
  // Geometric reference preparation only: finite reference positions/velocities,
  // natural rectangular material measure, stable parents and finite-wall coverage.
  // This does not impose or invent physical mass/constraints, and admits finite
  // tangential reference velocity. Later adapters must validate their real mass
  // and motion contracts. The legacy Initialize above retains its fixed-YZ gate.
  PlanarContactReport InitializeReference(const PlanarWallGeometry& wall,const Q4SurfaceView& reference,
                                         double exposed_clearance);
  bool initialized() const noexcept { return parent_count_ != 0; }
  Q4PlanarReferenceView view() const noexcept {
    return {parents_.data(),parent_count_,global_node_count_,wall_x_,wall_tolerance_};
  }
 private:
  PlanarContactReport InitializeImpl(const PlanarWallGeometry& wall,const Q4SurfaceView& reference,
                                    const Q4FixedYZMassView* mass,double exposed_clearance);
  std::array<PreparedQ4PlanarParent,MaxQ4PlanarParents> parents_{};
  std::uint32_t parent_count_ = 0,global_node_count_ = 0;
  double wall_x_ = 0,wall_tolerance_ = 0;
};
}  // namespace tlfea::contact
