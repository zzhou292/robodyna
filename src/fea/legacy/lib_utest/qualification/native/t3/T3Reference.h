#pragma once

#include "lib_src/math/Fixed3.h"
#include <array>
#include <cstdint>

namespace tl::qualification::t3 {
using Vec3 = tl::math::Vec3;
using Matrix3 = tl::math::Matrix3;

enum class Status { kSuccess, kInvalidInput, kUnsupportedGeometry, kNativeFailure, kNonfiniteResult };

// Frozen startup-only binary64 domain, independent of wall orientation. These
// numerical guards are not shell-quality, timestep or dynamics admission.
inline constexpr double kMaximumCoordinate = 1e6;    // m, each world component
inline constexpr double kMinimumEdge = 1e-9;         // m
inline constexpr double kMaximumEdge = 1e3;          // m
inline constexpr double kMinimumEdgeRatio = 1e-6;
inline constexpr double kMinimumNormalizedTwiceArea = 1e-6;
inline constexpr double kAcosBoundaryMargin = 1e-12;

struct ReferenceInput {
  std::array<Vec3, 3> position{};  // Three actual nodes, original cyclic order.
  std::array<std::uint64_t, 3> node_ids{{0, 1, 2}};
  double density = 7890;          // kg/m^3
  double thickness = .001648;     // m
  double young_modulus = 200e9;   // Retained experiment metadata, unused by startup mass.
  double poisson_ratio = .3;      // Retained domain 0 <= nu < .5.
};

struct ReferenceData {
  ReferenceInput input;
  Matrix3 frame;                  // Row-major; columns e1(edge1), e2, e3 in world.
  double area = 0;                // Complete native starter C3EVEC3, m^2.
  std::array<Vec3, 3> local_position{};
  std::array<double, 3> angle_cosine{};  // Exact native ACOS arguments, unclamped.
  std::array<double, 3> angle_weight{};  // Each ACOS(argument)/PI, unnormalized.
  std::array<double, 3> nodal_mass{};
  std::array<double, 3> physical_inertia{};  // Diagnostic (EM*t*t/12)*weight.
  std::array<double, 3> added_inertia{};     // Diagnostic (EM*(A/4.5))*weight.
  std::array<double, 3> isotropic_inertia{}; // Native XI*weight; never sum partitions.
  double element_mass = 0;
  double element_isotropic_inertia = 0;
  double element_physical_inertia = 0;
  double element_added_inertia = 0;
  double characteristic_length = 0;        // Selected C3DERII 2*A/max_edge, m.
  // C3DERII ISMSTR=-1 startup slots PX1/PY1/PY2 are ZERO. Engine current
  // derivatives are a separate later operation; do not populate them here.
  std::array<double, 3> startup_derivative{};
};

class Reference {
 public:
  bool prepared() const noexcept { return prepared_; }
  const ReferenceData& data() const noexcept { return data_; }
 private:
  ReferenceData data_{};
  bool prepared_ = false;
  friend Status Initialize(const ReferenceInput&, Reference&) noexcept;
};
static_assert(sizeof(Reference)<1024,"Revisit bounded native T3 reference storage");

// Qualification oracle only. Complete native C3EVEC3 is invoked; C3DERII,
// C3INMAS and SPMD_MSIN are explicitly selected source-expression adapters,
// not complete starter-driver invocations. IGTYP1/ISMSTR-1/unscaled uniform
// centered section; no force, stiffness, material history or time owner.
// Every edge and normalization is prechecked against the constants above;
// native ACOS arguments are checked before ACOS. No floor, clamp, repair or
// angle renormalization is applied. Positive mass/inertia outputs must remain
// finite and representable. Failure preserves the entire caller Reference.
// Internally serialized private native startup context; no shared QEPH COMMON.
Status Initialize(const ReferenceInput& input, Reference& output) noexcept;
}  // namespace tl::qualification::t3
