// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../../math/Fixed3.h"
#include <cstdint>

#if defined(__CUDACC__)
#define TL_BEAM18_HD __host__ __device__
#else
#define TL_BEAM18_HD
#endif

namespace tl::fea::beam18 {
using tl::math::Vec3;
enum class Status { Success, InvalidInput, UnsupportedScope, DegenerateGeometry, NonfiniteResult };
enum class WorkingUnits { SI, TonneMillimetreSecond };
enum class Profile { Unspecified, CircularFourPointStoredZero };
enum class OrientationBranch { ThirdNode, GlobalY, GlobalZ };
struct Input {
  std::uint64_t source_element_id = 0, source_part_id = 0;
  std::uint64_t source_section_id = 0, source_material_id = 0;
  std::uint64_t source_node_id[3]{};
  // Original working coordinates, never reconstructed from rounded SI values.
  Vec3 position[3]{};
  WorkingUnits units = WorkingUnits::SI;
  Profile profile = Profile::Unspecified;
  double radius = 0, density = 0, young = 0, poisson = 0;
  // Original RT1/RR1/RT2/RR2 and LOCAL; this profile releases no endpoint DOF.
  unsigned release[4]{};
  unsigned local = 2;
};
struct SectionPoint { double y = 0, z = 0, area = 0; };
struct Section {
  SectionPoint point[4]{};
  double area = 0, inertia_y = 0, inertia_z = 0, inertia_x = 0;
  double membrane_damping = 0, flexural_damping = 0;
};
struct Geometry {
  double length = 0, length_m = 0;
  Vec3 orientation_seed{}; // PEVECI SKEW, not a complete orthonormal frame.
  Vec3 endpoint_m[2]{};
  OrientationBranch orientation_branch = OrientationBranch::ThirdNode;
};
struct NativeMass {
  double endpoint_mass = 0, endpoint_total_inertia = 0;
  double translation_stiffness = 0, rotation_stiffness = 0;
  double interface_stiffness = 0; // PMASS STP for explicit I7STIFS=1.
  // Observed native intermediates, not a physical/added inertia partition.
  double facdt = 0, phii = 0, kphi = 0, axial_coefficient = 0;
  double axial_inertia_term = 0, section_inertia_term = 0, torsional_floor = 0;
};
struct Endpoint {
  double mass_kg = 0, native_total_inertia_kg_m2 = 0;
  double translation_stiffness_n_m = 0, rotation_stiffness_nm = 0;
  double interface_stiffness_n_m = 0;
};
class Reference {
 public:
  TL_BEAM18_HD bool prepared() const noexcept { return prepared_; }
  TL_BEAM18_HD const Input& input() const noexcept { return input_; }
  TL_BEAM18_HD const Section& section() const noexcept { return section_; }
  TL_BEAM18_HD const Geometry& geometry() const noexcept { return geometry_; }
  TL_BEAM18_HD const NativeMass& native_mass() const noexcept { return native_mass_; }
  TL_BEAM18_HD const Endpoint& endpoint() const noexcept { return endpoint_; }
 private:
  Input input_{};
  Section section_{};
  Geometry geometry_{};
  NativeMass native_mass_{};
  Endpoint endpoint_{};
  bool prepared_ = false;
  friend TL_BEAM18_HD Status InitializeReference(const Input&, Reference&) noexcept;
};
} // namespace tl::fea::beam18
