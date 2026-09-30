#pragma once
#include "T3Reference.h"

namespace tl::qualification::t3 {
struct PrescribedInterval {
  std::array<Vec3,3> position{};       // Endpoint world coordinates, actual nodes.
  std::array<Vec3,3> velocity{};       // Supplied world midpoint samples, m/s.
  std::array<Vec3,3> angular_velocity{}; // World midpoint samples, rad/s.
  double base_time=0,dt=0;
  std::uint64_t sample_index=1;        // Caller endpoint identity; no internal clock.
};

struct Kinematics {
  Matrix3 frame;                      // Native current frame, world basis columns.
  std::array<Vec3,3> local_position{}; // Native x2/y2/x3/y3; node0 origin.
  std::array<double,3> derivative{};   // C3DERI3 PX1/PY1/PY2, m.
  // Source material order XX,YY,XY,YZ,ZX,KXX,KYY,KXY. The first five raw
  // values are m^2/s, the last three m/s. C3CURV3 shear correction is included.
  std::array<double,8> raw_rate{};
  std::array<double,8> normalized_rate{}; // Diagnostic raw/AREA, never R3 input.
  std::array<double,3> corrected_velocity_difference{}; // VX13/VX23/VY12, m/s.
  double area=0,characteristic_length=0,area_scale=0;
  double base_time=0,position_time=0,velocity_time=0,dt=0;
  std::uint64_t sample_index=0;
  bool valid=false;
};
static_assert(sizeof(Kinematics)<1024,"Revisit native T3 kinematic value capacity");

// Complete pinned C3COOR3/C3EVEC3/C3DERI3/C3DEFO3/C3CURV3 invocation.
// Fixed branch ISH3N2/IFRAM_OLD1/IREP0/IDRAPE0/IGTYP1/ISMSTR-1/IRESP2,
// explicit IMPL_S0, OFF/OFFG1. No material, history, force or time advancement.
// Current coordinates obey the R1 numerical geometry bounds; actual native
// local Y3 must exceed 32*EM15 before C3DERI3, preventing its floor activation.
// Finite input/results, representable quarter step and strict midpoint timing
// are required. Private serialized engine context; failure preserves output
// and reference bytes. Repeated samples own no accepted/rejected state.
Status EvaluatePrescribed(const Reference&,const PrescribedInterval&,Kinematics&) noexcept;
}  // namespace tl::qualification::t3
