#pragma once
#include "T3History.h"
#include "T3Kinematics.h"

namespace tl::qualification::t3 {
struct ForceDiagnostics {
  double effective_thickness=0;       // Immutable force t, m.
  double native_sound_speed=0;        // m/s.
  double membrane_viscosity=0;        // Resolved native DM=.015; no T3 DN branch.
  double shear_factor=0;
  double transverse_shear_modulus=0;  // GS, Pa.
  double translational_stiffness=0;   // Native C3DT3 STI, N/m.
  double rotational_stiffness=0;      // Native C3DT3 STIR, N*m; not startup inertia.
  double unscaled_element_dt=0;       // DTFAC1(7)=1, s; diagnostic, not admission.
  std::array<double,2> internal_work_increment{};
};
struct ForceTrial {
  History proposed_history;
  Kinematics kinematics; // Original R2 38 observables before C3STRA3/C3DT3 mutation.
  std::array<Vec3,3> internal_force{};  // Native positive internal N, original order.
  std::array<Vec3,3> internal_couple{}; // Native positive internal world N*m.
  ForceDiagnostics diagnostics;
};
static_assert(sizeof(ForceTrial)<2048,"Keep prescribed native T3 packet bounded");
// Fixed centered LAW1/NPT0/ISH3N2/ITHK0/IDRIL0/DM.015. Complete native leaves
// plus explicitly labeled material/driver expressions; no dynamics or h=0.
// Exact bound reference, base time, next index. Complete base/output preserved
// on all failures. Caller accepts proposed history by value, without a clock.
// C3UPDT3 subtracts these positive internal values from the nodal RHS.
Status EvaluateForce(const Reference&,const History&,const PrescribedInterval&,ForceTrial&) noexcept;
namespace detail {
inline constexpr unsigned kForceValues=92; // R2 38 + history26 + loads18 + diagnostics10.
extern "C" void t3_r3_force(const double*,const double*,const double*,const double*,
    const double*,const double*,double*,int*);
// Test-only complete C3UPDT3 bridge: 1-based three-node map into four bounded
// physical nodes, caller seed RHS/stiffness. No standalone mechanics owner.
extern "C" void t3_r3_scatter(const int*,const double*,const double*,const double*,
    double*,double*,double*,double*);
}
} // namespace tl::qualification::t3
