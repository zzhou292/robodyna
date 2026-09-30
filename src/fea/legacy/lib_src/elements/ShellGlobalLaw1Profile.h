// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/math/Quaternion.h"

#if defined(__CUDACC__)
#define TL_GLOBAL_LAW1_HD __host__ __device__
#else
#define TL_GLOBAL_LAW1_HD
#endif
namespace tl::fea {
// Resolved coefficient policy, not the raw keyword NIP or a layer count.
enum class ShellLaw1Thickness : unsigned char { Reference, Accepted };
struct ShellGlobalLaw1Profile {
  ShellLaw1Thickness thickness=ShellLaw1Thickness::Reference;
  // Native length unit in metres, solely for the CNCOEF3B EM20 thickness
  // floor. Independent of QEPH's projection working-length descriptor.
  double coefficient_working_length_m=1.;
};
namespace shell_global_law1 {
// Pinned MYREAL8 constant_mod: EP20=EP19*TEN, EM20=ONE/EP20.
// Owning native coefficient qualification compares the actual Fortran constant.
inline constexpr double NativeThicknessFloor=1./1e20;
TL_GLOBAL_LAW1_HD inline bool Valid(const ShellGlobalLaw1Profile& p) noexcept {
  const double floor=NativeThicknessFloor*p.coefficient_working_length_m;
  return (p.thickness==ShellLaw1Thickness::Reference||p.thickness==ShellLaw1Thickness::Accepted)&&
      tl::math::Finite(p.coefficient_working_length_m)&&p.coefficient_working_length_m>0&&
      tl::math::Finite(floor)&&floor>0;
}
// Shared with virgin mapped STI: initial accepted thickness equals the
// declared reference, but CNCOEF3B's ITHK1 floor still applies at time zero.
TL_GLOBAL_LAW1_HD inline double QephCoefficientThickness(const ShellGlobalLaw1Profile& p,
    double reference,double accepted) noexcept {
  return p.thickness==ShellLaw1Thickness::Reference?reference:
      ::fmax(accepted,NativeThicknessFloor*p.coefficient_working_length_m);
}
TL_GLOBAL_LAW1_HD inline double T3CoefficientThickness(const ShellGlobalLaw1Profile& p,
    double reference,double accepted) noexcept {
  return p.thickness==ShellLaw1Thickness::Reference?reference:accepted;
}
} // namespace shell_global_law1
} // namespace tl::fea
#undef TL_GLOBAL_LAW1_HD
