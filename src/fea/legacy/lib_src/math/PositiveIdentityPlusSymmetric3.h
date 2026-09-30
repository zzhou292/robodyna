// SPDX-License-Identifier: AGPL-3.0-or-later
// Shared represented-tensor admission, extracted without arithmetic changes
// from materials/law42/Stress.h. This is a guard, not an eigenvalue solver.
#pragma once
#include "Quaternion.h"
#if defined(__CUDACC__)
#define TL_POSITIVE_IDENTITY_HD __host__ __device__
#else
#define TL_POSITIVE_IDENTITY_HD
#endif
namespace tl::math {
TL_POSITIVE_IDENTITY_HD inline bool PositiveIdentityPlusSymmetric3(const double (&a)[6]) noexcept {
  // Sylvester criterion on I+a; tensor order XX/YY/ZZ/XY/YZ/ZX.
  const double x = 1+a[0];
  const double y = 1+a[1];
  const double z = 1+a[2];
  const double minor = x*y-a[3]*a[3];
  const double determinant = x*(y*z-a[4]*a[4])-a[3]*(a[3]*z-a[4]*a[5])+
                             a[5]*(a[3]*a[4]-y*a[5]);
  return x>0 && minor>0 && determinant>0 && Finite(minor) && Finite(determinant);
}
} // namespace tl::math
#undef TL_POSITIVE_IDENTITY_HD
