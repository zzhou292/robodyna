// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Types.h"
#include "lib_src/math/SymmetricEigen3.h"
namespace tl::material::law42::detail {
TL_LAW42_HD inline double Maximum(double a,double b) noexcept {return a>b?a:b;}
TL_LAW42_HD inline double Minimum(double a,double b) noexcept {return a<b?a:b;}
TL_LAW42_HD inline bool PositiveStretchTensor(const double (&a)[6]) noexcept {
  // Sylvester's criterion on represented C=I+strain. An eigensolver's rounded
  // zero eigenvalue must not admit a collapsed or inverted stretch tensor.
  const double x=1+a[0],y=1+a[1],z=1+a[2];
  const double minor=x*y-a[3]*a[3];
  const double determinant=x*(y*z-a[4]*a[4])-a[3]*(a[3]*z-a[4]*a[5])+
                           a[5]*(a[3]*a[4]-y*a[5]);
  return x>0&&minor>0&&determinant>0&&tl::math::Finite(minor)&&tl::math::Finite(determinant);
}
TL_LAW42_HD inline void RotatePrincipal(const tl::math::Matrix3& v,
                                       const double (&p)[3],double (&stress)[6]) noexcept {
  constexpr unsigned row[6]{0,1,2,0,1,2},col[6]{0,1,2,1,2,0};
  // Follow native cyclic sum order for YY/ZZ/YZ/ZX as well as XX/XY.
  constexpr unsigned start[6]{0,1,2,0,1,2};
  for(unsigned q=0;q<6;++q) {
    stress[q]=0;
    for(unsigned j=0;j<3;++j) {
      const unsigned k=(start[q]+j)%3;
      stress[q]+=v.v[3*row[q]+k]*v.v[3*col[q]+k]*p[k];
    }
  }
}
} // namespace tl::material::law42::detail
