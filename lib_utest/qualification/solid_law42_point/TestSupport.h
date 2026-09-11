// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/materials/law42/Update.h"
#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
namespace law42_test {
namespace law=tl::material::law42;
inline law::Parameters Material(double cutoff=1e26,double nu=.463) {
  law::Parameters p;
  EXPECT_EQ(law::Prepare(24e6,nu,1980,cutoff,p),law::Status::Ok);
  return p;
}
inline law::Input Stretch(double a,double b,double c) {
  law::Input x;
  x.total_strain[0]=a*a-1;x.total_strain[1]=b*b-1;x.total_strain[2]=c*c-1;
  x.density_kg_m3=1980/(a*b*c);
  return x;
}
inline law::Input Rotate(const law::Input& x,double angle) {
  const double c=std::cos(angle),s=std::sin(angle);
  const Eigen::Matrix3d r=(Eigen::Matrix3d()<<c,-s,0,s,c,0,0,0,1).finished();
  const Eigen::Matrix3d a=(Eigen::Matrix3d()<<x.total_strain[0],x.total_strain[3]/2,x.total_strain[5]/2,
    x.total_strain[3]/2,x.total_strain[1],x.total_strain[4]/2,
    x.total_strain[5]/2,x.total_strain[4]/2,x.total_strain[2]).finished();
  const Eigen::Matrix3d v=r*a*r.transpose();
  law::Input next=x;
  next.total_strain[0]=v(0,0);next.total_strain[1]=v(1,1);next.total_strain[2]=v(2,2);
  next.total_strain[3]=2*v(0,1);next.total_strain[4]=2*v(1,2);next.total_strain[5]=2*v(2,0);
  return next;
}
inline void Near(double actual,double expected,double scale) {
  ASSERT_TRUE(std::isfinite(actual));ASSERT_TRUE(std::isfinite(expected));
  EXPECT_NEAR(actual,expected,2e-10*std::max(std::abs(expected),scale));
}
inline void Pack(const law::Parameters& p,const law::Result& r,double (&v)[13]) {
  for(unsigned k=0;k<6;++k)v[k]=r.stress_pa[k];
  v[6]=r.maximum_principal_stress_pa;v[7]=r.minimum_principal_stress_pa;v[8]=r.active;
  v[9]=r.sound_speed_m_s;v[10]=r.hourglass_tangent_factor;v[11]=r.material_viscosity_pa_s;v[12]=p.bulk_pa;
}
inline void Compare(const double (&a)[13],const double (&b)[13]) {
  double stress_scale=1;
  for(unsigned k=0;k<8;++k)stress_scale=std::max(stress_scale,std::abs(b[k]));
  for(unsigned k=0;k<13;++k) {
    SCOPED_TRACE(k);
    ASSERT_TRUE(std::isfinite(a[k]));ASSERT_TRUE(std::isfinite(b[k]));
    // Both independent cubic spectral routines have O(sqrt(epsilon)) root
    // sensitivity at repeated eigenvalues. Analytic tests retain 2e-10.
    const double spectral_bound=2*std::sqrt(std::numeric_limits<double>::epsilon());
    EXPECT_NEAR(a[k],b[k],(2e-10+spectral_bound)*std::max(std::abs(b[k]),k<8?stress_scale:1));
  }
}
} // namespace law42_test
