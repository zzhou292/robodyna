#pragma once
// Test-only independent long-double/atan2 oracle shared by synthetic and
// authenticated source-triangle checks. No production/native equations here.
#include "T3Reference.h"
#include <gtest/gtest.h>
#include <algorithm>
#include <array>
#include <cmath>

namespace tl::qualification::t3::test {
namespace native=tl::qualification::t3;
constexpr long double kBudget=2e-12L;  // Frozen before the first execution.

inline void Near(double actual,long double expected,long double dimension) {
  EXPECT_LE(std::abs(static_cast<long double>(actual)-expected),
            kBudget*(dimension+std::abs(expected)));
}
using Wide=std::array<long double,3>;
inline Wide Difference(Vec3 a,Vec3 b) {
  return {static_cast<long double>(a.x)-b.x,static_cast<long double>(a.y)-b.y,
          static_cast<long double>(a.z)-b.z};
}
inline long double Dot(Wide a,Wide b) { return a[0]*b[0]+a[1]*b[1]+a[2]*b[2]; }
inline long double CrossNorm(Wide a,Wide b) {
  return std::hypot(a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]);
}
struct Oracle {
  long double area=0,length=0,mass=0,physical=0,added=0,total=0;
  std::array<long double,3> weight{};
};
inline Oracle Independent(const ReferenceInput& input) {
  Oracle result;
  const auto& x=input.position;
  result.area=CrossNorm(Difference(x[1],x[0]),Difference(x[2],x[0]))/2;
  long double maximum=0;
  for (unsigned n=0;n<3;++n) {
    const auto a=Difference(x[(n+1)%3],x[n]),b=Difference(x[(n+2)%3],x[n]);
    maximum=std::max(maximum,std::sqrt(Dot(a,a)));
    // Independent atan2 of world edges, not donor law-of-cosines/ACOS.
    result.weight[n]=std::atan2(CrossNorm(a,b),Dot(a,b))/std::acos(-1.L);
  }
  result.length=2*result.area/maximum;
  result.mass=static_cast<long double>(input.density)*input.thickness*result.area;
  result.physical=result.mass*input.thickness*input.thickness/12;
  result.added=result.mass*result.area/4.5L;
  result.total=result.physical+result.added;  // Independent real-valued oracle only.
  return result;
}
inline void Check(const Reference& reference) {
  ASSERT_TRUE(reference.prepared());
  const auto& data=reference.data(); const auto expected=Independent(data.input);
  Near(data.area,expected.area,expected.area);
  Near(data.characteristic_length,expected.length,expected.length);
  Near(data.element_mass,expected.mass,expected.mass);
  Near(data.element_physical_inertia,expected.physical,expected.physical);
  Near(data.element_added_inertia,expected.added,expected.added);
  Near(data.element_isotropic_inertia,expected.total,expected.total);
  long double weight_sum=0,mass_sum=0,inertia_sum=0;
  for (unsigned n=0;n<3;++n) {
    SCOPED_TRACE(n);
    Near(data.angle_weight[n],expected.weight[n],1);
    Near(data.nodal_mass[n],expected.mass*expected.weight[n],expected.mass*expected.weight[n]);
    Near(data.physical_inertia[n],expected.physical*expected.weight[n],expected.physical*expected.weight[n]);
    Near(data.added_inertia[n],expected.added*expected.weight[n],expected.added*expected.weight[n]);
    Near(data.isotropic_inertia[n],expected.total*expected.weight[n],expected.total*expected.weight[n]);
    EXPECT_GT(data.nodal_mass[n],0); EXPECT_GT(data.isotropic_inertia[n],0);
    EXPECT_LE(std::abs(data.angle_cosine[n]),1-native::kAcosBoundaryMargin);
    EXPECT_EQ(data.startup_derivative[n],0);  // ISMSTR=-1, not current C3DERI3.
    EXPECT_EQ(data.local_position[n].z,0);
    weight_sum+=data.angle_weight[n]; mass_sum+=data.nodal_mass[n]; inertia_sum+=data.isotropic_inertia[n];
  }
  Near(static_cast<double>(weight_sum),1,1);
  Near(static_cast<double>(mass_sum),expected.mass,expected.mass);
  Near(static_cast<double>(inertia_sum),expected.total,expected.total);
  for (unsigned i=0;i<3;++i) for (unsigned j=0;j<3;++j) {
    long double dot=0;
    for (unsigned k=0;k<3;++k) dot+=static_cast<long double>(data.frame.v[3*k+i])*data.frame.v[3*k+j];
    Near(static_cast<double>(dot),i==j?1:0,1);
  }
  const auto& x=data.input.position;
  for (unsigned n=0;n<3;++n) {
    const auto world=Difference(x[n],x[0]);
    const auto& local=data.local_position[n];
    for (unsigned k=0;k<3;++k) {
      const auto reconstructed=static_cast<long double>(data.frame.v[3*k])*local.x+
          static_cast<long double>(data.frame.v[3*k+1])*local.y;
      Near(static_cast<double>(reconstructed),world[k],expected.length);
    }
  }
}

}  // namespace tl::qualification::t3::test
