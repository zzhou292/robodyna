#pragma once
#include "TestSupport.h"
extern "C" void law42_native_point(const double*,const double*,double,double,double*);
namespace law42_test {
inline void Native(const law::Parameters& p,const law::Input& x,double (&v)[13]) {
  const double parameters[]{p.mu_pa,p.poisson_ratio,p.density_kg_m3,p.tension_cutoff_pa};
  law42_native_point(parameters,x.total_strain,x.density_kg_m3,x.active,v);
}
inline void NativeCompare(const law::Parameters& p,const law::Input& x) {
  law::Result r;ASSERT_EQ(law::Update(p,x,r),law::Status::Ok);
  double actual[13],expected[13];Pack(p,r,actual);Native(p,x,expected);Compare(actual,expected);
}
}
