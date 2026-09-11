#pragma once
#include "lib_src/elements/solid18/law44/Force.h"
#include "lib_utest/qualification/solid18_law44_reference/TestSupport.h"
#include "lib_utest/qualification/solid_law44_point/TestSupport.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <gtest/gtest.h>

namespace rear_force_test {
namespace s = tl::fea::solid18;
namespace law = s::law44;
inline law::Material Material() { return law44_solid_test::Parameters(); }
inline law::Reference Reference(bool collapsed = false) {
  const auto input = collapsed ? rear18_test::Collapsed() : rear18_test::Cube();
  law::Reference result;
  EXPECT_EQ(law::InitializeReference(input,result),s::Status::Success);
  return result;
}
inline law::PrescribedInterval Step(const law::Reference& reference, const law::History& accepted,
                                    double gamma = .001, double gamma_rate = 1) {
  law::PrescribedInterval result;
  result.base_time_s = accepted.stamp().time_s;
  result.dt_s = 1e-5;
  result.sample_index = accepted.stamp().sample_index+1;
  for (unsigned n = 0; n < 8; ++n) {
    const auto x = reference.input().position_m[n];
    result.position_endpoint_m[n] = {x.x+gamma*x.y,x.y,x.z};
    result.velocity_midpoint_m_s[n] = {gamma_rate*x.y,0,0};
  }
  return result;
}
template<class T> std::array<unsigned char,sizeof(T)> Bytes(const T& value) {
  std::array<unsigned char,sizeof(T)> bytes{};
  std::memcpy(bytes.data(),&value,sizeof(T));
  return bytes;
}
}
