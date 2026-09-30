// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include <array>
#include <mutex>
#include <stdexcept>
namespace beam18_property_test {
extern "C" void beam18_property_area_native(const double*,double*,int*);
inline std::array<double,4> Native(double radius) {
  static std::mutex native;
  std::lock_guard<std::mutex> serial(native);
  std::array<double,4> result{};
  int status=1;
  beam18_property_area_native(&radius,result.data(),&status);
  if(status)throw std::runtime_error("Native circular property area rejected");
  return result;
}
}
