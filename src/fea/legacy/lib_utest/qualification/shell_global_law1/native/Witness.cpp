// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NativeReference.h"
#include "lib_utest/qualification/native/qeph/NativeQephBridge.h"
#include "lib_utest/qualification/native/t3/T3EngineContext.h"
#include <cmath>
#include <stdexcept>
namespace tl::qualification::global_law1_native {
extern "C" void global_law1_qeph_coefficient(const double*,const double*,const int*,double*);
extern "C" void global_law1_t3_coefficient(const double*,const double*,const int*,double*);
extern "C" double global_law1_em20();
namespace {
void Check(double reference,double accepted,int ithk) {
  if(!std::isfinite(reference)||reference<=0||!std::isfinite(accepted)||accepted<0||(ithk!=0&&ithk!=1))
    throw std::invalid_argument("Invalid native global LAW1 coefficient witness input");
}
}
double QephThickness(double reference,double accepted,int ithk) {
  Check(reference,accepted,ithk);double value=0;
  const std::lock_guard<std::mutex> lock(q::detail::NativeContext());
  global_law1_qeph_coefficient(&reference,&accepted,&ithk,&value);return value;
}
double T3Thickness(double reference,double accepted,int ithk) {
  Check(reference,accepted,ithk);double value=0;
  const std::lock_guard<std::mutex> lock(t::detail::NativeEngineContext());
  global_law1_t3_coefficient(&reference,&accepted,&ithk,&value);return value;
}
double NativeEm20() {
  const std::lock_guard<std::mutex> lock(q::detail::NativeContext());return global_law1_em20();
}
}
