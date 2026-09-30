// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "NativeOracle.h"
#include "lib_utils/BoundedArena.h"
#include "lib_src/math/ScalarBits.h"
#include <gtest/gtest.h>
#include <stdexcept>
namespace nodal_correction_test {
struct Fixture {
  std::vector<double> original{2,3,5,7,11,13,17,19,23,29};
  std::vector<c::Solid> solids;
  std::vector<std::uint32_t> secondaries;
  tl::util::HostArena scratch;
  c::Forecast forecast;
  std::vector<double> output;
  c::Input Input()const{return {original.data(),original.size(),solids.empty()?nullptr:solids.data(),solids.size(),
      secondaries.empty()?nullptr:secondaries.data(),secondaries.size()};}
  c::Output Output(){return {output.data(),output.size()};}
  c::Report Prepare(){auto r=c::Preflight(Input(),{},forecast);if(r.status!=c::Status::Ok)return r;
    if(!scratch.Initialize(forecast.scratch_bytes))throw std::runtime_error("Correction fixture scratch");output.assign(original.size(),97);return r;}
  c::Report Run(){return c::Apply(Input(),{},scratch.data(),scratch.bytes(),Output());}
};
inline c::Solid Solid(double bulk,double controlled,int control=1){return {{0,1,2,3,4,5,6,6},control,bulk,controlled};}
inline void Same(const std::vector<double>& a,const std::vector<double>& b){ASSERT_EQ(a.size(),b.size());
  for(std::size_t i=0;i<a.size();++i){SCOPED_TRACE(i);EXPECT_TRUE(tl::math::SameScalarBits(a[i],b[i]));}}
}
