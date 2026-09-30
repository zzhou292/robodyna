// SPDX-License-Identifier: MIT
#pragma once
#include "Fixture.h"
#include "lib_src/constraints/tied_shell/TiedSearchBounds.h"
extern "C" void tied_native_bounds(const double*,const double*,const double*,double*);

namespace tied_search_test {
inline ts::SearchBoundsInput BoundsInput(const ts::SearchInput& in,double maximum=0) {
  ts::SearchBoundsInput out;
  std::copy(std::begin(in.geometry_m.master_position),std::end(in.geometry_m.master_position),out.master_position_m);
  out.topology=in.topology;
  out.master_thickness_m=in.master_thickness_m;
  out.maximum_secondary_shell_thickness_m=maximum;
  out.working_length_to_m=in.working_length_to_m;
  return out;
}
inline std::array<double,8> NativeBounds(const ts::WorkingSearchBoundsInput& in,ts::Vec3 point) {
  double coordinates[15]{};
  for(unsigned i=0;i<5;++i) {
    const auto p=i==4?point:in.master_position[i];
    coordinates[3*i]=p.x;coordinates[3*i+1]=p.y;coordinates[3*i+2]=p.z;
  }
  const double master=in.master_thickness;
  const double secondary=in.maximum_secondary_shell_thickness;
  std::array<double,8> result{};
  tied_native_bounds(coordinates,&master,&secondary,result.data());
  return result;
}
inline std::array<double,8> NativeBounds(const ts::SearchBoundsInput& in,ts::Vec3 point) {
  ts::WorkingSearchBoundsInput working;
  working.working_length_to_m=in.working_length_to_m;
  working.topology=in.topology;
  working.master_thickness=in.master_thickness_m/in.working_length_to_m;
  working.maximum_secondary_shell_thickness=in.maximum_secondary_shell_thickness_m/in.working_length_to_m;
  for(unsigned i=0;i<4;++i) {
    const auto p=in.master_position_m[i];
    working.master_position[i]={p.x/in.working_length_to_m,p.y/in.working_length_to_m,p.z/in.working_length_to_m};
  }
  return NativeBounds(working,{point.x/in.working_length_to_m,point.y/in.working_length_to_m,point.z/in.working_length_to_m});
}
inline std::array<double,7> BoundValues(const ts::NativeSearchBounds& b) {
  return {b.minimum.x,b.minimum.y,b.minimum.z,b.maximum.x,b.maximum.y,b.maximum.z,b.inflation};
}
} // namespace tied_search_test
