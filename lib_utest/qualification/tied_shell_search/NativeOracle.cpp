// SPDX-License-Identifier: MIT
#include "NativeOracle.h"
extern "C" void tied_native_search(int,const double*,const double*,const double*,
    int,int*,double*,double*,double*);
namespace tied_search_test {
ts::CandidateProjection Native(const ts::SearchInput& in,int candidate,NativeChoice& choice) {
  double x[15]{};
  for(unsigned i=0;i<5;++i) {
    const auto p=i==4?in.geometry_m.secondary_position:in.geometry_m.master_position[i];
    x[3*i]=p.x/in.working_length_to_m;x[3*i+1]=p.y/in.working_length_to_m;
    x[3*i+2]=p.z/in.working_length_to_m;
  }
  const double master=in.master_thickness_m/in.working_length_to_m;
  const double secondary=in.secondary_shell_thickness_m/in.working_length_to_m;
  double out[9]{};
  tied_native_search(in.topology==ts::MasterTopology::TriangleRepeatedThird?1:0,x,
      &master,&secondary,candidate,&choice.selected,choice.st,&choice.distance,out);
  ts::CandidateProjection result;
  result.gap_m=out[0]*in.working_length_to_m;
  result.penetration_m=out[1]*in.working_length_to_m;
  result.distance_m=out[2]*in.working_length_to_m;
  result.selection_distance=out[2];result.working_length_to_m=in.working_length_to_m;
  result.s=out[3];result.t=out[4];result.fan=static_cast<unsigned>(out[5]);
  result.admissible=out[6]==1;
  result.outside_warning=std::abs(out[3])>1.02 || std::abs(out[4])>1.02;
  return result;
}
} // namespace tied_search_test
