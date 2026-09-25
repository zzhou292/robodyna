#pragma once
#include "Cases.h"
#include <algorithm>
namespace pen3_test {
extern "C" void rd_pen3_cohort(const int*,const double*,const double*,const double*,const int*,double*);
inline std::array<double,2> Evaluate(const Case& test,unsigned mode) {
  const bool target_last=mode==3;
  const auto other=Unrelated(mode==1);
  const Packet first=target_last?other:test.target;
  const Packet second=target_last?test.target:other;
  double coordinates[30],gaps[]{first.gap,second.gap};
  int flags[12];
  std::copy(first.coordinates.begin(),first.coordinates.end(),coordinates);
  std::copy(second.coordinates.begin(),second.coordinates.end(),coordinates+15);
  std::copy(first.flags.begin(),first.flags.end(),flags);
  std::copy(second.flags.begin(),second.flags.end(),flags+6);
  const int count=mode==0?1:2;
  std::array<double,2> output{};
  rd_pen3_cohort(&count,coordinates,gaps,&test.margin,flags,output.data());
  return target_last?std::array<double,2>{output[1],output[0]}:output;
}
} // namespace pen3_test
