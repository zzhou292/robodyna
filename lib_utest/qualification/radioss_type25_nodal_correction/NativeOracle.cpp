// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NativeOracle.h"
#include <climits>
#include <stdexcept>
extern "C" void rd_nodal_correction(const int*,const int*,const int*,const int*,
    const double*,const double*,const double*,const int*,const int*,double*);
namespace nodal_correction_test {
std::vector<double> Oracle(const c::Input& in) {
  if(in.node_count>1024||in.solid_count>256||in.type24_secondary_count>1024)
    throw std::runtime_error("Native correction fixture capacity");
  const int nodes=in.node_count,solids=in.solid_count,count=in.type24_secondary_count;
  std::vector<int> slots(8*solids),controls(solids),secondary(count);
  std::vector<double> bulk(solids),controlled(solids),result(nodes);
  for(int i=0;i<solids;++i) {
    const auto& s=in.solids[i];controls[i]=s.control;
    // Disabled branches retain deliberately nonfinite unused data, just as the
    // donor's CYCLE avoids consuming it. Their node slots stay in-range here.
    bulk[i]=s.bulk;controlled[i]=s.controlled_bulk;
    for(unsigned k=0;k<8;++k)slots[8*i+k]=s.control==1?int(s.nodes[k]+1):1;
  }
  for(int i=0;i<count;++i)secondary[i]=in.type24_secondaries[i]>=in.node_count?nodes+1:int(in.type24_secondaries[i]+1);
  rd_nodal_correction(&nodes,&solids,slots.data(),controls.data(),bulk.data(),controlled.data(),
      in.coefficients,&count,secondary.data(),result.data());
  return result;
}
}
