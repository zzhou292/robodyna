// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NativeMaps.h"
namespace type25_search_test {
extern "C" void rd_search_reference(const int*,const int*,const int*,const int*,
    const int*,const int*,const int*,const double*,double*);
std::vector<double> NativeReference(const s::Source& source,const s::Current& current) {
  const int nodes=source.physical_nodes,ns=source.secondaries,nm=source.mains,no=source.main_1d;
  std::vector<int> secondary(ns),main(nm),one_d(no);
  for(int i=0;i<ns;++i)secondary[i]=source.secondary_nodes[i]+1;
  for(int i=0;i<nm;++i)main[i]=NativeMain(source,current,source.main_nodes[i]);
  for(int i=0;i<no;++i)one_d[i]=NativeMain(source,current,source.main_1d_nodes[i]);
  const double length=source.input_units==s::InputUnits::Si?source.units.length_m:1;
  std::vector<double> x(3*nodes),saved(3*((ns+nm+no<nodes)?ns+nm+no:nodes));
  for(int i=0;i<nodes;++i){const auto v=current.positions.at(i);
    x[3*i]=v.x/length;x[3*i+1]=v.y/length;x[3*i+2]=v.z/length;}
  rd_search_reference(&nodes,&ns,&nm,&no,secondary.data(),main.data(),one_d.data(),x.data(),saved.data());
  return saved;
}
}
