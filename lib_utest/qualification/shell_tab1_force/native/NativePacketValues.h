#pragma once
#include "NativePacket.h"

namespace tab1_force_test::native::values {
struct Material {
  double unused_curve[2]{};
  double linear[2],rate[3],table[4];
  Material(const sec::PointParameters& p,const sec::ShellLayeredTab1Parameters& f):
      linear{p.linear.initial_yield_pa,p.linear.tangent_modulus_pa},
      rate{p.rate.cowper_symonds_c_per_s,p.rate.cowper_symonds_p,p.rate.cutoff_hz},
      table{f.table.triaxiality[0],f.table.triaxiality[1],f.table.triaxiality[2],f.table.failure_strain} {}
};
template<std::size_t N> std::array<double,3*N> Nodes(const Vec3 (&v)[N]) {
  std::array<double,3*N> out{};
  for(unsigned i=0;i<N;++i) {
    out[3*i]=v[i].x;
    out[3*i+1]=v[i].y;
    out[3*i+2]=v[i].z;
  }
  return out;
}
inline void Finish(Packet& p,int status,unsigned start,unsigned count) {
  if(status) throw std::runtime_error("native glass force rejected");
  for(double x:p.force) if(!std::isfinite(x)) throw std::runtime_error("nonfinite native glass force");
  std::copy_n(p.force.begin()+start,count,p.history.begin());
}
} // namespace tab1_force_test::native::values
