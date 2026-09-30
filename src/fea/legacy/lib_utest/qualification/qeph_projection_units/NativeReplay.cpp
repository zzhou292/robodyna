// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NativeReplay.h"
#include "lib_utest/qualification/native/qeph/NativeQephBridge.h"
#include <cmath>
#include <mutex>
#include <stdexcept>
namespace qeph_projection_test {
extern "C" void qeph_projection_native_rates(const double*,const double*,const double*,const int*,const double*,double*,int*);
extern "C" void qeph_projection_native_forces(const double*,const int*,const int*,const double*,const double*,const double*,const double*,double*);
namespace {
void Need(bool value){if(!value)throw std::invalid_argument("Unsupported or nonfinite native projection packet");}
template<class A> void Finite(const A& a){for(double x:a)Need(std::isfinite(x));}
std::array<int,6> Pack(const Controls& c) {
  Need(c.npt==3&&c.idril==0&&c.ifini==0&&c.iresp==0&&c.impl_s==0&&c.ikproj==0);
  Need(std::isfinite(c.tolerance)&&c.tolerance>0);
  return {c.npt,c.idril,c.ifini,c.iresp,c.impl_s,c.ikproj};
}
std::array<double,29> Pack(const Geometry& g) {
  std::array<double,29> p{g.area,g.area_i,g.x13,g.x24,g.y13,g.y24,g.mx13,g.my13,g.z1,g.ll,g.l13,g.l24};
  for(unsigned n=0;n<4;++n)for(unsigned k=0;k<2;++k)p[12+2*n+k]=g.corel[n][k];
  for(unsigned k=0;k<9;++k)p[20+k]=g.vq[k];
  Finite(p);Need(g.area>0&&g.area_i>0&&g.ll>0&&g.l13>0&&g.l24>0);return p;
}
template<std::size_t D> std::array<double,4*D> Pack(const std::array<std::array<double,D>,4>& values) {
  std::array<double,4*D> result{};
  for(unsigned n=0;n<4;++n)for(unsigned k=0;k<D;++k){Need(std::isfinite(values[n][k]));result[D*n+k]=values[n][k];}
  return result;
}
}
RateResult NativeRates(const RateInput& input) {
  const auto c=Pack(input.controls);const auto g=Pack(input.geometry);const auto omega=Pack(input.world_omega);
  std::array<double,17> r{};
  for(unsigned k=0;k<3;++k){r[k]=input.v13[k];r[3+k]=input.v24[k];r[6+k]=input.vhi[k];}
  for(unsigned n=0;n<4;++n)for(unsigned k=0;k<2;++k)r[9+2*n+k]=input.rlxyz[n][k];
  Finite(r);std::array<double,48> result{};int planar=0;
  {std::lock_guard<std::mutex> guard(tl::qualification::qeph::detail::NativeContext());
   qeph_projection_native_rates(g.data(),r.data(),omega.data(),c.data(),&input.controls.tolerance,result.data(),&planar);}
  Finite(result);Need(planar==0||planar==1);
  RateResult out;out.projection.planar=planar!=0;out.projection.warped_defined=planar==0;out.projection.z1=result[17];
  for(unsigned k=0;k<3;++k){out.v13[k]=result[k];out.v24[k]=result[3+k];out.vhi[k]=result[6+k];}
  for(unsigned n=0;n<4;++n)for(unsigned k=0;k<2;++k)out.rlxyz[n][k]=result[9+2*n+k];
  if(!out.projection.planar) {
    for(unsigned k=0;k<6;++k)out.projection.di[k]=result[18+k];
    for(unsigned n=0;n<4;++n)for(unsigned k=0;k<3;++k){out.projection.db[n][k]=result[24+3*n+k];out.projection.vqn[n][k]=result[36+3*n+k];}
  }
  return out;
}
ForceResult NativeForces(const ForceInput& input) {
  const auto c=Pack(input.controls);const auto g=Pack(input.geometry);const auto vf=Pack(input.vf);const auto vm=Pack(input.vm);
  Need(std::isfinite(input.projection.z1));Need(input.projection.warped_defined==!input.projection.planar);
  std::array<double,30> projection{};
  if(!input.projection.planar) {
    Finite(input.projection.di);for(unsigned k=0;k<6;++k)projection[k]=input.projection.di[k];
    for(unsigned n=0;n<4;++n)for(unsigned k=0;k<3;++k){projection[6+3*n+k]=input.projection.db[n][k];projection[18+3*n+k]=input.projection.vqn[n][k];}
    Finite(projection);
  }
  std::array<double,24> result{};const int planar=input.projection.planar?1:0;
  {std::lock_guard<std::mutex> guard(tl::qualification::qeph::detail::NativeContext());
   qeph_projection_native_forces(g.data(),c.data(),&planar,&input.projection.z1,projection.data(),vf.data(),vm.data(),result.data());}
  Finite(result);ForceResult out;
  for(unsigned n=0;n<4;++n)for(unsigned k=0;k<3;++k){out.force[n][k]=result[3*n+k];out.couple[n][k]=result[12+3*n+k];}
  return out;
}
} // namespace qeph_projection_test
