#include "NativeOracle.h"
#include <stdexcept>
#include <vector>
extern "C" {
void law36_native_point(int,const double*,double,double,double,const double*,const double*,double,int,double*);
void law36_native_caller(int,const double*,double,double,double,const double*,const double*,const double*,int,double*);
}
namespace law36_test {
namespace {
std::vector<double> CurvePacket(law::Curve c) {
  if (!law::detail::CurveShape(c)) throw std::invalid_argument("Native curve count");
  std::vector<double> v(2*(c.count+1));
  for (unsigned i=0;i<c.count;++i) {
    v[2*(i+1)]=c.plastic_strain[i];
    v[2*(i+1)+1]=c.yield_stress_pa[i];
  }
  return v;
}
std::array<double,7> MotionPacket(const law::Kinematics& k) {
  std::array<double,7> v{};
  std::copy_n(k.engineering_rate_per_s,6,v.begin());
  v[6]=k.dt_s;
  return v;
}
law::History ReadHistory(const double* v) {
  law::History h{};
  std::copy_n(v,6,h.stress_pa);
  std::copy_n(v+6,6,h.engineering_strain);
  h.plastic_strain=v[12];
  h.deviatoric_rate_per_s=v[13];
  return h;
}
}
std::array<double,20> Native(const law::Parameters& p,const law::History& h,
                            const law::Input& in,int defaults) {
  const auto curve=CurvePacket(p.curve);
  const auto base=Values(h);
  const auto k=MotionPacket(in.kinematics);
  std::array<double,20> out{};
  law36_native_point(p.curve.count,curve.data(),p.young_pa,p.poisson_ratio,p.density_kg_m3,
                    base.data(),k.data(),in.relative_density,defaults,out.data());
  return out;
}
std::array<double,26> Native(const law::Parameters& p,const law::CallerHistory& h,
                            const law::Kinematics& in,const law::Measures& m,int defaults) {
  const auto curve=CurvePacket(p.curve);
  std::array<double,16> base{};
  const auto point=Values(h.point);
  std::copy(point.begin(),point.end(),base.begin());
  base[14]=h.internal_energy_density_j_m3;
  base[15]=h.plastic_work_j;
  const auto k=MotionPacket(in);
  const double measures[6]={m.density_kg_m3,m.storage_volume_m3,m.current_volume_m3,
                           m.volume_increment_m3,m.old_bulk_pressure_pa,m.new_bulk_pressure_pa};
  std::array<double,26> out{};
  law36_native_caller(p.curve.count,curve.data(),p.young_pa,p.poisson_ratio,p.density_kg_m3,
                     base.data(),k.data(),measures,defaults,out.data());
  return out;
}
law::History NativeHistory(const std::array<double,20>& v) { return ReadHistory(v.data()); }
law::CallerHistory NativeHistory(const std::array<double,26>& v) {
  return {ReadHistory(v.data()),v[20],v[21]};
}
}
