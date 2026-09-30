#include "NativeOracle.h"
#include <stdexcept>
#include <vector>

extern "C" void law44_solid_native(int,const double*,const double*,int,const double*,int,
    const double*,double,double*,int*,double*,int*);
namespace law44_solid_test {
NativeResult Native(const law::Parameters& p,const law::History& h,const law::Input& in) {
  if(!law::detail::CurveShape(p.curve) || h.curve_cursor>=p.curve.count-1)
    throw std::invalid_argument("Native LAW44 curve shape/cursor");
  std::vector<double> curve(2*(p.curve.count+1));
  for(unsigned i=0;i<p.curve.count;++i) {
    curve[2*(i+1)]=p.curve.plastic_strain[i];
    curve[2*(i+1)+1]=p.curve.yield_stress_pa[i];
  }
  const auto& m=p.material;
  // A zero Fcut exercises the actual original native 10000/s default.
  const double material[]={m.young_pa,m.poisson_ratio,m.density_kg_m3,
      m.rate_c_per_s,m.rate_p,m.cutoff_hz==10000 ? 0 : m.cutoff_hz,270.*1e6};
  double base[14]{},motion[7]{};
  std::copy_n(h.stress_pa,6,base);
  std::copy_n(h.engineering_strain,6,base+6);
  base[12]=h.plastic_strain;
  base[13]=h.filtered_rate_per_s;
  std::copy_n(in.engineering_rate_per_s,6,motion);
  motion[6]=in.dt_s;
  std::array<double,19> values{};
  NativeResult r{};
  int cursor=-1,status=-1;
  law44_solid_native(p.curve.count,curve.data(),material,static_cast<int>(m.native_units),
      base,h.curve_cursor,motion,in.relative_density,values.data(),&cursor,r.prepared.data(),&status);
  if(status || cursor<0 || static_cast<unsigned>(cursor)>=p.curve.count-1)
    throw std::runtime_error("Native LAW44 packet rejected");
  std::copy_n(values.begin(),6,r.result.history.stress_pa);
  std::copy_n(values.begin()+6,6,r.result.history.engineering_strain);
  r.result.history.plastic_strain=values[12];
  r.result.history.filtered_rate_per_s=values[13];
  r.result.history.curve_cursor=cursor;
  r.result.plastic_increment=values[14];
  r.result.yield_stress_pa=values[15];
  r.result.sound_speed_m_s=values[16];
  r.result.material_viscosity_pa_s=values[17];
  r.result.tangent_factor=values[18];
  return r;
}
void Compare(const law::Result& a,const law::Result& b,const law::Parameters& p,
             const law::History& h,const law::Input& in) {
  const auto av=Values(a),bv=Values(b);
  for(unsigned i=0;i<av.size();++i) {
    SCOPED_TRACE(i);
    const double tolerance=i<6 ? StressTolerance(p,h,in) :
        3e-11*std::max({std::abs(av[i]),std::abs(bv[i]),1e-12});
    EXPECT_NEAR(av[i],bv[i],tolerance);
  }
  EXPECT_EQ(a.history.curve_cursor,b.history.curve_cursor);
  EXPECT_EQ(a.plastic_increment==0,b.plastic_increment==0);
  EXPECT_EQ(a.material_viscosity_pa_s,0);
  EXPECT_EQ(b.material_viscosity_pa_s,0);
}
}  // namespace law44_solid_test
