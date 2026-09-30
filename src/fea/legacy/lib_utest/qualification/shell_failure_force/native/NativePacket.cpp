#include "NativePacket.h"
#include "../../native/qeph/NativeQephBridge.h"
#include "../../native/t3/T3EngineContext.h"
#include "../../shell_layered_j2/native_recurrence/NativeLayeredInput.h"

namespace failure_force_test::native {
extern "C" void lf_qeph_force(const double*,const double*,const double*,const double*,const double*,const double*,
 int,const double*,const double*,double*,int,const double*,const double*,const double*,double*,double*,int*,double*,double*,int*,int*);
extern "C" void lf_t3_force(const double*,const double*,const double*,const double*,const double*,const double*,
 int,const double*,const double*,double*,int,const double*,const double*,const double*,double*,double*,int*,double*,double*,int*);
namespace {
struct Material {
  int mode,count;std::vector<double> curve;
  double linear[2],rate[3];
  explicit Material(const sec::PointParameters& p):mode(p.hardening==tl::material::ShellPlasticityHardeningKind::Tabulated),
      count(mode?static_cast<int>(p.curve.count):0),curve(2*(count+1),0),
      linear{p.linear.initial_yield_pa,p.linear.tangent_modulus_pa},
      rate{p.rate.cowper_symonds_c_per_s,p.rate.cowper_symonds_p,p.rate.cutoff_hz} {
    for(int i=0;i<count;++i) {curve[2*i+2]=p.curve.plastic_strain[i];curve[2*i+3]=p.curve.yield_stress_pa[i];}
  }
};
template<std::size_t N> std::array<double,3*N> Nodes(const Vec3 (&v)[N]) {
  std::array<double,3*N> out{};for(unsigned i=0;i<N;++i){out[3*i]=v[i].x;out[3*i+1]=v[i].y;out[3*i+2]=v[i].z;}return out;
}
void Finish(Packet& p,int status,unsigned start,unsigned count) {
  if(status)throw std::runtime_error("native family force rejected");
  for(double x:p.force)if(!std::isfinite(x))throw std::runtime_error("nonfinite native force");
  std::copy_n(p.force.begin()+start,count,p.history.begin());
}
}
void Advance(const nq::Reference& r,const q::PrescribedInterval& in,const sec::PointParameters& material,double d1,Packet& p) {
  const auto& s=r.data().input;const double basic[]{s.density,s.young_modulus,s.poisson_ratio,s.thickness,r.data().nodal_mass[0]};
  const auto x=Nodes(in.position_endpoint),v=Nodes(in.velocity_midpoint),w=Nodes(in.omega_midpoint);Material m(material);
  const double time=in.base_time+in.dt;int status=-1;
  const std::scoped_lock lock(nq::detail::NativeContext(),tl::qualification::layered_native::detail::SectionContext());
  lf_qeph_force(x.data(),v.data(),w.data(),basic,p.history.data(),&in.dt,m.count,m.curve.data(),m.rate,
      p.points.data(),m.mode,m.linear,&d1,&time,p.failures.data(),p.point_values.data(),&p.removed,
      p.diagnostics.data(),p.force.data(),&p.planar,&status);
  Finish(p,status,80,38);
}
void Advance(const nt::Reference& r,const t::PrescribedInterval& in,const sec::PointParameters& material,double d1,Packet& p) {
  const auto& s=r.data().input;const double basic[]{s.density,s.young_modulus,s.poisson_ratio,s.thickness};
  const auto x=Nodes(in.position),v=Nodes(in.velocity),w=Nodes(in.angular_velocity);Material m(material);
  const double time=in.base_time+in.dt;int status=-1;
  const std::scoped_lock lock(nt::detail::NativeEngineContext(),tl::qualification::layered_native::detail::SectionContext());
  lf_t3_force(x.data(),v.data(),w.data(),basic,p.history.data(),&in.dt,m.count,m.curve.data(),m.rate,
      p.points.data(),m.mode,m.linear,&d1,&time,p.failures.data(),p.point_values.data(),&p.removed,
      p.diagnostics.data(),p.force.data(),&status);
  Finish(p,status,38,26);
}
} // namespace failure_force_test::native
