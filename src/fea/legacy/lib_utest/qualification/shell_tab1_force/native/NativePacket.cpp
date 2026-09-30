#include "NativePacket.h"
#include "NativePacketValues.h"
#include "../../native/qeph/NativeQephBridge.h"
#include "../../native/t3/T3EngineContext.h"
#include "../../shell_layered_j2/native_recurrence/NativeLayeredInput.h"

namespace tab1_force_test::native {
extern "C" void tab1_qeph_force(const double*,const double*,const double*,const double*,const double*,const double*,
    int,const double*,const double*,double*,int,const double*,const double*,const double*,double*,double*,int*,double*,double*,int*,int*);
extern "C" void tab1_t3_force(const double*,const double*,const double*,const double*,const double*,const double*,
    int,const double*,const double*,double*,int,const double*,const double*,const double*,double*,double*,int*,double*,double*,int*);
using namespace values;

void Advance(const nq::Reference& r,const q::PrescribedInterval& in,
    const sec::PointParameters& material,const sec::ShellLayeredTab1Parameters& failure,Packet& p) {
  const auto& s=r.data().input;
  const double basic[]{s.density,s.young_modulus,s.poisson_ratio,s.thickness,r.data().nodal_mass[0]};
  const auto x=Nodes(in.position_endpoint),v=Nodes(in.velocity_midpoint),w=Nodes(in.omega_midpoint);
  Material m(material,failure);
  const double time=in.base_time+in.dt;
  int status=-1;
  const std::scoped_lock lock(nq::detail::NativeContext(),tl::qualification::layered_native::detail::SectionContext());
  tab1_qeph_force(x.data(),v.data(),w.data(),basic,p.history.data(),&in.dt,0,m.unused_curve,m.rate,
      p.points.data(),0,m.linear,m.table,&time,p.failures.data(),p.point_values.data(),&p.removed,
      p.diagnostics.data(),p.force.data(),&p.planar,&status);
  Finish(p,status,80,38);
}
void Advance(const nt::Reference& r,const t::PrescribedInterval& in,
    const sec::PointParameters& material,const sec::ShellLayeredTab1Parameters& failure,Packet& p) {
  const auto& s=r.data().input;
  const double basic[]{s.density,s.young_modulus,s.poisson_ratio,s.thickness};
  const auto x=Nodes(in.position),v=Nodes(in.velocity),w=Nodes(in.angular_velocity);
  Material m(material,failure);
  const double time=in.base_time+in.dt;
  int status=-1;
  const std::scoped_lock lock(nt::detail::NativeEngineContext(),tl::qualification::layered_native::detail::SectionContext());
  tab1_t3_force(x.data(),v.data(),w.data(),basic,p.history.data(),&in.dt,0,m.unused_curve,m.rate,
      p.points.data(),0,m.linear,m.table,&time,p.failures.data(),p.point_values.data(),&p.removed,
      p.diagnostics.data(),p.force.data(),&status);
  Finish(p,status,38,26);
}
} // namespace tab1_force_test::native
