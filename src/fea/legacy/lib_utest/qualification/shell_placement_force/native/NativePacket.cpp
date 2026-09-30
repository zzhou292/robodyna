#include "NativePacket.h"
#include "../../shell_tab1_force/native/NativePacketValues.h"
#include "../../native/qeph/NativeQephBridge.h"
#include "../../native/t3/T3EngineContext.h"
#include "../../shell_layered_j2/native_recurrence/NativeLayeredInput.h"

namespace placement_force_test::native {
namespace q=tl::fea::qeph;
namespace t=tl::fea::t3;
namespace sec=tl::fea::sections;
namespace nq=tl::qualification::qeph;
namespace nt=tl::qualification::t3;
using tl::math::Vec3;
extern "C" void placed_tab1_qeph_force(int,const double*,const double*,const double*,const double*,const double*,const double*,
    int,const double*,const double*,double*,int,const double*,const double*,const double*,double*,double*,int*,double*,double*,int*,int*);
extern "C" void placed_tab1_t3_force(int,const double*,const double*,const double*,const double*,const double*,const double*,
    int,const double*,const double*,double*,int,const double*,const double*,const double*,double*,double*,int*,double*,double*,int*);
using namespace tab1_force_test::native::values;

void Advance(const q::ReferenceData& r,const q::PrescribedInterval& in,
    const sec::PointParameters& material,const sec::ShellLayeredTab1Parameters& failure,Packet& p) {
  const auto& s=r.input;
  const int ipos=s.placement==Placement::Centered?0:s.placement==Placement::TopReferencePlane?3:4;
  // Native startup supplies the Q4 mass used by CNDT3; no production mass
  // or inertia value is used as an oracle input. Placement does not alter mass.
  const auto startup=tab1_force_test::native::Reference<nq::Reference>(s);
  const double basic[]{s.density,s.young_modulus,s.poisson_ratio,s.thickness,startup.data().nodal_mass[0]};
  const auto x=Nodes(in.position_endpoint),v=Nodes(in.velocity_midpoint),w=Nodes(in.omega_midpoint);
  Material m(material,failure);
  const double time=in.base_time+in.dt;
  int status=-1;
  const std::scoped_lock lock(nq::detail::NativeContext(),tl::qualification::layered_native::detail::SectionContext());
  placed_tab1_qeph_force(ipos,x.data(),v.data(),w.data(),basic,p.history.data(),&in.dt,0,m.unused_curve,m.rate,
      p.points.data(),0,m.linear,m.table,&time,p.failures.data(),p.point_values.data(),&p.removed,
      p.diagnostics.data(),p.force.data(),&p.planar,&status);
  Finish(p,status,80,38);
}
void Advance(const t::ReferenceData& r,const t::PrescribedInterval& in,
    const sec::PointParameters& material,const sec::ShellLayeredTab1Parameters& failure,Packet& p) {
  const auto& s=r.input;
  const int ipos=s.placement==Placement::Centered?0:s.placement==Placement::TopReferencePlane?3:4;
  const double basic[]{s.density,s.young_modulus,s.poisson_ratio,s.thickness};
  const auto x=Nodes(in.position),v=Nodes(in.velocity),w=Nodes(in.angular_velocity);
  Material m(material,failure);
  const double time=in.base_time+in.dt;
  int status=-1;
  const std::scoped_lock lock(nt::detail::NativeEngineContext(),tl::qualification::layered_native::detail::SectionContext());
  placed_tab1_t3_force(ipos,x.data(),v.data(),w.data(),basic,p.history.data(),&in.dt,0,m.unused_curve,m.rate,
      p.points.data(),0,m.linear,m.table,&time,p.failures.data(),p.point_values.data(),&p.removed,
      p.diagnostics.data(),p.force.data(),&status);
  Finish(p,status,38,26);
}
} // namespace placement_force_test::native
