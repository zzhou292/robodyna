#pragma once
#include "../FailureForceFixture.h"
#include "../../native/qeph/QephReference.h"
#include "../../native/t3/T3Reference.h"
#include <array>

namespace failure_force_test::native {
namespace nq=tl::qualification::qeph;namespace nt=tl::qualification::t3;
// Independent native recurrence buffers. Initial prescribed values are packed
// once; subsequent calls retain only the preceding native result.
struct Packet {
  std::array<double,38> history{};
  std::array<double,21> points{};
  std::array<double,9> failures{};
  std::array<double,152> force{};
  std::array<double,39> point_values{};
  std::array<double,9> diagnostics{};
  int removed=0,planar=0;
};
inline std::vector<double> History(const q::HistoryValues& h) {
  std::vector<double> v;Append(v,h.stress);Append(v,h.material_stress);Append(v,h.bending_stress);
  Append(v,h.stabilization);Append(v,h.strain_curvature);Append(v,h.thickness);Append(v,h.internal_work);
  Append(v,h.hourglass_viscous_work);Append(v,h.active);return v;
}
inline std::vector<double> History(const t::HistoryValues& h) {
  std::vector<double> v;Append(v,h.stress);Append(v,h.material_stress);Append(v,h.bending_stress);
  Append(v,h.strain_curvature);Append(v,h.thickness);Append(v,h.internal_work);
  Append(v,h.equivalent_strain_rate);Append(v,h.active);return v;
}
template<class H> Packet Seed(const H& h) {
  Packet p;const auto values=History(h.shell.data());std::copy(values.begin(),values.end(),p.history.begin());
  for(unsigned i=0;i<3;++i) {
    std::copy_n(h.section.saved.point[i].stress,5,p.points.begin()+7*i);
    p.points[7*i+5]=h.section.saved.point[i].plastic_strain;p.points[7*i+6]=h.section.saved.point[i].filtered_rate_per_s;
    p.failures[3*i]=h.section.failure[i].damage;p.failures[3*i+1]=h.section.failure[i].failure_time_s;
    p.failures[3*i+2]=h.section.failure[i].point_active?1:0;
  }return p;
}
template<class N,class R> N Reference(const R& input) {
  N value;auto in=value.data().input;
  in.density=input.density;in.young_modulus=input.young_modulus;in.poisson_ratio=input.poisson_ratio;in.thickness=input.thickness;
  for(unsigned i=0;i<in.position.size();++i) {in.position[i]=input.position[i];in.node_ids[i]=input.node_ids[i];}
  if(static_cast<int>(Initialize(in,value))!=0)throw std::runtime_error("independent native reference");return value;
}
void Advance(const nq::Reference&,const q::PrescribedInterval&,const sec::PointParameters&,double d1,Packet&);
void Advance(const nt::Reference&,const t::PrescribedInterval&,const sec::PointParameters&,double d1,Packet&);
} // namespace failure_force_test::native
