#pragma once
#include "../Tab1ForceFixture.h"
#include "../../shell_failure_force/native/NativePacket.h"

namespace tab1_force_test::native {
namespace nq=tl::qualification::qeph;
namespace nt=tl::qualification::t3;
using failure_force_test::native::Reference;
using failure_force_test::native::History;
struct Packet {
  std::array<double,38> history{};
  std::array<double,21> points{};
  std::array<double,15> failures{}; // damage, TT, FOFF, DFMAX, table cache per point
  std::array<double,152> force{};
  std::array<double,39> point_values{};
  std::array<double,9> diagnostics{};
  int removed=0,planar=0;
};
template<class H> Packet Seed(const H& h) {
  Packet p;
  const auto values=History(h.shell.data());
  std::copy(values.begin(),values.end(),p.history.begin());
  for(unsigned i=0;i<3;++i) {
    const auto& point=h.section.saved.point[i];
    std::copy_n(point.stress,5,p.points.begin()+7*i);
    p.points[7*i+5]=point.plastic_strain;
    p.points[7*i+6]=point.filtered_rate_per_s;
    const auto& f=h.section.failure[i];
    p.failures[5*i]=f.damage;
    p.failures[5*i+1]=f.failure_time_s;
    p.failures[5*i+2]=f.point_active?1:0;
    p.failures[5*i+3]=f.maximum_damage;
    p.failures[5*i+4]=f.table_segment;
  }
  return p;
}
void Advance(const nq::Reference&,const q::PrescribedInterval&,const sec::PointParameters&,
    const sec::ShellLayeredTab1Parameters&,Packet&);
void Advance(const nt::Reference&,const t::PrescribedInterval&,const sec::PointParameters&,
    const sec::ShellLayeredTab1Parameters&,Packet&);
} // namespace tab1_force_test::native
