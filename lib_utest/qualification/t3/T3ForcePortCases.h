#pragma once
#include "T3ForcePortFixture.h"
namespace t3_force_port_test {
// 864 signed-column cases plus6 tiny translated cases, shared by host/device.
// Parity configurations use real source-order inputs, not deformation re-setup.
constexpr unsigned ParityCases=870;
template<class F> void ForEachParityCase(F&& check) {
  unsigned index=0;
  for(double scale:{1.,.02}) for(unsigned shape:{0u,1u}) for(unsigned pose=0;pose<3;++pose)
    for(unsigned column=0;column<18;++column) for(double sign:{-1.,1.}) for(bool seeded:{false,true}) {
      SCOPED_TRACE(index++);
      auto input=Triangle(scale,shape); input.node_ids[2]=(1ULL<<53)+37;
      for(auto& x:input.position) {
        if(pose==1)x=kt::Rotate(kt::Rotation(),x);
        if(pose==2)x={x.y,x.z,x.x};
      }
      auto in=Interval(input,1e-4);
      Set(column%6<3?in.velocity[column/6]:in.angular_velocity[column/6],column%3,sign*.001);
      auto values=seeded?Values(oracle::Seed(input.thickness)):port::HistoryValues{};
      values.thickness=input.thickness;
      check(input,values,in);
    }
  for(unsigned pose=0;pose<3;++pose) for(bool seeded:{false,true}) {
    SCOPED_TRACE(index++);
    auto input=Triangle(1e-5); input.node_ids[1]=(1ULL<<53)+81;
    for(auto& x:input.position) {
      if(pose==1)x=kt::Rotate(kt::Rotation(),x);
      if(pose==2)x={x.y,x.z,x.x};
      x.x+=5; x.y-=3; x.z+=.125;
    }
    auto in=Interval(input,1e-4); in.velocity[1].z=.001; in.angular_velocity[2].x=.002;
    auto values=seeded?Values(oracle::Seed(input.thickness)):port::HistoryValues{};
    values.thickness=input.thickness;
    check(input,values,in);
  }
  EXPECT_EQ(index,ParityCases);
}
constexpr unsigned InvalidForceCases=13;
inline void Fault(unsigned index,port::ReferenceData& r,port::History& h,port::PrescribedInterval& in) {
  switch(index) {
    case 0: r={}; break;
    case 1: ++r.input.node_ids[0]; break;
    case 2: in.base_time=.25; break;
    case 3: ++in.sample_index; break;
    case 4: in.dt=0; break;
    case 5: in.velocity[1].x=std::numeric_limits<double>::quiet_NaN(); break;
    case 6: h={}; break;
    case 7: h=History(r,h.data(),{0,UINT64_MAX}); break;
    case 8: in.position[2]=in.position[1]; break;
    case 9: Mode(in,0,1e6); break; // Late reported-thickness rejection.
    case 10: {
      auto values=h.data();
      for(unsigned i=0;i<3;++i) values.material_stress[i]=std::numeric_limits<double>::max();
      h=History(r,values); const auto large=Triangle(480);
      for(unsigned n=0;n<3;++n) { in.position[n]=large.position[n]; in.velocity[n]={}; in.angular_velocity[n]={}; }
      break; // Late force overflow, finite prescribed history.
    }
    case 11: in.angular_velocity[2].x=std::numeric_limits<double>::max(); break;
    case 12: {
      auto input=r.input; input.young_modulus=std::numeric_limits<double>::max();
      r=Reference(input); h=History(r); break; // Genuine bound reference; A11 overflows in material preparation.
    }
  }
}
} // namespace t3_force_port_test
