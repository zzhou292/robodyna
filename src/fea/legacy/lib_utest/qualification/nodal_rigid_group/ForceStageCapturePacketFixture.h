#pragma once
#include "GroupStepTestSupport.h"
#include "lib_src/constraints/NodalRigidGroupCandidate.h"
#include <cstring>

namespace force_stage_capture_test {
namespace native=rigid_step_test;
namespace rigid=tl::fea::rigid;
using native::Vec3;
constexpr std::size_t PacketNodes=4,PacketState=19*PacketNodes+rigid::GroupStateValues;
struct PacketCapture {
  std::array<double,PacketState> accepted{},plain{},captured{};
  std::array<double,6*PacketNodes> loads{},accelerations{};
  std::array<double,6> primary{};
  rigid::GroupRange group{};
  std::array<rigid::MemberMetric,PacketNodes> members{};
  std::array<std::uint8_t,PacketNodes> member_nodes{{1,1,1,1}};
  explicit PacketCapture(const native::Input& in) {
    group={0,4,in.body.mass,in.body.previous_frame.inertia};
    rigid::WriteGroupState(accepted.data()+19*PacketNodes,
      {in.body.center,in.body.velocity,in.body.omega,in.body.previous_frame.axes});
    for(unsigned n=0;n<PacketNodes;++n) {
      const auto& m=in.member[n];members[n]={n,m.mass,m.inertia};
      rigid::candidate_detail::WriteNode(accepted.data(),n,m.position);
      rigid::candidate_detail::WriteNode(accepted.data()+3*PacketNodes,n,m.velocity);
      rigid::candidate_detail::WriteNode(accepted.data()+6*PacketNodes,n,m.omega);
      for(unsigned j=0;j<3;++j) {
        loads[j*PacketNodes+n]=rigid_test::Get(m.force,j);
        loads[(3+j)*PacketNodes+n]=rigid_test::Get(m.couple,j);
      }
    }
    plain=accepted;captured=accepted;accelerations.fill(-999);primary.fill(-999);
  }
  rigid::GroupDeviceView view() {return {&group,members.data(),member_nodes.data(),1,4};}
  rigid::AccelerationSink sink() {return {accelerations.data(),accelerations.data()+3*PacketNodes,primary.data(),primary.data()+3};}
};
inline Vec3 Read(const double* x,std::size_t n=0) {return {x[3*n],x[3*n+1],x[3*n+2]};}
inline void CheckAcceleration(const PacketCapture& p,const native::Trial& expected) {
  native::Agreement(Read(p.primary.data()),expected.primary.acceleration);
  native::Agreement(Read(p.primary.data()+3),expected.primary.angular_acceleration);
  for(unsigned n=0;n<PacketNodes;++n) {
    native::Agreement(Read(p.accelerations.data(),n),expected.member[n].acceleration);
    native::Agreement(Read(p.accelerations.data()+3*PacketNodes,n),expected.member[n].angular_acceleration);
  }
}
} // namespace force_stage_capture_test
