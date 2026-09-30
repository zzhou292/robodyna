#pragma once
#include "GroupStepNativeFixture.h"
#include "lib_src/constraints/NodalRigidTwoMemberStep.h"
namespace rigid_two_test {
using namespace rigid_step_test;
// Original source IDs, member order and bit-exact canonical SI geometry. Native
// M/J below are deliberately synthetic; this does not claim full-car mass closure.
inline constexpr std::uint64_t GroupIds[]{2200631,2200641,2200751,2200753};
inline constexpr std::uint64_t NodeIds[][2]{{2129738,2129663},{2274961,2391081},
  {2411713,2411808},{2411943,2411886}};
inline constexpr Vec3 Position[][2]{
  {{-0x1.1b1ae12cd621ep+1,0x1.38821cd045524p-1,0x1.393664401b790p+0},
   {-0x1.1c34d925a0a37p+1,0x1.38bb53fe84d5cp-1,0x1.391967b01281ep+0}},
  {{-0x1.1e19b258ad404p+1,-0x1.352776bc74e5dp-1,0x1.3984230fcf80dp+0},
   {-0x1.1c09e3ae90a1bp+1,-0x1.3858108d40f0bp-1,0x1.37e6560ace902p+0}},
  {{-0x1.942e0e30446b6p+1,0x1.7c24b8462d0b0p-1,0x1.a8a9458921267p-1},
   {-0x1.94166094954ddp+1,0x1.816e4be3026e4p-1,0x1.a91403edf63d3p-1}},
  {{-0x1.942e140f817d4p+1,-0x1.7c24b23bfcdb6p-1,0x1.a8a97ce685859p-1},
   {-0x1.942b6ae7d566dp+1,-0x1.85a0128491f59p-1,0x1.a924cae596f32p-1}}};
struct SourceFixture {
  std::array<fe::NodalRigidGroupMember,8> members{};
  std::array<fe::NodalRigidGroupInput,4> groups{};
  fe::NodalRigidGroupModel model;
  explicit SourceFixture(double native_j=1e-12) {
    for(unsigned g=0;g<4;++g) {
      groups[g]={GroupIds[g],GroupIds[g],members.data()+2*g,2};
      for(unsigned i=0;i<2;++i) members[2*g+i]={NodeIds[g][i],2*g+i,Position[g][i],
        i?.05:.03,native_j,.4*native_j,.6*native_j};
    }
    const auto r=model.Initialize({781,9,groups.data(),4,{1000,.001}});
    EXPECT_TRUE(r)<<r.message;
  }
  rigid_step_test::Input Packet(unsigned g,rigid::StepDurations durations={0,1./2048,1./1024}) const {
    rigid_step_test::Input in{}; const auto& p=model.groups()[g];
    in.body={p.principal,p.center,{.3,-.2,.1},{},p.total_mass_kg,{},durations};
    for(unsigned i=0;i<2;++i) { const auto& m=members[2*g+i];
      in.member[i]={m.position,in.body.velocity,{},{},{},m.mass_kg,m.total_inertia_kg_m2}; }
    return in;
  }
};
inline rigid::StepStatus EvaluateTwoPacket(const rigid_step_test::Input& in,Trial& out,double length=.001) {
  auto body=in.body; Vec3 x[2],f[2],c[2];
  for(unsigned i=0;i<2;++i) { x[i]=in.member[i].position; f[i]=in.member[i].force;c[i]=in.member[i].couple; }
  if(rigid::AggregateWrench(body.center,x,f,c,2,body.applied)!=rigid::MathStatus::Success)
    return rigid::StepStatus::InvalidInput;
  Trial next{}; auto status=rigid::EvaluateTwoMemberPrimaryStep(body,next.primary);
  if(status!=rigid::StepStatus::Success) return status;
  for(unsigned i=0;i<2;++i) {
    status=rigid::EvaluateTwoMemberStep(body,next.primary,in.member[i],length,next.member[i]);
    if(status!=rigid::StepStatus::Success) return status;
  }
  out=next;return rigid::StepStatus::Success;
}
inline void Carry(const Trial& t,rigid_step_test::Input& in) {
  in.body.center=t.primary.center;in.body.velocity=t.primary.velocity;in.body.omega=t.primary.omega;
  in.body.previous_frame=t.primary.force_frame;
  for(unsigned i=0;i<2;++i) {in.member[i].position=t.member[i].position;
    in.member[i].velocity=t.member[i].velocity;in.member[i].omega=t.member[i].omega;}
}
} // namespace rigid_two_test
