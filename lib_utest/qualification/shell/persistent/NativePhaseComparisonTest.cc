#include "PersistentShell.h"
#include "ShellPhaseNative.h"
#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>

namespace tl::qualification::shell {
namespace {
void NearPhase(double actual,double expected) {
  EXPECT_NEAR(actual,expected,2e-11*std::max({1.0,std::abs(actual),std::abs(expected)}));
}
struct Motion {
  std::array<double,3*MaxNodes> x{},v{},w{};
  std::size_t count=4;
  fea::HostNodalKinematicsView view()const{return {x.data(),v.data(),w.data(),count};}
};
Motion Initial(std::size_t elements) {
  Motion m;m.count=4*elements;
  for(std::size_t e=0;e<elements;++e){
    const double points[12]={-1,-.5,0,1,-.5,0,1,.5,0,-1,.5,0};
    for(int i=0;i<4;++i){m.x[12*e+3*i]=points[3*i]+3*e;m.x[12*e+3*i+1]=points[3*i+1];}
  }
  return m;
}
std::array<ShellPhaseInput,MaxElements> Inputs(const Configuration& c,const Motion& m,
    const Snapshot& old,const Snapshot& evaluated,double dt) {
  std::array<ShellPhaseInput,MaxElements> in{};
  for(std::size_t e=0;e<c.element_count;++e){
    auto& p=in[e];
    for(int i=0;i<4;++i)for(int j=0;j<3;++j){
      const auto global=3*c.elements[e].nodes[i]+j;
      p.x[3*i+j]=m.x[global];p.v[3*i+j]=m.v[global];p.omega[3*i+j]=m.w[global];
    }
    std::copy(evaluated.elements[e].frame.begin(),evaluated.elements[e].frame.end(),p.frame);
    p.dt=dt;p.section_thickness=c.elements[e].thickness;p.current_thickness=old.elements[e].thickness;
    p.young=Young;p.poisson=Poisson;p.density=7800;p.sound_speed=std::sqrt(Young/(1-Poisson*Poisson)/7800);
    p.shear_factor=ShearFactor;p.ismstr=static_cast<int>(c.geometry);p.ithk=c.update_thickness?1:0;
    const auto& hg=c.stabilization;
    p.h[0]=hg.H1;p.h[1]=hg.H2;p.h[2]=hg.H3;p.srh[0]=hg.SRH1;p.srh[1]=hg.SRH2;p.srh[2]=hg.SRH3;
  }
  return in;
}
void Compare(const Snapshot& s,const std::array<ShellPhaseState,MaxElements>& old,
             const std::array<ShellPhaseOutput,MaxElements>& out) {
  for(std::size_t e=0;e<s.element_count;++e){
    const auto& a=s.elements[e];const auto& b=out[e];
    EXPECT_EQ(a.off,b.state.off);NearPhase(a.area,b.area);
    for(int j=0;j<6;++j)NearPhase(a.reference_coordinates[j],b.state.smstr[j]);
    for(int j=0;j<8;++j){NearPhase(a.generalized_strain[j],b.state.gstr[j]);NearPhase(a.generalized_strain[j]-old[e].gstr[j],b.dstrain[j]);}
    const double gradients[4]={a.px1,a.px2,a.py1,a.py2};
    for(int j=0;j<4;++j)NearPhase(gradients[j],b.gradient[j]);
    NearPhase(a.vhx,b.vhx);NearPhase(a.vhy,b.vhy);NearPhase(a.step_thickness,b.thk0);
    NearPhase(a.step_thickness*a.step_thickness,b.thk02);NearPhase(b.shear_factor,ShearFactor);
  }
}

void EvolvingCase(GeometryMode mode,std::size_t count,bool thickness) {
  Configuration c;c.geometry=mode;c.element_count=count;c.node_count=4*count;c.update_thickness=thickness;
  c.stabilization.H1=.02;c.stabilization.H2=.03;c.stabilization.HELAS=.2;
  for(std::size_t e=0;e<count;++e){c.elements[e].nodes={int(4*e),int(4*e+1),int(4*e+2),int(4*e+3)};c.elements[e].thickness=.01*(e+1);}
  Motion m=Initial(count);const Motion rest=m;
  PersistentShell shell;
  auto initialized=shell.Initialize(c,m.view());ASSERT_EQ(initialized.status,Status::Ok)<<initialized.message;
  std::array<ShellPhaseState,MaxElements> native{};for(auto& state:native)state.off=1;
  std::array<ShellPhaseOutput,MaxElements> output{};
  auto input=Inputs(c,m,shell.accepted(),shell.accepted(),.01);
  ASSERT_EQ(crash_shell_phase_native(static_cast<int>(count),input.data(),native.data(),output.data()),0);
  Compare(shell.accepted(),native,output);
  for(std::size_t e=0;e<count;++e)native[e]=output[e].state;
  for(int step=1;step<=4;++step){
    const Motion previous=m;
    const double time_end=.01*step,dt=time_end-shell.accepted().time;
    const double angle=.02*step,cs=std::cos(angle),sn=std::sin(angle);
    for(std::size_t e=0;e<count;++e)for(int i=0;i<4;++i){
      const int k=12*e+3*i;
      const double stretch=1+.003*step*(e+1);
      const double local_x=(rest.x[k]-3*e)*stretch+.001*step*rest.x[k+1];
      m.x[k]=cs*local_x+3*e;m.x[k+1]=rest.x[k+1]/stretch;m.x[k+2]=-sn*local_x;
      for(int j=0;j<3;++j)m.v[k+j]=(m.x[k+j]-previous.x[k+j])/dt;
      m.w[k]=0;m.w[k+1]=.02/dt;m.w[k+2]=0;
    }
    TrialToken token;Request request{m.view(),shell.accepted().time,time_end,RejectAfter::None};
    auto result=shell.Evaluate(request,&token);ASSERT_EQ(result.status,Status::Ok)<<result.message;
    ASSERT_NE(shell.trial(),nullptr);
    input=Inputs(c,m,shell.accepted(),*shell.trial(),dt);
    ASSERT_EQ(crash_shell_phase_native(static_cast<int>(count),input.data(),native.data(),output.data()),0);
    Compare(*shell.trial(),native,output);
    ASSERT_EQ(shell.Commit(token),Status::Ok);
    for(std::size_t e=0;e<count;++e)native[e]=output[e].state;
  }
}

TEST(PersistentNativePhase, EvolvingFrozenAndCurrentReferenceAgreeWithOriginalRoutines) {
  EvolvingCase(GeometryMode::FrozenReference,1,false);
  EvolvingCase(GeometryMode::Current,1,false);
}
TEST(PersistentNativePhase, TwoDistinctSectionsAndUpdatedThicknessPreserveNativePhase) {
  EvolvingCase(GeometryMode::FrozenReference,2,true);
  EvolvingCase(GeometryMode::Current,2,true);
}
}  // namespace
}  // namespace tl::qualification::shell
