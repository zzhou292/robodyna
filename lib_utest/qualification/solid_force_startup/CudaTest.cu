// SPDX-License-Identifier: AGPL-3.0-or-later
#include "../solid18_force/PacketValues.h"
#include "../solid24_force/NativeOracle.h"
#include "../solid6z_force/TestSupport.h"
#include "../solid6z_force/Compare.h"
#include "lib_src/elements/solids/ForceStiffness.h"
#include <cuda_runtime.h>

namespace {
namespace a=tl::fea::solid18;
namespace b=tl::fea::solid24;
namespace c=tl::fea::solid6z;
struct Input {
  a::Reference reference18; a::Material material18; a::PrescribedInterval interval18;
  b::Reference reference24; b::Material material42; b::PrescribedInterval interval24;
  c::Reference reference6z; c::PrescribedInterval interval6z;
};
struct Output {
  a::ForceTrial initial18,rejected18,next18,retry18;
  b::ForceTrial initial24,rejected24,next24,retry24;
  c::ForceTrial initial6z,rejected6z,next6z,retry6z;
  int status[12]{};
  tl::fea::solids::NodalStiffness stiffness[3];
  bool stiffness_valid=false;
};
__global__ void EvaluateDeviceConstructor(const Input* input,Output* output) {
  const auto& i=*input; auto& o=*output;
  const a::Vec3 velocity{11.123,-.37,.129};
  o.status[0]=int(a::InitializeForce(i.reference18,i.material18,velocity,o.initial18));
  o.status[1]=int(b::InitializeForce(i.reference24,i.material42,velocity,o.initial24));
  o.status[2]=int(c::InitializeForce(i.reference6z,i.material42,{},velocity,o.initial6z));
  if(o.status[0] || o.status[1] || o.status[2]) return;
  // Each kernel history originates here. No host/production history seeds an oracle.
  auto bad18=i.interval18; auto bad24=i.interval24; auto bad6z=i.interval6z;
  bad18.position_endpoint_m[7].z=nan("");
  bad24.position_m[7].z=nan("");
  bad6z.position_endpoint_m[5].z=nan("");
  o.rejected18=o.initial18;o.rejected24=o.initial24;o.rejected6z=o.initial6z;
  o.status[3]=int(a::EvaluateForce(i.reference18,o.initial18.proposed_history,bad18,i.material18,o.rejected18));
  o.status[4]=int(b::EvaluateForce(i.reference24,o.initial24.proposed_history,bad24,i.material42,o.rejected24));
  o.status[5]=int(c::EvaluateForce(i.reference6z,o.initial6z.proposed_history,bad6z,i.material42,{},o.rejected6z));
  o.status[6]=int(a::EvaluateForce(i.reference18,o.initial18.proposed_history,i.interval18,i.material18,o.next18));
  o.status[7]=int(b::EvaluateForce(i.reference24,o.initial24.proposed_history,i.interval24,i.material42,o.next24));
  o.status[8]=int(c::EvaluateForce(i.reference6z,o.initial6z.proposed_history,i.interval6z,i.material42,{},o.next6z));
  o.status[9]=int(a::EvaluateForce(i.reference18,o.initial18.proposed_history,i.interval18,i.material18,o.retry18));
  o.status[10]=int(b::EvaluateForce(i.reference24,o.initial24.proposed_history,i.interval24,i.material42,o.retry24));
  o.status[11]=int(c::EvaluateForce(i.reference6z,o.initial6z.proposed_history,i.interval6z,i.material42,{},o.retry6z));
  o.stiffness_valid=tl::fea::solids::PrepareNodalStiffness(o.next18,o.stiffness[0]) &&
      tl::fea::solids::PrepareNodalStiffness(o.next24,o.stiffness[1]) &&
      tl::fea::solids::PrepareNodalStiffness(o.next6z,o.stiffness[2]);
}
// Exact named packets, never C++ padding or borrowed curve-pointer bytes.
template<class Array> void SameArray(const Array& x,const Array& y) {
  EXPECT_EQ(std::memcmp(std::data(x),std::data(y),std::size(x)*sizeof(double)),0);
}
void Same18(const a::ForceTrial& x,const a::ForceTrial& y) {
  const auto p=solid18_force_test::Values(x),q=solid18_force_test::Values(y);
  SameArray(p.next.point,q.next.point);SameArray(p.next.global,q.next.global);
  SameArray(p.next.saved,q.next.saved);SameArray(p.force,q.force);
  SameArray(p.geometry,q.geometry);SameArray(p.observation,q.observation);SameArray(p.diagnostics,q.diagnostics);
  EXPECT_EQ(x.proposed_history.stamp().sample_index,y.proposed_history.stamp().sample_index);
  EXPECT_EQ(x.proposed_history.stamp().time_s,y.proposed_history.stamp().time_s);
}
void Same6z(const c::ForceTrial& x,const c::ForceTrial& y) {
  const auto p=solid6z_force_test::Pack(x),q=solid6z_force_test::Pack(y);
  SameArray(p.geometry,q.geometry);SameArray(p.material,q.material);SameArray(p.history,q.history);
  SameArray(p.forces,q.forces);SameArray(p.stabilization,q.stabilization);
  const double ax[]{x.stabilization.first_work_j,x.stabilization.second_work_j,x.total_internal_work_increment_j};
  const double ay[]{y.stabilization.first_work_j,y.stabilization.second_work_j,y.total_internal_work_increment_j};
  SameArray(ax,ay);
  EXPECT_EQ(x.proposed_history.stamp().sample_index,y.proposed_history.stamp().sample_index);
  EXPECT_EQ(x.proposed_history.stamp().time_s,y.proposed_history.stamp().time_s);
}
TEST(SolidStartupCuda, ThreeDeviceOwnedConstructorsRejectLateInputsAndRetryExactNativeInterval) {
  Input input;
  input.reference18=solid18_force_test::Reference(); input.material18=solid18_force_test::Material();
  input.reference24=heph_test::Reference();
  input.material42=heph_test::Material(input.reference24.input().density_kg_m3);
  input.reference6z=solid6z_force_test::Reference();
  input.interval18=solid18_force_test::Path(input.reference18,0);
  b::History virgin;ASSERT_EQ(b::InitializeHistory(input.reference24,input.material42,virgin),b::ForceStatus::Success);
  input.interval24=heph_test::Interval(input.reference24,virgin);
  for(unsigned n=0;n<8;++n)input.interval24.position_m[n].x*=1.001;
  input.interval6z=solid6z_force_test::Path(input.reference6z,0);
  const auto host_material=input.material18;
  const auto points=host_material.curve.count;
  double* curve=nullptr;Input* device_input=nullptr;Output* device_output=nullptr;
  ASSERT_EQ(cudaMalloc(&curve,2*points*sizeof(double)),cudaSuccess);
  ASSERT_EQ(cudaMalloc(&device_input,sizeof(Input)),cudaSuccess);
  ASSERT_EQ(cudaMalloc(&device_output,sizeof(Output)),cudaSuccess);
  ASSERT_EQ(cudaMemcpy(curve,host_material.curve.plastic_strain,points*sizeof(double),cudaMemcpyHostToDevice),cudaSuccess);
  ASSERT_EQ(cudaMemcpy(curve+points,host_material.curve.yield_stress_pa,points*sizeof(double),cudaMemcpyHostToDevice),cudaSuccess);
  input.material18.curve.plastic_strain=curve;input.material18.curve.yield_stress_pa=curve+points;
  ASSERT_EQ(cudaMemcpy(device_input,&input,sizeof(input),cudaMemcpyHostToDevice),cudaSuccess);
  EvaluateDeviceConstructor<<<1,1>>>(device_input,device_output);
  ASSERT_EQ(cudaGetLastError(),cudaSuccess);ASSERT_EQ(cudaDeviceSynchronize(),cudaSuccess);
  const auto output=std::make_unique<Output>();
  ASSERT_EQ(cudaMemcpy(output.get(),device_output,sizeof(Output),cudaMemcpyDeviceToHost),cudaSuccess);
  EXPECT_EQ(cudaFree(device_output),cudaSuccess);EXPECT_EQ(cudaFree(device_input),cudaSuccess);EXPECT_EQ(cudaFree(curve),cudaSuccess);
  const auto& o=*output;
  for(unsigned k=0;k<12;++k) {
    if(k>=3 && k<6) EXPECT_NE(o.status[k],0) << k;
    else ASSERT_EQ(o.status[k],0) << k;
  }
  ASSERT_TRUE(o.stiffness_valid);
  Same18(o.initial18,o.rejected18);Same18(o.next18,o.retry18);
  EXPECT_TRUE(heph_test::Same(o.initial24,o.rejected24));EXPECT_TRUE(heph_test::Same(o.next24,o.retry24));
  Same6z(o.initial6z,o.rejected6z);Same6z(o.next6z,o.retry6z);
  const a::Vec3 v{11.123,-.37,.129};
  a::PrescribedInterval ia;b::PrescribedInterval ib;c::PrescribedInterval ic;
  for(unsigned n=0;n<8;++n){ia.position_endpoint_m[n]=input.reference18.input().position_m[n];ia.velocity_midpoint_m_s[n]=v;
    ib.position_m[n]=input.reference24.input().position_m[n];ib.velocity_m_s[n]=v;}
  for(unsigned n=0;n<6;++n){ic.position_endpoint_m[n]=input.reference6z.input().position_m[n];ic.velocity_midpoint_m_s[n]=v;}
  auto na=solid18_force_test::Native(host_material,solid18_force_test::NativeInitial(input.reference18.input()),ia);
  ASSERT_TRUE(solid18_force_test::Agree(o.initial18,na));
  ASSERT_TRUE(solid18_force_test::Agree(o.next18,solid18_force_test::Native(host_material,na.next,input.interval18)));
  auto nb=heph_test::InitializeNative(input.reference24.input());
  const auto eb=heph_test::NativeStep(nb,ib,input.material42);heph_test::Compare(o.initial24,eb);heph_test::AcceptNative(eb,nb);
  heph_test::Compare(o.next24,heph_test::NativeStep(nb,input.interval24,input.material42));
  solid6z_force_test::NativeHistory nc;ASSERT_TRUE(nc.Initialize(input.reference6z.input(),input.material42));
  const auto ec=nc.InitializeForce(v);ASSERT_TRUE(solid6z_force_test::Agree(solid6z_force_test::Pack(o.initial6z),ec));nc.Accept(ec);
  ASSERT_TRUE(solid6z_force_test::Agree(solid6z_force_test::Pack(o.next6z),nc.Evaluate(input.interval6z)));
}
} // namespace
