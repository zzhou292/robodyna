// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NativeOracle.h"
#include "lib_src/collision/radioss_type25/activity_operands/Values.h"
#include <cuda_runtime.h>
#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include <vector>
namespace native_activity_operand_value_test {
namespace n=tlfea::contact::radioss_type25;
namespace d=n::activity_operands::detail;
namespace ref=type25_activity_native;
struct Input {
  double coefficient=0;std::int32_t connected=0;std::uint32_t events=0;
  bool supported=false; n::activity_source::Controls controls;
  double secondary=17,scale=1000;
};
struct Output {
  d::MainResult main;double marked=0,normalized=0,si=77;bool scaled=false;
};
__global__ void Evaluate(const Input* inputs,Output* outputs,unsigned count) {
  const unsigned i=blockIdx.x*blockDim.x+threadIdx.x;if(i>=count)return;
  const auto& x=inputs[i];Output y;
  y.main=d::Main(x.coefficient,x.connected,x.events,x.supported,x.controls);
  y.marked=d::MarkSecondary(x.secondary,x.supported,x.controls);
  y.normalized=d::NormalizeSecondary(y.marked);
  y.scaled=d::Scale(x.coefficient,x.scale,y.si);outputs[i]=y;
}
struct Device {
  Input* input=nullptr;Output* output=nullptr;cudaStream_t stream=nullptr;
  ~Device(){if(input)cudaFree(input);if(output)cudaFree(output);if(stream)cudaStreamDestroy(stream);}
};
void RunDeviceValues(const std::vector<Input>& input,std::vector<Output>& output) {
  Device device;output.resize(input.size());
  ASSERT_EQ(cudaStreamCreateWithFlags(&device.stream,cudaStreamNonBlocking),cudaSuccess);
  ASSERT_EQ(cudaMalloc(&device.input,input.size()*sizeof(Input)),cudaSuccess);
  ASSERT_EQ(cudaMalloc(&device.output,input.size()*sizeof(Output)),cudaSuccess);
  ASSERT_EQ(cudaMemcpyAsync(device.input,input.data(),input.size()*sizeof(Input),cudaMemcpyHostToDevice,device.stream),cudaSuccess);
  Evaluate<<<1,32,0,device.stream>>>(device.input,device.output,unsigned(input.size()));
  ASSERT_EQ(cudaGetLastError(),cudaSuccess);
  ASSERT_EQ(cudaMemcpyAsync(output.data(),device.output,output.size()*sizeof(Output),cudaMemcpyDeviceToHost,device.stream),cudaSuccess);
  ASSERT_EQ(cudaStreamSynchronize(device.stream),cudaSuccess);
}
ref::Element Solid(double off) {
  ref::Element e;e.kind=ref::Kind::Solid8;e.nodes={1,2,3,4,5,6,7,8};e.off=off;return e;
}
ref::Element Shell(bool triangle,double off) {
  ref::Element e;e.kind=triangle?ref::Kind::Triangle:ref::Kind::QuadShell;
  e.nodes={1,2,3,4,0,0,0,0};e.off=off;return e;
}
TEST(NativeActivityOperandValuesCuda, NativeDiscoveryAndSequentialExposureAgreeWithoutFittedOutputs) {
  std::vector<ref::SurfaceCase> cases(5);
  for(auto& c:cases){c.corners={{{1,2,3,4}}};c.coefficients={-17};c.deletion=1;c.solid_erosion=true;c.connected_elements={2};}
  // A T3 face affects the larger main even while both solid owners remain active.
  cases[0].mesh={8,{Solid(1),Solid(1),Shell(true,0)}};
  // Native2->1->0 exposure is observable despite the final zero counter.
  cases[1].mesh={8,{Solid(0),Solid(0),Shell(false,1)}};
  // Complete current support is gone: native deletion takes priority over exposure.
  cases[2].mesh={8,{Solid(0),Solid(0)}};
  // Repeated events on an already-zero main still remain removal events.
  cases[3]=cases[2];cases[3].coefficients={0};
  // Ordinary nonnegative mains retain negative native bookkeeping counters.
  cases[4].mesh={4,{Shell(false,0),Shell(false,1)}};cases[4].coefficients={17};cases[4].connected_elements={0};
  std::vector<Input> inputs;std::vector<ref::SurfaceResult> native;
  for(auto& c:cases){c.affected=ref::Discover(c.mesh,c.corners);native.push_back(ref::Surfaces(c));
    inputs.push_back({c.coefficients[0],c.connected_elements[0],std::uint32_t(c.affected.size()),
      !ref::Removed(c.mesh,c.corners[0]),{n::activity_source::Deletion::ContainingElement,false,n::startup::SolidErosion::Enabled}});}
  ASSERT_EQ(cases[0].affected,(std::vector<int>{1}));
  ASSERT_EQ(cases[1].affected,(std::vector<int>{1,1}));
  std::vector<Output> outputs;RunDeviceValues(inputs,outputs);
  ASSERT_FALSE(HasFailure());
  for(unsigned i=0;i<inputs.size();++i){SCOPED_TRACE(i);const auto& actual=outputs[i].main;const auto& expected=native[i];
    EXPECT_TRUE(actual.valid);
  EXPECT_EQ(actual.connected,expected.connected_elements[0]);
    EXPECT_EQ(actual.removed,!expected.removed.empty());
  EXPECT_EQ(actual.exposure,!expected.exposed.empty());
    // Exposure is an explicitly rejected capability, never a published coefficient.
    if(!actual.exposure)EXPECT_DOUBLE_EQ(actual.coefficient,expected.coefficients[0]);
    const auto secondary=ref::Secondaries(expected.tags,{1},{17},true);
    EXPECT_DOUBLE_EQ(outputs[i].marked,secondary.marked_coefficients[0]);
    EXPECT_DOUBLE_EQ(outputs[i].normalized,secondary.marked_coefficients[0]<0?0:secondary.marked_coefficients[0]);
    EXPECT_TRUE(outputs[i].scaled);
  EXPECT_DOUBLE_EQ(outputs[i].si,inputs[i].coefficient*1000);
  }
  EXPECT_EQ(native[3].removed.size(),2u);
  EXPECT_EQ(native[3].coefficients[0],0);
  EXPECT_FALSE(outputs[2].main.exposure);
  EXPECT_TRUE(outputs[0].main.exposure);
  EXPECT_TRUE(outputs[1].main.exposure);
}
TEST(NativeActivityOperandValuesCuda, WallRetentionSignedZeroAndScaleFailuresPreserveTheirContracts) {
  const n::activity_source::Controls wall{n::activity_source::Deletion::Disabled,false,n::startup::SolidErosion::Disabled};
  std::vector<Input> inputs{{17,0,2,false,wall,17,1000},{-0.,0,0,false,wall,-0.,1000},
    {std::numeric_limits<double>::denorm_min(),0,0,true,wall,17,.5},
    {std::numeric_limits<double>::quiet_NaN(),0,0,true,wall,17,1}};
  std::vector<Output> outputs;RunDeviceValues(inputs,outputs);
  ASSERT_FALSE(HasFailure());
  EXPECT_DOUBLE_EQ(outputs[0].main.coefficient,17);
  EXPECT_FALSE(outputs[0].main.removed);
  EXPECT_DOUBLE_EQ(outputs[0].marked,17);
  EXPECT_DOUBLE_EQ(outputs[0].normalized,17);
  EXPECT_TRUE(std::signbit(outputs[1].main.coefficient));
  EXPECT_TRUE(std::signbit(outputs[1].marked));
  EXPECT_TRUE(std::signbit(outputs[1].normalized));
  EXPECT_TRUE(std::signbit(outputs[1].si));
  EXPECT_FALSE(outputs[2].scaled);
  EXPECT_DOUBLE_EQ(outputs[2].si,77);
  EXPECT_FALSE(outputs[3].scaled);
  EXPECT_DOUBLE_EQ(outputs[3].si,77);
  EXPECT_FALSE(outputs[3].main.valid);
}
}
