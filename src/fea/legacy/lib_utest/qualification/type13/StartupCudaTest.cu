// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
#include <gtest/gtest.h>
#include <cuda_runtime.h>
#include <array>
#include <type_traits>

namespace type13_test {
struct Packet { t::Property property{};t::Startup startup{};t::Status property_status{},element_status{}; };
static_assert(std::is_trivially_copyable_v<Packet>);
__global__ void StartupPacket(Packet* result,unsigned mode) {
  Fixture fixture;
  if(mode==1)fixture.points[3][4].x=fixture.points[3][3].x;
  auto reference=DenseReference();
  if(mode==2)reference.endpoint_release[3]=1;
  result->property_status=t::InitializeProperty(fixture.Input(),result->property);
  if(result->property_status==t::Status::Success)
    result->element_status=t::InitializeElement(result->property,reference,result->startup);
}
std::array<double,22> Values(const t::Startup& s) {
  std::array<double,22> out{};unsigned n=0;
  for(const auto& p:s.reference.position_m){out[n++]=p.x;out[n++]=p.y;out[n++]=p.z;}
  for(double x:s.reference.axes.v)out[n++]=x;
  out[n++]=s.reference.length_native;out[n++]=s.reference.length_m;
  out[n++]=s.reference.third_node_alignment;out[n++]=static_cast<unsigned>(s.reference.branch);
  out[n++]=s.endpoint.mass_kg;out[n++]=s.endpoint.isotropic_inertia_kg_m2;out[n++]=s.endpoint.added_inertia_kg_m2;
  return out;
}
TEST(Type13Cuda,ActualDeviceOriginalPropertyFrameAndCoefficients) {
  Fixture fixture;Packet expected;
  ASSERT_EQ(t::InitializeProperty(fixture.Input(),expected.property),t::Status::Success);
  ASSERT_EQ(t::InitializeElement(expected.property,DenseReference(),expected.startup),t::Status::Success);
  Packet* device=nullptr;ASSERT_EQ(cudaMalloc(&device,sizeof(Packet)),cudaSuccess);
  Packet actual{};ASSERT_EQ(cudaMemcpy(device,&actual,sizeof(actual),cudaMemcpyHostToDevice),cudaSuccess);
  StartupPacket<<<1,1>>>(device,0);
  ASSERT_EQ(cudaGetLastError(),cudaSuccess);
  ASSERT_EQ(cudaMemcpy(&actual,device,sizeof(actual),cudaMemcpyDeviceToHost),cudaSuccess);
  ASSERT_EQ(cudaFree(device),cudaSuccess);
  ASSERT_EQ(actual.property_status,t::Status::Success);ASSERT_EQ(actual.element_status,t::Status::Success);
  EXPECT_EQ(Values(actual.startup),Values(expected.startup));
  for(unsigned c=0;c<6;++c) {
    EXPECT_EQ(actual.property.channel(c).native_stiffness,expected.property.channel(c).native_stiffness);
    EXPECT_EQ(actual.property.channel(c).stiffness_si,expected.property.channel(c).stiffness_si);
    EXPECT_EQ(actual.property.channel(c).declaration.curve_index,expected.property.channel(c).declaration.curve_index);
  }
  for(unsigned c=0;c<4;++c)for(unsigned k=0;k<5;++k) {
    EXPECT_EQ(actual.property.curve(c).points[k].x,expected.property.curve(c).points[k].x);
    EXPECT_EQ(actual.property.curve(c).points[k].y,expected.property.curve(c).points[k].y);
  }
}
TEST(Type13Cuda,LateFailurePreservesDestinationAndExactRetry) {
  Fixture fixture;Packet baseline;
  ASSERT_EQ(t::InitializeProperty(fixture.Input(),baseline.property),t::Status::Success);
  ASSERT_EQ(t::InitializeElement(baseline.property,DenseReference(),baseline.startup),t::Status::Success);
  baseline.element_status=t::Status::InvalidInput;
  Packet* device=nullptr;ASSERT_EQ(cudaMalloc(&device,sizeof(Packet)),cudaSuccess);
  ASSERT_EQ(cudaMemcpy(device,&baseline,sizeof(baseline),cudaMemcpyHostToDevice),cudaSuccess);
  for(unsigned mode:{1u,2u,0u}) {
    StartupPacket<<<1,1>>>(device,mode);ASSERT_EQ(cudaGetLastError(),cudaSuccess);
    Packet actual;ASSERT_EQ(cudaMemcpy(&actual,device,sizeof(actual),cudaMemcpyDeviceToHost),cudaSuccess);
    EXPECT_EQ(Values(actual.startup),Values(baseline.startup));
    EXPECT_EQ(actual.property.curve(3).points[4].x,baseline.property.curve(3).points[4].x);
    EXPECT_EQ(actual.property_status,mode==1?t::Status::InvalidInput:t::Status::Success);
    EXPECT_EQ(actual.element_status,mode==1?t::Status::InvalidInput:
      mode==2?t::Status::UnsupportedScope:t::Status::Success);
  }
  EXPECT_EQ(cudaFree(device),cudaSuccess);
}
} // namespace type13_test
