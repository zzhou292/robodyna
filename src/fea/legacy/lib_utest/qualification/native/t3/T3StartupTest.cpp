#include "T3Reference.h"
#include "T3StartupTestOracle.h"
#include "lib_src/math/Quaternion.h"

#include <gtest/gtest.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <map>
#include <type_traits>

namespace {
namespace native=tl::qualification::t3;
using native::Reference;
using native::ReferenceInput;
using native::Status;
using native::Vec3;
using namespace native::test;
template<class T> auto Bytes(const T& object) {
  static_assert(std::is_trivially_copyable_v<T>);
  std::array<unsigned char,sizeof(T)> bytes{};
  std::memcpy(bytes.data(),&object,sizeof(T)); return bytes;
}
ReferenceInput Triangle(Vec3 a={0,0,0},Vec3 b={1,0,0},Vec3 c={0,1,0}) {
  ReferenceInput input;
  input.position={{a,b,c}}; input.node_ids={{1001,7,93}};
  input.density=10; input.thickness=.1; input.young_modulus=2e6; input.poisson_ratio=.25;
  return input;
}

TEST(T3StartupCheck, RightEquilateralAndScaleneAnalyticMass) {
  for (const auto input : {Triangle(),Triangle({0,0,0},{2,0,0},{1,std::sqrt(3.),0}),
                           Triangle({0,0,0},{3,0,0},{.7,1.3,0})}) {
    Reference reference;
    ASSERT_EQ(native::Initialize(input,reference),Status::kSuccess);
    Check(reference);
  }
  Reference right,equilateral;
  ASSERT_EQ(native::Initialize(Triangle(),right),Status::kSuccess);
  ASSERT_EQ(native::Initialize(Triangle({0,0,0},{2,0,0},{1,std::sqrt(3.),0}),equilateral),Status::kSuccess);
  Near(right.data().angle_weight[0],.5L,1);
  Near(right.data().angle_weight[1],.25L,1);
  Near(right.data().angle_weight[2],.25L,1);
  for (double weight:equilateral.data().angle_weight) Near(weight,1.L/3,1);
  RecordProperty("frame_reference","complete native starter C3EVEC3");
  RecordProperty("mass_reference","selected C3INMAS and SPMD_MSIN expressions");
  RecordProperty("full_starter_or_dynamics_qualified","false");
}

TEST(T3StartupCheck, ProperRotationCyclicReversalAndEdgeOnUseActualThreeNodes) {
  const auto input=Triangle({-.3,.1,.7},{1.2,.6,.4},{.2,1.4,1.1});
  const double rotation_vector[]{.37,-.63,.42}; tl::math::Quaternion q;
  ASSERT_TRUE(tl::math::IncrementWorldRotation({},rotation_vector,q));
  for (unsigned reverse=0;reverse<2;++reverse) for (unsigned shift=0;shift<3;++shift) {
    auto transformed=input;
    for (unsigned n=0;n<3;++n) {
      const unsigned index=(shift+(reverse?3-n:n))%3;
      const auto x=input.position[index];
      const auto y=tl::math::Product(tl::math::Product(q,{0,x.x,x.y,x.z}),{q.w,-q.x,-q.y,-q.z});
      transformed.position[n]={y.x+.7,y.y-.2,y.z+.4};
      transformed.node_ids[n]=input.node_ids[index];
    }
    Reference reference;
    ASSERT_EQ(native::Initialize(transformed,reference),Status::kSuccess);
    Check(reference);
  }
  Reference edge_on;
  ASSERT_EQ(native::Initialize(Triangle({0,0,0},{2,0,0},{.7,0,1}),edge_on),Status::kSuccess);
  Check(edge_on);
  EXPECT_NEAR(edge_on.data().frame.v[2],0,2e-12);  // normal.x=0, still a regular T3.
}

TEST(T3StartupCheck, DensityThicknessAndLengthScaleSeparateInertiaPartitions) {
  const auto base=Triangle({0,0,0},{1.7,0,0},{.3,.8,0});
  Reference baseline;
  ASSERT_EQ(native::Initialize(base,baseline),Status::kSuccess);
  for (unsigned change=0;change<3;++change) {
    auto input=base;
    if (change==0) input.density*=7;
    if (change==1) input.thickness*=3;
    if (change==2) for (auto& x:input.position) { x.x*=4; x.y*=4; x.z*=4; }
    Reference reference;
    ASSERT_EQ(native::Initialize(input,reference),Status::kSuccess);
    Check(reference);
    const auto& a=baseline.data(); const auto& b=reference.data();
    const double mass_factor=change==0?7:change==1?3:16;
    const double physical_factor=change==0?7:change==1?27:16;
    const double added_factor=change==0?7:change==1?3:256;
    for (unsigned n=0;n<3;++n) {
      Near(b.nodal_mass[n],mass_factor*a.nodal_mass[n],mass_factor*a.nodal_mass[n]);
      Near(b.physical_inertia[n],physical_factor*a.physical_inertia[n],physical_factor*a.physical_inertia[n]);
      Near(b.added_inertia[n],added_factor*a.added_inertia[n],added_factor*a.added_inertia[n]);
    }
  }
}

TEST(T3StartupCheck, SharedPhysicalNodesReceiveAngleWeightedContributionsOnce) {
  auto first=Triangle({0,0,0},{2,0,0},{1.3,1.1,0}); first.node_ids={{71,91,103}};
  auto second=Triangle({0,0,0},{1.3,1.1,0},{-.2,.7,0}); second.node_ids={{71,103,9}};
  std::map<std::uint64_t,std::array<long double,4>> sums;
  for (const auto& input:{first,second}) {
    Reference reference;
    ASSERT_EQ(native::Initialize(input,reference),Status::kSuccess);
    const auto expected=Independent(input);
    for (unsigned n=0;n<3;++n) {
      auto& sum=sums[input.node_ids[n]];
      sum[0]+=reference.data().nodal_mass[n]; sum[1]+=expected.mass*expected.weight[n];
      sum[2]+=reference.data().isotropic_inertia[n]; sum[3]+=expected.total*expected.weight[n];
    }
  }
  ASSERT_EQ(sums.size(),4u);
  for (const auto& pair:sums) {
    const auto& sum=pair.second;
    Near(static_cast<double>(sum[0]),sum[1],sum[1]);
    Near(static_cast<double>(sum[2]),sum[3],sum[3]);
  }
}

TEST(T3StartupCheck, MalformedInputsPreserveDefaultAndPreparedReferences) {
  Reference prepared;
  ASSERT_EQ(native::Initialize(Triangle(),prepared),Status::kSuccess);
  for (unsigned fault=0;fault<9;++fault) {
    SCOPED_TRACE(fault);
    auto input=Triangle();
    if (fault==0) input.density=0;
    if (fault==1) input.thickness=-1;
    if (fault==2) input.young_modulus=std::numeric_limits<double>::infinity();
    if (fault==3) input.poisson_ratio=.5;
    if (fault==4) input.position[2].z=std::numeric_limits<double>::quiet_NaN();
    if (fault==5) input.node_ids[2]=input.node_ids[0];
    if (fault==6) input.position[2]=input.position[1];
    if (fault==7) input.position[2]={2,0,0};
    if (fault==8) input.position[2].z=std::nextafter(native::kMaximumCoordinate,INFINITY);
    Reference empty;
    const auto old_empty=Bytes(empty),old_prepared=Bytes(prepared);
    EXPECT_NE(native::Initialize(input,empty),Status::kSuccess);
    EXPECT_NE(native::Initialize(input,prepared),Status::kSuccess);
    EXPECT_EQ(Bytes(empty),old_empty); EXPECT_EQ(Bytes(prepared),old_prepared);
  }
}

TEST(T3StartupCheck, FrozenConditioningAndNativeAcosBoundaryAreNotClamped) {
  Reference reference;
  ASSERT_EQ(native::Initialize(Triangle(),reference),Status::kSuccess);
  const auto before=Bytes(reference);
  // Area and edge-ratio gates pass, but the small acute angle is closer to
  // ACOS(+1) than the independently frozen 1e-12 margin: diagnosed rejection.
  auto skinny=Triangle({0,0,0},{1,0,0},{1,1.1e-6,0});
  EXPECT_EQ(native::Initialize(skinny,reference),Status::kUnsupportedGeometry);
  EXPECT_EQ(Bytes(reference),before);
  skinny.position[2].y=2e-6;
  ASSERT_EQ(native::Initialize(skinny,reference),Status::kSuccess);
  for (double cosine:reference.data().angle_cosine)
    EXPECT_LE(std::abs(cosine),1-native::kAcosBoundaryMargin);
  const auto accepted=Bytes(reference);
  for (double length:{.5e-9,1.1e3}) {
    const auto outside=Triangle({0,0,0},{length,0,0},{0,length,0});
    EXPECT_EQ(native::Initialize(outside,reference),Status::kUnsupportedGeometry);
    EXPECT_EQ(Bytes(reference),accepted);
  }
}

TEST(T3StartupCheck, LateUnrepresentableInertiaPreservesOutputAndRetryMatchesClean) {
  Reference output,clean;
  ASSERT_EQ(native::Initialize(Triangle(),output),Status::kSuccess);
  const auto before=Bytes(output);
  auto underflow=Triangle(); underflow.thickness=1e-200;
  EXPECT_EQ(native::Initialize(underflow,output),Status::kNonfiniteResult);
  EXPECT_EQ(Bytes(output),before);
  auto overflow=Triangle(); overflow.density=std::numeric_limits<double>::max(); overflow.thickness=4;
  EXPECT_EQ(native::Initialize(overflow,output),Status::kNonfiniteResult);
  EXPECT_EQ(Bytes(output),before);
  const auto retry=Triangle({0,0,0},{2,0,0},{.2,.9,0});
  ASSERT_EQ(native::Initialize(retry,output),Status::kSuccess);
  ASSERT_EQ(native::Initialize(retry,clean),Status::kSuccess);
  // Padding is not an equality oracle on successful assignment.
  EXPECT_EQ(output.data().angle_weight,clean.data().angle_weight);
  EXPECT_EQ(output.data().nodal_mass,clean.data().nodal_mass);
  EXPECT_EQ(output.data().isotropic_inertia,clean.data().isotropic_inertia);
  EXPECT_EQ(output.data().startup_derivative,clean.data().startup_derivative);
  EXPECT_EQ(output.data().area,clean.data().area);
}
}  // namespace
