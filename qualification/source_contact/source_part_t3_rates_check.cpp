#include "SourcePartT3ReferenceInput.h"
#include "T3KinematicsTestOracle.h"
#include <gtest/gtest.h>
#include <filesystem>
#include <iostream>
#include <set>

namespace {
namespace source=crash::qualification::source_contact;
namespace native=tl::qualification::t3;
namespace test=native::kinematic_test;
std::filesystem::path readiness_path;

class SourcePartT3RatesCheck:public ::testing::Test {
 protected:
  source::SourcePartContactFixture fixture;
  void SetUp() override {
    const auto report=source::LoadPinnedSourcePartContact(readiness_path,&fixture);
    ASSERT_EQ(report.status,source::FixtureStatus::Ok)<<report.diagnostic;
  }
};

TEST_F(SourcePartT3RatesCheck, AllSixOriginalTrianglesRetainNativeRatesAndOriginalCoordinates) {
  const std::set<std::uint64_t> expected{2126272,2126274,2126283,2213538,2213540,2213595};
  std::set<std::uint64_t> visited;
  for(unsigned p=0;p<source::ParentCount;++p) {
    const auto& parent=fixture.parents()[p]; if(parent.arity!=3)continue;
    SCOPED_TRACE(parent.source_id);
    const auto input=source::T3ReferenceInput(fixture,p); const auto reference=test::MakeReference(input);
    const auto before=test::Bytes(reference);
    auto sample=test::Interval(input,2e-6); sample.sample_index=parent.source_id;
    for(unsigned n=0;n<3;++n) {
      // Prescribed world affine samples about this original first vertex.
      // No node is moved, projected, flattened, duplicated or mass-replaced.
      const auto dx=native::test::Difference(input.position[n],input.position[0]);
      sample.velocity[n]={double(.3L*dx[0]-.4L*dx[1]),double(.2L*dx[1]+.7L*dx[2]),double(-.6L*dx[0]+.5L*dx[2])};
      sample.angular_velocity[n]={double(.2L+3*dx[1]),double(-.1L+2*dx[2]),double(.3L-4*dx[0])};
    }
    native::Kinematics value;
    ASSERT_EQ(native::EvaluatePrescribed(reference,sample,value),native::Status::kSuccess);
    test::Check(value,sample);
    EXPECT_EQ(test::Bytes(reference),before);
    ASSERT_EQ(parent.local_node_indices[3],parent.local_node_indices[2]);
    for(unsigned n=0;n<3;++n) {
      EXPECT_EQ(input.node_ids[n],parent.raw_record[2+n]);
      const auto original=fixture.positions().at(parent.local_node_indices[n]);
      EXPECT_EQ(sample.position[n].x,original.x); EXPECT_EQ(sample.position[n].y,original.y); EXPECT_EQ(sample.position[n].z,original.z);
    }
    visited.insert(parent.source_id);
  }
  EXPECT_EQ(visited,expected);
  RecordProperty("source_readiness_sha256",source::ReadinessSha256);
  RecordProperty("native_t3_count",6);
  RecordProperty("native_scope","complete C3COOR3/C3EVEC3/C3DERI3/C3DEFO3/C3CURV3; prescribed endpoint/midpoint samples");
  RecordProperty("material_or_source_dynamics_admitted","false");
}

TEST_F(SourcePartT3RatesCheck, SourceSamplesAndNativeContextsRepeatWithoutHistoryOrClockAdvance) {
  for(unsigned p=0;p<source::ParentCount;++p) {
    if(fixture.parents()[p].arity!=3)continue;
    const auto input=source::T3ReferenceInput(fixture,p); const auto reference=test::MakeReference(input);
    auto sample=test::Interval(input,2e-6); sample.velocity.fill({.125,-.25,.375});
    native::Kinematics value;
    ASSERT_EQ(native::EvaluatePrescribed(reference,sample,value),native::Status::kSuccess);
    const auto accepted=test::Bytes(value);
    auto bad=sample; bad.position[2]=bad.position[1];
    EXPECT_EQ(native::EvaluatePrescribed(reference,bad,value),native::Status::kUnsupportedGeometry);
    EXPECT_EQ(test::Bytes(value),accepted);
    const auto other=test::MakeReference(test::Triangle(.02));
    native::Kinematics unused;
    ASSERT_EQ(native::EvaluatePrescribed(other,test::Interval(other.data().input),unused),native::Status::kSuccess);
    native::Kinematics retry;
    ASSERT_EQ(native::EvaluatePrescribed(reference,sample,retry),native::Status::kSuccess);
    EXPECT_EQ(retry.raw_rate,value.raw_rate); EXPECT_EQ(retry.normalized_rate,value.normalized_rate);
    EXPECT_EQ(retry.position_time,value.position_time); EXPECT_EQ(retry.velocity_time,value.velocity_time);
    test::Check(retry,sample);
  }
}
}  // namespace

int main(int argc,char** argv) {
  ::testing::InitGoogleTest(&argc,argv);
  if(argc!=2) { std::cerr<<"Usage: source_part_t3_rates_check READINESS.json\n"; return 2; }
  readiness_path=argv[1]; return RUN_ALL_TESTS();
}
