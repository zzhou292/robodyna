#include "SourcePartT3ReferenceInput.h"
#include "T3ForceTestOracle.h"
#include "T3EngineContext.h"
#include <gtest/gtest.h>
#include <filesystem>
#include <iostream>
#include <set>

namespace {
namespace source=crash::qualification::source_contact;
namespace native=tl::qualification::t3;
namespace kt=native::kinematic_test;
namespace oracle=native::force_test;
std::filesystem::path readiness_path;

class SourcePartT3ForceCheck:public ::testing::Test {
 protected:
  source::SourcePartContactFixture fixture;
  void SetUp() override {
    const auto report=source::LoadPinnedSourcePartContact(readiness_path,&fixture);
    ASSERT_EQ(report.status,source::FixtureStatus::Ok)<<report.diagnostic;
  }
};

TEST_F(SourcePartT3ForceCheck, AllSixOriginalTrianglesRetainNativeMaterialAndForceModes) {
  const std::set<std::uint64_t> expected{2126272,2126274,2126283,2213538,2213540,2213595};
  std::set<std::uint64_t> visited;
  for(unsigned p=0;p<source::ParentCount;++p) {
    const auto& parent=fixture.parents()[p]; if(parent.arity!=3)continue;
    SCOPED_TRACE(parent.source_id);
    const auto input=source::T3ReferenceInput(fixture,p); const auto reference=kt::MakeReference(input);
    const auto reference_bytes=kt::Bytes(reference);
    for(unsigned mode=0;mode<8;++mode) {
      SCOPED_TRACE(mode);
      const auto base=oracle::MakeHistory(reference);
      auto sample=kt::Interval(input,2e-6); oracle::Mode(sample,mode,.001);
      native::ForceTrial value;
      ASSERT_EQ(native::EvaluateForce(reference,base,sample,value),native::Status::kSuccess);
      oracle::Check(reference,base.data(),sample,value);
      oracle::CheckFixedResultantPower(input,value);
      EXPECT_EQ(kt::Bytes(reference),reference_bytes);
      EXPECT_EQ(reference.data().input.density,fixture.surface_mass().density_kg_m3);
      EXPECT_EQ(reference.data().input.thickness,fixture.surface_mass().thickness_m);
      for(unsigned n=0;n<3;++n) {
        EXPECT_EQ(input.node_ids[n],parent.raw_record[2+n]);
        const auto x=fixture.positions().at(parent.local_node_indices[n]);
        EXPECT_EQ(sample.position[n].x,x.x); EXPECT_EQ(sample.position[n].y,x.y); EXPECT_EQ(sample.position[n].z,x.z);
      }
    }
    EXPECT_EQ(parent.local_node_indices[3],parent.local_node_indices[2]);
    ASSERT_TRUE(visited.insert(parent.source_id).second);
  }
  EXPECT_EQ(visited,expected);
  RecordProperty("source_readiness_sha256",source::ReadinessSha256);
  RecordProperty("native_t3_count",6);
  RecordProperty("prescribed_material_modes",48);
  RecordProperty("native_scope","complete T3 geometry/rate/force leaves and SIGEPS01G; selected PM/EPSD/MULAWGLC adapters");
  RecordProperty("material_experiment","E=200GPa nu=.3 LAW1/NPT0/ISH3N2; original MAT024/NIP3 not consumed");
  RecordProperty("source_formulation_or_dynamics_admitted","false");
}

TEST_F(SourcePartT3ForceCheck, SourceHistoryRetryAndNativeSignedContributionsMapOnceToPhysicalNodes) {
  std::array<long double,3*source::NodeCount> force{},couple{},expected_force{},expected_couple{};
  std::array<long double,source::NodeCount> stiffness{},rotary{},expected_stiffness{},expected_rotary{};
  unsigned count=0;
  for(unsigned p=0;p<source::ParentCount;++p) {
    const auto& parent=fixture.parents()[p]; if(parent.arity!=3)continue;
    SCOPED_TRACE(parent.source_id);
    const auto input=source::T3ReferenceInput(fixture,p); const auto reference=kt::MakeReference(input);
    auto history=oracle::MakeHistory(reference,oracle::Seed(input.thickness));
    native::ForceTrial value;
    for(double load:{.001,0.,-.001}) {
      auto sample=kt::Interval(input,2e-6); sample.base_time=history.stamp().time;
      sample.sample_index=history.stamp().sample_index+1; oracle::Mode(sample,2,load);
      ASSERT_EQ(native::EvaluateForce(reference,history,sample,value),native::Status::kSuccess);
      oracle::Check(reference,history.data(),sample,value);
      oracle::CheckFixedResultantPower(input,value);
      const auto saved=kt::Bytes(value);
      const auto base=kt::Bytes(history);
      auto invalid=sample; invalid.position[2]=invalid.position[1];
      EXPECT_EQ(native::EvaluateForce(reference,history,invalid,value),native::Status::kUnsupportedGeometry);
      EXPECT_EQ(kt::Bytes(value),saved); EXPECT_EQ(kt::Bytes(history),base);
      native::ForceTrial retry;
      ASSERT_EQ(native::EvaluateForce(reference,history,sample,retry),native::Status::kSuccess);
      std::array<double,26> a{},b{};
      native::detail::PackHistory(value.proposed_history.data(),a);
      native::detail::PackHistory(retry.proposed_history.data(),b); EXPECT_EQ(a,b);
      history=retry.proposed_history;
    }
    // Invoke the owning bounded native scatter in its local four-node buffer,
    // then reduce its three physical contributions through original source IDs.
    // This is host qualification composition, not a resident 117-node owner.
    std::array<double,9> f{},m{}; native::detail::PackVectors(value.internal_force,f);
    native::detail::PackVectors(value.internal_couple,m);
    const std::array<double,2> k{value.diagnostics.translational_stiffness,value.diagnostics.rotational_stiffness};
    std::array<double,12> rhs{},moment{}; std::array<double,4> kn{},kr{};
    const std::array<int,3> map{1,2,3};
    {
      const std::lock_guard<std::mutex> lock(native::detail::NativeEngineContext());
      native::detail::t3_r3_scatter(map.data(),f.data(),m.data(),k.data(),rhs.data(),moment.data(),kn.data(),kr.data());
    }
    for(unsigned n=0;n<3;++n) {
      const auto index=parent.local_node_indices[n];
      stiffness[index]+=kn[n]; rotary[index]+=kr[n];
      expected_stiffness[index]+=k[0]; expected_rotary[index]+=k[1];
      for(unsigned axis=0;axis<3;++axis) {
        force[3*index+axis]+=rhs[3*n+axis]; couple[3*index+axis]+=moment[3*n+axis];
        expected_force[3*index+axis]-=f[3*n+axis]; expected_couple[3*index+axis]-=m[3*n+axis];
      }
    }
    EXPECT_EQ(rhs[9],0); EXPECT_EQ(rhs[10],0); EXPECT_EQ(rhs[11],0);
    EXPECT_EQ(kn[3],0); EXPECT_EQ(kr[3],0); ++count;
  }
  EXPECT_EQ(count,6u); EXPECT_EQ(force,expected_force); EXPECT_EQ(couple,expected_couple);
  EXPECT_EQ(stiffness,expected_stiffness); EXPECT_EQ(rotary,expected_rotary);
  RecordProperty("native_scatter","complete C3UPDT3 per parent; original117-node host mapping once");
  RecordProperty("material_history_owner_or_full_part_dynamics_admitted","false");
}
} // namespace

int main(int argc,char** argv) {
  ::testing::InitGoogleTest(&argc,argv);
  if(argc!=2) { std::cerr<<"Usage: source_part_t3_force_check READINESS.json\n"; return 2; }
  readiness_path=argv[1]; return RUN_ALL_TESTS();
}
