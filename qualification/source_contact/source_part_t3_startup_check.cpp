#include "SourcePartContactFixture.h"
#include "T3Reference.h"
#include "T3StartupTestOracle.h"

#include <gtest/gtest.h>
#include <array>
#include <filesystem>
#include <iostream>
#include <set>

namespace {
namespace source=crash::qualification::source_contact;
namespace native=tl::qualification::t3;
std::filesystem::path readiness_path;

native::ReferenceInput Input(const source::SourcePartContactFixture& fixture,unsigned p) {
  native::ReferenceInput input;
  const auto& parent=fixture.parents()[p];
  for (unsigned n=0;n<3;++n) {
    const auto index=parent.local_node_indices[n];
    const auto x=fixture.positions().at(index);
    input.position[n]={x.x,x.y,x.z}; input.node_ids[n]=fixture.nodes()[index].source_id;
  }
  // Exact authenticated converted bytes. No averaging/flattening or proxy
  // nodal lumping is imported into the separate native startup experiment.
  input.density=fixture.surface_mass().density_kg_m3;
  input.thickness=fixture.surface_mass().thickness_m;
  input.young_modulus=200e9; input.poisson_ratio=.3;
  return input;
}
class SourcePartT3StartupCheck:public ::testing::Test {
 protected:
  source::SourcePartContactFixture fixture;
  void SetUp() override {
    const auto report=source::LoadPinnedSourcePartContact(readiness_path,&fixture);
    ASSERT_EQ(report.status,source::FixtureStatus::Ok)<<report.diagnostic;
  }
};

TEST_F(SourcePartT3StartupCheck, AllSixOriginalTrianglesUseNativeFrameAndSelectedMassExpressions) {
  const std::set<std::uint64_t> expected{2126272,2126274,2126283,2213538,2213540,2213595};
  std::set<std::uint64_t> visited;
  for (unsigned p=0;p<source::ParentCount;++p) {
    const auto& parent=fixture.parents()[p]; if (parent.arity!=3) continue;
    SCOPED_TRACE(parent.source_id);
    ASSERT_EQ(parent.local_node_indices[3],parent.local_node_indices[2]);
    ASSERT_EQ(parent.raw_record[5],parent.raw_record[4]);
    const auto input=Input(fixture,p); native::Reference reference;
    ASSERT_EQ(native::Initialize(input,reference),native::Status::kSuccess);
    native::test::Check(reference);
    EXPECT_EQ(reference.data().input.node_ids,input.node_ids);
    EXPECT_EQ(reference.data().input.density,fixture.surface_mass().density_kg_m3);
    EXPECT_EQ(reference.data().input.thickness,fixture.surface_mass().thickness_m);
    for (unsigned n=0;n<3;++n) {
      const auto& actual=reference.data().input.position[n]; const auto& original=input.position[n];
      EXPECT_EQ(actual.x,original.x); EXPECT_EQ(actual.y,original.y); EXPECT_EQ(actual.z,original.z);
      EXPECT_EQ(input.node_ids[n],parent.raw_record[2+n]);
    }
    ASSERT_TRUE(visited.insert(parent.source_id).second);
  }
  EXPECT_EQ(visited,expected);
  RecordProperty("source_readiness_sha256",source::ReadinessSha256);
  RecordProperty("native_t3_count",6);
  RecordProperty("native_reference_scope","complete C3EVEC3; selected C3DERII/C3INMAS/SPMD_MSIN expressions");
  RecordProperty("material_experiment","centered uniform elastic metadata E=200GPa nu=0.3; original MAT024 not consumed");
  RecordProperty("source_formulation_or_dynamics_qualified","false");
}

TEST_F(SourcePartT3StartupCheck, AllSixContributionsAssembleOnceAtOriginalPhysicalNodes) {
  std::array<long double,source::NodeCount> mass{},inertia{},expected_mass{},expected_inertia{};
  long double element_mass=0,element_inertia=0;
  unsigned count=0;
  for (unsigned p=0;p<source::ParentCount;++p) {
    const auto& parent=fixture.parents()[p]; if (parent.arity!=3) continue;
    const auto input=Input(fixture,p); native::Reference reference;
    ASSERT_EQ(native::Initialize(input,reference),native::Status::kSuccess);
    const auto independent=native::test::Independent(input);
    element_mass+=independent.mass; element_inertia+=independent.total; ++count;
    for (unsigned n=0;n<3;++n) {
      const auto index=parent.local_node_indices[n];
      mass[index]+=reference.data().nodal_mass[n]; inertia[index]+=reference.data().isotropic_inertia[n];
      expected_mass[index]+=independent.mass*independent.weight[n];
      expected_inertia[index]+=independent.total*independent.weight[n];
    }
  }
  ASSERT_EQ(count,6u);
  long double total_mass=0,total_inertia=0;
  for (unsigned n=0;n<source::NodeCount;++n) {
    native::test::Near(static_cast<double>(mass[n]),expected_mass[n],expected_mass[n]);
    native::test::Near(static_cast<double>(inertia[n]),expected_inertia[n],expected_inertia[n]);
    total_mass+=mass[n]; total_inertia+=inertia[n];
  }
  native::test::Near(static_cast<double>(total_mass),element_mass,element_mass);
  native::test::Near(static_cast<double>(total_inertia),element_inertia,element_inertia);
  RecordProperty("mass_policy","native T3 angle/pi; no equal-node or Q4 lumping");
  RecordProperty("full_part_structural_mass_admitted","false");
}
}  // namespace

int main(int argc,char** argv) {
  ::testing::InitGoogleTest(&argc,argv);
  if (argc!=2) { std::cerr<<"Usage: source_part_t3_startup_check READINESS.json\n"; return 2; }
  readiness_path=argv[1];
  return RUN_ALL_TESTS();
}
