// SPDX-License-Identifier: MIT
#include "Fixture.h"

namespace active_use_test {
namespace {
struct CounterfeitMergedParts {
  fe::rigid::NodalRigidPartTopology topology;
  fe::rigid::NodalRigidPartAssemblyModel parts;
  fe::NodalRigidAssemblyBinding binding;
  explicit CounterfeitMergedParts(const ExecutionFixture& fixture) {
    const std::array<std::uint64_t,8> ids{{10,11,12,13,14,20,21,22}};
    const fe::rigid::PartTopologyPartInput declarations[]{
        {1000,ids.data(),5},{2000,ids.data()+5,3}};
    const fe::rigid::PartTopologyMerge merge{1000,2000};
    fe::rigid::PartTopologyInput input;
    input.source_instance_id=77;
    input.parts=declarations;
    input.part_count=2;
    input.expected_members=ids.data();
    input.expected_member_count=ids.size();
    input.merges=&merge;
    input.merge_count=1;
    EXPECT_TRUE(topology.Initialize(input));
    EXPECT_TRUE(parts.Initialize(topology,fixture.ledger,{1000,.001}));
    EXPECT_TRUE(binding.Initialize(parts));
  }
};
}

TEST(SelfContactActiveUses, MissingExecutionAndCounterfeitAuthorityRejectThenRetry) {
  {
    Fixture fixture(1);
    Rigid domain_matching(fixture,{{10,11,12,13}});
    c::SelfContactActiveUseBinding uses;
    EXPECT_EQ(uses.Initialize(
        fixture.facets,{&domain_matching.binding,{}}).status,
        Code::IdentityMismatch);
    EXPECT_FALSE(uses.prepared());
    EXPECT_EQ(uses.Initialize(fixture.facets).status,Code::Ok);
  }
  {
    ExecutionFixture fixture(1,ExecutionRigidMode::MergedParts);
    CounterfeitMergedParts counterfeit(fixture);
    ASSERT_EQ(counterfeit.binding.groups().size(),fixture.rigid.groups().size());
    ASSERT_EQ(counterfeit.binding.members().size(),fixture.rigid.members().size());
    ASSERT_NE(counterfeit.binding.groups().data(),fixture.rigid.groups().data());
    c::SelfContactActiveUseBinding uses;
    EXPECT_EQ(uses.Initialize(fixture.facets,{&counterfeit.binding,{}}).status,
        Code::IdentityMismatch);
    EXPECT_FALSE(uses.prepared());
    EXPECT_EQ(uses.Initialize(fixture.facets,{&fixture.rigid,{}}).status,Code::Ok);
  }
}

TEST(SelfContactActiveUses, RetainedAuthoritySurvivesSourceDestructionAndMaySeedAlias) {
  c::SelfContactActiveUseBinding uses;
  std::size_t vertex=SIZE_MAX,facet=SIZE_MAX;
  c::WeightedSurfacePoint face_point;
  {
    auto fixture=std::make_unique<ExecutionFixture>(
        1,ExecutionRigidMode::MergedParts);
    ASSERT_EQ(uses.Initialize(
        fixture->facets,{&fixture->rigid,{}}).status,Code::Ok);
    vertex=fixture->VertexUse(100,uses,10);
    facet=fixture->RemoteFacet(300,uses.vertex_uses()[vertex].feature,uses);
    ASSERT_NE(vertex,SIZE_MAX);
    ASSERT_NE(facet,SIZE_MAX);
    face_point=fixture->FacePoint(facet,uses);
  }
  ASSERT_TRUE(uses.facets()->prepared());
  ASSERT_TRUE(uses.rigid()->prepared());
  auto active=std::vector<std::uint8_t>(uses.parents().size(),1);
  c::SelfContactPairClassification pair;
  ASSERT_EQ(uses.ClassifyVertexFace(vertex,facet,face_point,
      {active.data(),active.data(),active.size()},&pair).status,Code::Ok);
  EXPECT_EQ(pair.status,c::SelfContactPairStatus::ExcludedSameRigidGroup);

  c::SelfContactActiveUseBinding alias_source;
  ASSERT_EQ(alias_source.Initialize(
      *uses.facets(),{uses.rigid(),uses.cin()}).status,Code::Ok);
  EXPECT_EQ(alias_source.rigid()->groups().data(),uses.rigid()->groups().data());
  EXPECT_EQ(alias_source.rigid()->members().data(),uses.rigid()->members().data());
}
} // namespace active_use_test
