#include "Fixture.h"
namespace cin_test {
TEST(TiedCinValues, ShuffledDomainPreservesOriginalRowsAndRepeatedTriangleSlot) {
  Fixture f;
  tied::TiedCinAttachmentModel out;
  ASSERT_TRUE(tied::PrepareCinAttachments(f.post,f.domain,f.Input(),&out));
  EXPECT_TRUE(out.domain()->SharesStorage(f.domain));
  EXPECT_TRUE(out.classification()->SharesStorage(f.post));
  ASSERT_EQ(out.rows().count,2u);
  for (std::size_t row = 0; row < 2; ++row) {
    const auto& mapped = out.rows().data[row];
    const auto& source = f.declarations[row];
    EXPECT_EQ(mapped.original_nsv_row,source.original_nsv_row);
    EXPECT_EQ(mapped.ordered_master_rank,source.ordered_master_rank);
    EXPECT_EQ(mapped.master_source.element_id,source.master_source.element_id);
    EXPECT_EQ(mapped.master_source.part_id,source.master_source.part_id);
    EXPECT_EQ(f.domain.nodes()[mapped.secondary_domain_node].source_id,source.secondary_source_id);
    for (std::size_t slot = 0; slot < 4; ++slot)
      EXPECT_EQ(f.domain.nodes()[mapped.master_domain_nodes[slot]].source_id,source.master_source_ids[slot]);
  }
  EXPECT_EQ(out.rows().data[1].master_domain_nodes[2],out.rows().data[1].master_domain_nodes[3]);
  EXPECT_EQ(out.current_geometry(),tied::CinAttachmentObligation::Pending);
  EXPECT_EQ(out.master_activity_and_release(),tied::CinAttachmentObligation::Pending);
}
TEST(TiedCinValues, LateMissingNodePositionAndConflictingMasterPreserveResultAndRetry) {
  Fixture f;
  tied::TiedCinAttachmentModel out;
  ASSERT_TRUE(tied::PrepareCinAttachments(f.post,f.domain,f.Input(),&out));
  const auto* prior = out.rows().data;
  auto nodes = f.nodes;
  nodes.erase(std::remove_if(nodes.begin(),nodes.end(),[](const auto& n) { return n.source_id == 202; }),nodes.end());
  tl::fea::NodalNodeDomain missing;
  ASSERT_TRUE(missing.Initialize({73,nodes.data(),nodes.size()}));
  auto report = tied::PrepareCinAttachments(f.post,missing,f.Input(),&out);
  EXPECT_EQ(report.status,tied::CinAttachmentStatus::MissingDomainNode);
  EXPECT_EQ(report.row,1u);
  auto original = f.declarations;
  f.declarations.back().reference_positions.back().x = std::nextafter(f.declarations.back().reference_positions.back().x,1.);
  EXPECT_EQ(tied::PrepareCinAttachments(f.post,f.domain,f.Input(),&out).status,tied::CinAttachmentStatus::PositionMismatch);
  f.declarations = original;
  f.declarations.back().ordered_master_rank = f.declarations.front().ordered_master_rank;
  EXPECT_EQ(tied::PrepareCinAttachments(f.post,f.domain,f.Input(),&out).status,tied::CinAttachmentStatus::ConflictingMaster);
  f.declarations = original;
  f.declarations.back().master_source.element_id = f.declarations.front().master_source.element_id;
  EXPECT_EQ(tied::PrepareCinAttachments(f.post,f.domain,f.Input(),&out).status,tied::CinAttachmentStatus::ConflictingMaster);
  EXPECT_EQ(out.rows().data,prior);
  f.declarations = original;
  ASSERT_TRUE(tied::PrepareCinAttachments(f.post,f.domain,f.Input(),&out));
}
TEST(TiedCinValues, PenaltyAndWrongTopologyAreNotSilentlyDropped) {
  Fixture f;
  tied::TiedCinAttachmentModel out;
  f.slaves.back().irupt = 1;
  tied::PostKinChkResult penalty;
  ASSERT_TRUE(tied::PostKinChk(f.PostInput(),&penalty));
  EXPECT_EQ(tied::PrepareCinAttachments(penalty,f.domain,f.Input(),&out).status,tied::CinAttachmentStatus::UnsupportedDisposition);
  EXPECT_FALSE(out.prepared());
  f.declarations.back().topology = tied::CinMasterTopology::Quad;
  EXPECT_EQ(tied::PrepareCinAttachments(f.post,f.domain,f.Input(),&out).status,tied::CinAttachmentStatus::InvalidTopology);
  f.declarations.back().topology = tied::CinMasterTopology::TriangleRepeatedThird;
  ASSERT_TRUE(tied::PrepareCinAttachments(f.post,f.domain,f.Input(),&out));
}
TEST(TiedCinValues, ExactCapsPrecedeBorrowedReadsAndDistinctRetainedBackingsAreCharged) {
  Fixture f;
  tied::CinAttachmentForecast bytes;
  ASSERT_TRUE(tied::ForecastCinAttachments(f.post,f.domain,2,0,&bytes));
  tied::CinAttachmentLimits limits;
  limits.max_host_bytes = bytes.startup_payload_bytes-1;
  tied::TiedCinAttachmentModel out;
  const auto* poisoned = reinterpret_cast<const tied::CinAttachmentDeclaration*>(16);
  EXPECT_EQ(tied::PrepareCinAttachments(f.post,f.domain,{poisoned,2},&out,limits).status,tied::CinAttachmentStatus::ResourceLimit);
  ++limits.max_host_bytes;
  ASSERT_TRUE(tied::PrepareCinAttachments(f.post,f.domain,f.Input(),&out,limits));
  const auto own = out.forecast().model_payload_bytes;
  ASSERT_TRUE(tied::ForecastCinAttachments(f.post,f.domain,2,own,&bytes));
  limits.max_host_bytes = bytes.startup_payload_bytes;
  ASSERT_TRUE(tied::PrepareCinAttachments(f.post,f.domain,f.Input(),&out,limits));
  tl::fea::NodalNodeDomain separate;
  ASSERT_TRUE(separate.Initialize({73,f.nodes.data(),f.nodes.size()}));
  EXPECT_EQ(tied::PrepareCinAttachments(f.post,separate,f.Input(),&out,limits).status,tied::CinAttachmentStatus::ResourceLimit);
  limits = {};
  ASSERT_TRUE(tied::PrepareCinAttachments(f.post,separate,f.Input(),&out,limits));
  EXPECT_TRUE(out.domain()->SharesStorage(separate));
}
TEST(TiedCinValues, SignedZeroSourceIdentityIsNotCoordinateTolerance) {
  Fixture f;
  const auto id = f.declarations[0].master_source_ids[0];
  for (auto& node : f.nodes) if (node.source_id == id) node.position.z = -0.0;
  f.declarations[0].reference_positions[1].z = -0.0;
  tl::fea::NodalNodeDomain domain;
  ASSERT_TRUE(domain.Initialize({73,f.nodes.data(),f.nodes.size()}));
  tied::TiedCinAttachmentModel model;
  ASSERT_TRUE(tied::PrepareCinAttachments(f.post,domain,f.Input(),&model));
  const auto* prior = model.rows().data;
  f.declarations[0].reference_positions[1].z = 0.0;
  EXPECT_EQ(tied::PrepareCinAttachments(f.post,domain,f.Input(),&model).status,tied::CinAttachmentStatus::PositionMismatch);
  EXPECT_EQ(model.rows().data,prior);
}
TEST(TiedCinValues, CompleteOriginalSlaveCountMapsTailAndRejectsLateSourceFailure) {
  Fixture f;
  constexpr std::size_t count = 11165;
  const auto seed = f.declarations[0];
  f.nodes.clear();
  f.slaves.clear();
  f.declarations.clear();
  for (std::size_t slot = 0; slot < 4; ++slot)
    f.nodes.push_back({seed.master_source_ids[slot],seed.reference_positions[slot+1]});
  for (std::size_t row = 0; row < count; ++row) {
    auto value = seed;
    value.original_nsv_row = 2*row;
    value.secondary_source_id = 10000+row;
    value.reference_positions[0].z += 1e-8*row;
    f.nodes.push_back({value.secondary_source_id,value.reference_positions[0]});
    f.slaves.push_back({static_cast<std::uint32_t>(value.secondary_source_id),0,{2,7,7,0,0}});
    f.declarations.push_back(value);
  }
  tl::fea::NodalNodeDomain domain;
  ASSERT_TRUE(domain.Initialize({73,f.nodes.data(),f.nodes.size()},tl::fea::NodalDomainLimits::Vehicle()));
  tied::PostKinChkResult post;
  ASSERT_TRUE(tied::PostKinChk(f.PostInput(),&post));
  tied::TiedCinAttachmentModel model;
  ASSERT_TRUE(tied::PrepareCinAttachments(post,domain,f.Input(),&model));
  ASSERT_EQ(model.rows().count,count);
  EXPECT_EQ(domain.nodes()[model.rows().data[count-1].secondary_domain_node].source_id,10000+count-1);
  const auto* prior = model.rows().data;
  f.declarations.back().master_source.part_id = 1234;
  const auto report = tied::PrepareCinAttachments(post,domain,f.Input(),&model);
  EXPECT_EQ(report.status,tied::CinAttachmentStatus::ConflictingMaster);
  EXPECT_EQ(report.row,count-1);
  EXPECT_EQ(model.rows().data,prior);
  f.declarations.back().master_source.part_id = seed.master_source.part_id;
  ASSERT_TRUE(tied::PrepareCinAttachments(post,domain,f.Input(),&model));
  RecordProperty("synthetic_11165_model_payload_bytes",std::to_string(model.forecast().model_payload_bytes));
}
}
