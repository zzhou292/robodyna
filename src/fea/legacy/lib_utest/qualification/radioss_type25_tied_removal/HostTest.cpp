// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
#include <gtest/gtest.h>
#include <cfenv>
#include <type_traits>
namespace type25_tied_removal_test {
namespace {
template<class T>void SameArray(const T* actual,const std::vector<T>& expected) {
  for(std::size_t i=0;i<expected.size();++i) {SCOPED_TRACE(i);EXPECT_EQ(actual[i],expected[i]);}
}
void Same(const t::Snapshot& actual,const NativeResult& expected) {
  ASSERT_EQ(actual.search.removal_count,expected.nodes.size());ASSERT_EQ(actual.history_count,expected.history.size());
  SameArray(actual.search.main_offsets,expected.main_offsets);SameArray(actual.search.removed_nodes,expected.nodes);
  SameArray(actual.search.secondary_offsets,expected.secondary_offsets);SameArray(actual.search.removed_mains,expected.mains);
  for(std::size_t i=0;i<expected.history.size();++i) {
    SCOPED_TRACE(i);
    EXPECT_EQ(std::memcmp(&actual.history[i],&expected.history[i],sizeof(t::History)),0);
  }
  EXPECT_EQ(actual.native_removal_extent,expected.native_extent);
}
}
TEST(Type25TiedRemoval, CompleteNativeOrderAndHistoryMatchForQuadTriangleAndMixed) {
  for(unsigned mode=0;mode<3;++mode) {
    SCOPED_TRACE(mode);
    Fixture f(mode);const auto in=f.Input();Built built(in);
    ASSERT_GT(in.geometric.geometry.removal_count,0u);
    const auto report=built.Run(in);ASSERT_EQ(report.status,t::Status::Ok);
    ASSERT_GT(report.added_removals,0u);EXPECT_TRUE(report.removal_count_complete);
    Same(built.result,Oracle(in));
    EXPECT_EQ(built.result.native_removal_extent,in.native_removal_extent+report.added_removals);
    EXPECT_EQ(built.result.search.native_model_nodes,in.geometric.geometry.native_model_nodes);
    EXPECT_EQ(std::memcmp(built.result.search.primary_extent,in.geometric.geometry.primary_extent,
      in.geometric.geometry.primary_count*sizeof(double)),0);
    EXPECT_EQ(std::memcmp(built.result.search.initial_contact,in.geometric.geometry.initial_contact,
      in.history_count*sizeof(int)),0);
  }
}
TEST(Type25TiedRemoval, PositiveForbiddenMarkerClearsOnlyExactChannelsAndNegativeMarkerSurvives) {
  Fixture f;auto in=f.Input();const auto expected=Oracle(in);
  auto row=std::size_t(0);while(row<f.history.size()&&expected.secondary_offsets[row]==expected.secondary_offsets[row+1])++row;
  ASSERT_LT(row,f.history.size());const int forbidden=int(expected.mains[expected.secondary_offsets[row]]);
  Built built(in);f.history[row].irtlm[0]=forbidden;
  ASSERT_EQ(built.Run(f.Input()).status,t::Status::Ok);
  const t::History zero{};EXPECT_EQ(std::memcmp(&built.result.history[row],&zero,sizeof(zero)),0);
  Same(built.result,Oracle(f.Input()));
  f.history[row].irtlm[0]=-forbidden;
  ASSERT_EQ(built.Run(f.Input()).status,t::Status::Ok);
  EXPECT_EQ(std::memcmp(&built.result.history[row],&f.history[row],sizeof(zero)),0);
  Same(built.result,Oracle(f.Input()));
}
TEST(Type25TiedRemoval, NoNewRelationsPreserveExistingInverseAndRetainedHistoryBits) {
  Fixture f;for(auto& interface:f.interfaces){interface.rows=nullptr;interface.row_count=0;}
  auto in=f.Input();in.geometric.contributors=in.source.contributors;
  ASSERT_GT(in.geometric.geometry.removal_count,0u);
  // Seed a literal forbidden positive history to prove the no-additions path
  // does not execute the otherwise required REMN_I2OP history reset.
  std::size_t retained=0;
  while(in.geometric.geometry.secondary_offsets[retained]==in.geometric.geometry.secondary_offsets[retained+1])++retained;
  f.history[retained].irtlm[0]=int(in.geometric.geometry.removed_mains[in.geometric.geometry.secondary_offsets[retained]]);
  Built built(in);const auto report=built.Run(in);ASSERT_EQ(report.status,t::Status::Ok);
  EXPECT_EQ(report.added_removals,0u);EXPECT_EQ(report.reset_rows,0u);
  Same(built.result,Oracle(in));
  for(std::size_t i=0;i<f.history.size();++i)EXPECT_EQ(std::memcmp(&built.result.history[i],&f.history[i],sizeof(t::History)),0);
}
TEST(Type25TiedRemoval, ActualInterfaceStorageOrderControlsNewReverseDiscovery) {
  Fixture f;auto in=f.Input();Built built(in);
  ASSERT_EQ(built.Run(in).status,t::Status::Ok);Same(built.result,Oracle(in));
  const auto original=Oracle(in).nodes;
  std::reverse(f.rows[0].begin(),f.rows[0].end());
  std::reverse(f.rows[1].begin(),f.rows[1].end());
  ASSERT_EQ(built.Run(f.Input()).status,t::Status::Ok);Same(built.result,Oracle(f.Input()));
  EXPECT_NE(Oracle(f.Input()).nodes,original);
  std::swap(f.interfaces[0],f.interfaces[1]);f.interfaces[0].native_ordinal=2;f.interfaces[1].native_ordinal=5;
  ASSERT_EQ(built.Run(f.Input()).status,t::Status::Ok);Same(built.result,Oracle(f.Input()));
}
TEST(Type25TiedRemoval, CompleteCapacityFailurePreservesOldPublicationAndRetry) {
  Fixture f;const auto in=f.Input();Built built(in);
  const auto first=built.Run(in);ASSERT_EQ(first.status,t::Status::Ok);
  const auto bytes=built.Bytes();const auto view=built.result;
  t::Limits small;small.search.max_removals=first.required_removals-1;
  const auto failed=built.Run(in,small);EXPECT_EQ(failed.status,t::Status::ResourceLimit);
  EXPECT_TRUE(failed.removal_count_complete);EXPECT_EQ(failed.required_removals,first.required_removals);
  EXPECT_EQ(built.Bytes(),bytes);EXPECT_EQ(built.result.history,view.history);
  small={};small.max_relation_visits=1;
  EXPECT_EQ(built.Run(in,small).status,t::Status::ResourceLimit);EXPECT_EQ(built.Bytes(),bytes);
  ASSERT_EQ(built.Run(in).status,t::Status::Ok);Same(built.result,Oracle(in));
}
TEST(Type25TiedRemoval, FinalizationRosterCountsAndInvalidRowsRejectWithoutPartialOutput) {
  Fixture f;const auto good=f.Input();Built built(good);ASSERT_EQ(built.Run(good).status,t::Status::Ok);
  const auto bytes=built.Bytes();
  for(unsigned which=0;which<8;++which) {
    SCOPED_TRACE(which);
    auto in=good;auto interfaces=f.interfaces;auto rows=f.rows[0];
    in.interfaces=interfaces.data();interfaces[0].rows=rows.data();
    if(which==0)in.finalization=t::Finalization::Unspecified;
    if(which==1)in.tied_removal=3;
    if(which==2)++in.source.contributors.cin_links;
    if(which==3)rows[0].local_main=0;
    if(which==4)rows[0].node=UINT32_MAX;
    if(which==5)interfaces[1].native_ordinal=interfaces[0].native_ordinal;
    if(which==6)interfaces[1].source_id=interfaces[0].source_id;
    if(which==7)interfaces[0].level=2;
    EXPECT_NE(built.Run(in).status,t::Status::Ok);EXPECT_EQ(built.Bytes(),bytes);
  }
  auto malformed=good;malformed.interface_count=SIZE_MAX;malformed.interfaces=reinterpret_cast<const t::Interface*>(1);
  EXPECT_NE(t::Preflight(malformed).status,t::Status::Ok);
  ASSERT_EQ(built.Run(good).status,t::Status::Ok);Same(built.result,Oracle(good));
}
TEST(Type25TiedRemoval, CorruptCsrAndAllBorrowedWriteAliasesPreservePublication) {
  Fixture f;const auto good=f.Input();Built built(good);ASSERT_EQ(built.Run(good).status,t::Status::Ok);
  const auto bytes=built.Bytes();auto in=good;
  std::vector<std::uint32_t> inverse(good.geometric.geometry.secondary_offsets,good.geometric.geometry.secondary_offsets+good.history_count+1);
  inverse.back()+=1;in.geometric.geometry.secondary_offsets=inverse.data();
  EXPECT_EQ(built.Run(in).status,t::Status::InvalidInput);EXPECT_EQ(built.Bytes(),bytes);
  in=good;in.history=static_cast<const t::History*>(built.output.data());
  EXPECT_EQ(built.Run(in).status,t::Status::InvalidInput);EXPECT_EQ(built.Bytes(),bytes);
  EXPECT_EQ(t::Build(good,{},built.output,built.scratch,reinterpret_cast<t::Snapshot*>(built.scratch.data())).status,t::Status::InvalidInput);
  EXPECT_EQ(built.Bytes(),bytes);
  in=good;in.interfaces=reinterpret_cast<const t::Interface*>(reinterpret_cast<const unsigned char*>(good.interfaces)+1);
  EXPECT_EQ(built.Run(in).status,t::Status::InvalidInput);EXPECT_EQ(built.Bytes(),bytes);
  ASSERT_EQ(built.Run(good).status,t::Status::Ok);
}
TEST(Type25TiedRemoval, HostArithmeticAndByteCapsRejectThenRetry) {
  Fixture f;const auto in=f.Input();Built built(in);ASSERT_EQ(built.Run(in).status,t::Status::Ok);const auto bytes=built.Bytes();
  t::Limits cap;cap.search.max_output_bytes=built.plan.output_bytes-1;
  EXPECT_EQ(built.Run(in,cap).status,t::Status::ResourceLimit);EXPECT_EQ(built.Bytes(),bytes);
  const auto rounding=std::fegetround();ASSERT_EQ(std::fesetround(FE_UPWARD),0);
  const auto report=built.Run(in);std::fesetround(rounding);
  EXPECT_EQ(report.status,t::Status::UnsupportedArithmetic);EXPECT_EQ(built.Bytes(),bytes);
  ASSERT_EQ(built.Run(in).status,t::Status::Ok);Same(built.result,Oracle(in));
}
}
