#include "Fixture.h"
#include <type_traits>

namespace nodal_domain_test {
static_assert(std::is_nothrow_copy_constructible_v<fe::NodalNodeDomain>);
static_assert(!std::is_copy_assignable_v<fe::NodalNodeDomain>);

TEST(NodalDomain, OrderedIdentityOwnsInputAndCoincidentDistinctNodes) {
  std::vector<fe::NodalDomainNode> nodes{{901,{0,0,0}},{7,{0,0,0}},
      {(std::uint64_t{1}<<62)+1,{1,2,-0.}}};
  auto a=Domain(nodes);
  EXPECT_EQ(a.Find(901),0); EXPECT_EQ(a.Find(7),1); EXPECT_EQ(a.Find(nodes[2].source_id),2);
  EXPECT_EQ(a.Find(0),SIZE_MAX); EXPECT_EQ(a.Find(900),SIZE_MAX);
  auto independent=Domain(nodes);
  EXPECT_TRUE(a.Matches(independent)); EXPECT_FALSE(a.SharesStorage(independent));
  fe::NodalNodeDomain copy(a),moved(std::move(a));
  nodes.clear(); nodes.shrink_to_fit();
  EXPECT_TRUE(a.prepared()); EXPECT_TRUE(a.SharesStorage(copy)); EXPECT_TRUE(a.SharesStorage(moved));
  EXPECT_EQ(copy.nodes()[2].source_id,(std::uint64_t{1}<<62)+1);
  const auto before=Bytes(a);
  EXPECT_EQ(a.Initialize({0,reinterpret_cast<const fe::NodalDomainNode*>(1),SIZE_MAX}).status,S::AlreadyInitialized);
  EXPECT_EQ(Bytes(a),before);
}

TEST(NodalDomain, ExactIdentityIncludesSourceOrderAndSignedZero) {
  std::vector<fe::NodalDomainNode> nodes{{9,{1,2,-0.}},{5,{3,4,5}}};
  auto original=Domain(nodes);
  auto other_source=Domain(nodes,72); EXPECT_FALSE(original.Matches(other_source));
  nodes[0].position.z=0.; auto zero=Domain(nodes); EXPECT_FALSE(original.Matches(zero));
  nodes[0].position.z=-0.; nodes[1].position.x=std::nextafter(3.,4.);
  auto next=Domain(nodes); EXPECT_FALSE(original.Matches(next));
  nodes[1].position.x=3.; std::swap(nodes[0],nodes[1]);
  auto reordered=Domain(nodes); EXPECT_FALSE(original.Matches(reordered));
  fe::NodalNodeDomain empty; EXPECT_FALSE(empty.Matches(empty)); EXPECT_FALSE(empty.SharesStorage(empty));
  EXPECT_EQ(empty.nodes().size(),0); EXPECT_EQ(empty.Find(9),SIZE_MAX);
}

TEST(NodalDomain, OriginalFirstFailureAndLateInvalidRetry) {
  std::vector<fe::NodalDomainNode> nodes{{900,{0,0,0}},{1,{1,0,0}},{900,{2,0,0}},{1,{3,0,0}}};
  fe::NodalNodeDomain value; const auto before=Bytes(value);
  auto report=value.Initialize({71,nodes.data(),nodes.size()});
  EXPECT_EQ(report.status,S::DuplicateIdentity); EXPECT_EQ(report.node,2); EXPECT_EQ(Bytes(value),before);
  nodes[2].source_id=8; nodes[3].source_id=2; nodes[3].position.z=std::numeric_limits<double>::quiet_NaN();
  report=value.Initialize({71,nodes.data(),nodes.size()});
  EXPECT_EQ(report.status,S::InvalidInput); EXPECT_EQ(report.node,3); EXPECT_EQ(Bytes(value),before);
  nodes[3].position.z=0.; nodes[3].source_id=0;
  report=value.Initialize({71,nodes.data(),nodes.size()});
  EXPECT_EQ(report.status,S::InvalidInput); EXPECT_EQ(report.node,3);
  nodes[3].source_id=2;
  ASSERT_EQ(value.Initialize({71,nodes.data(),nodes.size()}).status,S::Success);
  EXPECT_TRUE(value.Matches(Domain(nodes)));
}

TEST(NodalDomain, CompleteBytesAndCountsPrecedeBorrowedReads) {
  const std::vector<fe::NodalDomainNode> nodes{{1,{0,0,0}},{2,{1,2,3}}};
  auto good=Domain(nodes); const auto bytes=good.owned_payload_bytes();
  EXPECT_EQ(bytes,good.startup_payload_bytes());
  EXPECT_GT(bytes,sizeof(fe::NodalNodeDomain)+nodes.size()*sizeof(fe::NodalDomainNode));
  fe::NodalNodeDomain value; const auto before=Bytes(value);
  const auto* poison=reinterpret_cast<const fe::NodalDomainNode*>(1);
  EXPECT_EQ(value.Initialize({71,poison,2},{2,bytes-1}).status,S::ResourceLimit);
  EXPECT_EQ(value.Initialize({71,poison,SIZE_MAX},fe::NodalDomainLimits::Vehicle()).status,S::ResourceLimit);
  EXPECT_EQ(value.Initialize({71,poison,2},{2,bytes}).status,S::InvalidInput);
  const auto address=UINTPTR_MAX-(UINTPTR_MAX%alignof(fe::NodalDomainNode));
  EXPECT_EQ(value.Initialize({71,reinterpret_cast<const fe::NodalDomainNode*>(address),2},{2,bytes}).status,S::InvalidInput);
  EXPECT_EQ(value.Initialize({0,nodes.data(),2},{2,bytes}).status,S::InvalidInput);
  EXPECT_EQ(Bytes(value),before);
  EXPECT_EQ(value.Initialize({71,nodes.data(),2},{2,bytes}).status,S::Success);
  EXPECT_EQ(value.owned_payload_bytes(),bytes);
}
} // namespace nodal_domain_test
