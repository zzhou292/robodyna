#include "VehicleShellFixture.h"
#include "lib_src/elements/ShellBindingIdentityIndex.h"
#include <memory>

namespace vehicle_shell_test {
TEST(VehicleShellHostSourceSize, Complete349645Parents359785NodesAndOrderedNativeMass) {
  auto f=std::make_unique<Fixture>(Fixture::SourceQ,Fixture::SourceT,Fixture::SourceNodes);
  Geometry b;ASSERT_EQ(b.Initialize(f->input(),fe::ShellHostBindingLimits::Vehicle()).status,Status::Success);
  EXPECT_EQ(b.node_count(),359785u);EXPECT_EQ(b.qeph_count()+b.t3_count(),349645u);
  ASSERT_EQ(b.active_nodes().size(),f->count);EXPECT_EQ(b.nodes().size(),f->count);
  EXPECT_EQ(b.inventory().words().size(),4+29*Fixture::SourceQ+23*Fixture::SourceT);
  std::vector<fe::ShellBindingMass> expected(f->count);fe::ShellBindingMass total;
  fe::qeph::ReferenceData q;fe::t3::ReferenceData t,tail;
  ASSERT_EQ(fe::qeph::InitializeReference(f->q.front().reference,q),fe::qeph::Status::kSuccess);
  ASSERT_EQ(fe::t3::InitializeReference(f->t.front().reference,t),fe::t3::Status::kSuccess);
  ASSERT_EQ(fe::t3::InitializeReference(f->t.back().reference,tail),fe::t3::Status::kSuccess);
  for(std::size_t i=0;i<f->q.size();++i) {
    const auto& source=f->q[i];
    EXPECT_EQ(b.qeph_nodes(i),source.nodes);EXPECT_EQ(b.qeph_source_id(i),source.source_parent_id);
    EXPECT_EQ(Bytes(b.qeph_reference(i).input),Bytes(source.reference));
    AddExpected(q,source,expected,total);
  }
  for(std::size_t i=0;i<f->t.size();++i) {
    const auto& source=f->t[i];
    EXPECT_EQ(b.t3_nodes(i),source.nodes);EXPECT_EQ(b.t3_source_id(i),source.source_parent_id);
    EXPECT_EQ(Bytes(b.t3_reference(i).input),Bytes(source.reference));
    AddExpected(i+1==f->t.size()?tail:t,source,expected,total);
  }
  for(std::size_t n=0;n<f->count;++n) {
    const auto& node=b.nodes()[n];
    EXPECT_EQ(node.source_id,f->NodeId(n));EXPECT_EQ(Bytes(node.position),Bytes(f->Position(n)));
    shell_binding_test::Exact(node.native,expected[n]);
  }
  shell_binding_test::Exact(b.totals(),total);
  // Independent flat-area mass check, separate from the ordered native ledger.
  const long double mass=Fixture::SourceQ*32.L+Fixture::SourceT*(7890.L/256);
  shell_binding_test::Near(b.totals().mass,mass);
  EXPECT_GT(b.nodes()[f->count-1].source_id,std::uint64_t{1}<<54);
  EXPECT_GT(b.nodes()[f->count-1].native.mass,0);
  RecordProperty("owned_host_bytes",std::to_string(b.host_bytes()));
  RecordProperty("startup_scratch_bytes",std::to_string(fe::shell_binding_detail::ScratchBytes(
      Fixture::SourceQ,Fixture::SourceT,Fixture::SourceNodes,false)));
  Geometry copy(b),moved(std::move(copy));fe::ShellBatchInventory words(b.inventory());
  f.reset();EXPECT_EQ(copy.inventory(),moved.inventory());EXPECT_EQ(words,b.inventory());
  EXPECT_EQ(moved.node_count(),359785u);
}
TEST(VehicleShellHostSourceSize, LastSourceIdentityFailurePreservesObjectAndExactRetry) {
  Fixture f(Fixture::SourceQ,Fixture::SourceT,Fixture::SourceNodes);
  Geometry b;const auto old=Bytes(b);auto limits=fe::ShellHostBindingLimits::Vehicle();
  const auto correct=f.t.back().reference.node_ids[2];
  f.t.back().reference.node_ids[2]=f.q.front().reference.node_ids[0];
  const auto report=b.Initialize(f.input(),limits);
  ASSERT_EQ(report.status,Status::IdentityMismatch);
  EXPECT_EQ(report.family,fe::ShellBindingFamily::T3);EXPECT_EQ(report.parent_index,21300u);
  EXPECT_EQ(report.local_node,2u);EXPECT_EQ(report.global_node,359784u);EXPECT_EQ(Bytes(b),old);
  f.t.back().reference.node_ids[2]=correct;
  ASSERT_EQ(b.Initialize(f.input(),limits).status,Status::Success);
  Geometry clean;ASSERT_EQ(clean.Initialize(f.input(),limits).status,Status::Success);
  EXPECT_EQ(b.inventory(),clean.inventory());shell_binding_test::Exact(b.totals(),clean.totals());
  for(std::size_t n=0;n<f.count;++n) EXPECT_EQ(Bytes(b.nodes()[n]),Bytes(clean.nodes()[n]));
}
} // namespace vehicle_shell_test
