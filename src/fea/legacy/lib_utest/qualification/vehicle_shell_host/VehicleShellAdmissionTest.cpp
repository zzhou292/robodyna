#include "VehicleShellFixture.h"
#include "lib_src/elements/ShellBindingIdentityIndex.h"
#include "lib_src/elements/ShellBatchPlasticityBinding.h"
#include "../host_shell_collection/AllocationFailure.h"

namespace vehicle_shell_test {
namespace detail=fe::shell_binding_detail;
static_assert(fe::MaxShellHostParents==1024&&fe::MaxShellHostNodes==2048);
static_assert(fe::MaxShellCollectionParents==128&&fe::MaxShellCollectionNodes==128);
TEST(VehicleShellHost, IndependentCapsRejectBeforeBorrowedReads) {
  Fixture f;Geometry full;auto limits=fe::ShellHostBindingLimits::Vehicle();
  ASSERT_EQ(full.Initialize(f.input(),limits).status,Status::Success);
  auto in=f.input();in.qeph=reinterpret_cast<const fe::ShellQephBindingInput*>(1);
  in.t3=reinterpret_cast<const fe::ShellT3BindingInput*>(1);
  Geometry b;const auto old=Bytes(b);
  EXPECT_EQ(b.Initialize(in).status,Status::InvalidInput);
  EXPECT_EQ(b.Initialize(in,{}).status,Status::InvalidInput);
  for(unsigned fault=0;fault<7;++fault) {
    auto cap=limits;
    if(fault==0) cap.max_owned_bytes=full.host_bytes()-1;
    if(fault==1) cap.max_startup_scratch_bytes=detail::ScratchBytes(f.q.size(),f.t.size(),f.count,false)-1;
    if(fault==2) cap.max_parents=f.q.size()+f.t.size()-1;
    if(fault==3) cap.max_nodes=f.count-1;
    if(fault==4) cap.max_owned_bytes=fe::MaxVehicleShellBindingOwnedBytes+1;
    if(fault==5) cap.max_startup_scratch_bytes=fe::MaxVehicleShellBindingScratchBytes+1;
    if(fault==6) cap.max_nodes=fe::MaxVehicleShellBindingNodes+1;
    EXPECT_EQ(b.Initialize(in,cap).status,Status::ResourceLimit)<<fault;EXPECT_EQ(Bytes(b),old);
  }
  in.node_count=fe::MaxVehicleShellBindingNodes+1;
  EXPECT_EQ(b.Initialize(in,limits).status,Status::InvalidInput);
  in=f.input();in.qeph_count=std::numeric_limits<std::size_t>::max();
  EXPECT_EQ(b.Initialize(in,limits).status,Status::InvalidInput);EXPECT_EQ(Bytes(b),old);
  limits.max_owned_bytes=full.host_bytes();
  limits.max_startup_scratch_bytes=detail::ScratchBytes(f.q.size(),f.t.size(),f.count,false);
  ASSERT_EQ(b.Initialize(f.input(),limits).status,Status::Success);
  EXPECT_EQ(b.inventory(),full.inventory());EXPECT_EQ(b.host_bytes(),full.host_bytes());
}
TEST(VehicleShellHost, SortedIndexesKeepEarliestSecondOccurrenceAndValidationPhase) {
  Fixture f;Geometry b;const auto old=Bytes(b);auto limits=fe::ShellHostBindingLimits::Vehicle();
  f.q[0].source_parent_id=900;f.q[1].source_parent_id=2;
  f.q[3].source_parent_id=900;f.q[4].source_parent_id=2;
  auto report=b.Initialize(f.input(),limits);
  EXPECT_EQ(report.status,Status::InvalidParentIdentity);EXPECT_EQ(report.parent_index,3u);
  EXPECT_EQ(report.family,fe::ShellBindingFamily::Qeph);EXPECT_EQ(Bytes(b),old);
  f.q[3].nodes[3]=f.count;
  report=b.Initialize(f.input(),limits);
  EXPECT_EQ(report.status,Status::InvalidConnectivity);EXPECT_EQ(report.parent_index,3u);
  EXPECT_EQ(report.local_node,3u);EXPECT_EQ(Bytes(b),old);
  f.q[1].reference.density=-1; // All connectivity/parent IDs precede native startup.
  report=b.Initialize(f.input(),limits);EXPECT_EQ(report.status,Status::InvalidConnectivity);
  Fixture clean;ASSERT_EQ(b.Initialize(clean.input(),limits).status,Status::Success);
  Geometry nodes;
  clean.q[1].reference.node_ids[0]=clean.q[0].reference.node_ids[0];
  clean.q[2].reference.node_ids[0]=clean.q[0].reference.node_ids[1];
  report=nodes.Initialize(clean.input(),limits);
  EXPECT_EQ(report.status,Status::IdentityMismatch);EXPECT_EQ(report.parent_index,1u);
  EXPECT_EQ(report.local_node,0u);EXPECT_EQ(report.global_node,4u);
}
TEST(VehicleShellHost, LateNodeIdentitySignedZeroAndCoverageFailuresAreAtomic) {
  auto limits=fe::ShellHostBindingLimits::Vehicle();
  for(unsigned fault=0;fault<4;++fault) {
    Fixture f;Geometry b;const auto old=Bytes(b);auto in=f.input();
    if(fault==0) f.t.back().reference.node_ids[2]=f.q[0].reference.node_ids[0];
    if(fault==1) f.t.back().reference.position[1].z=0.; // Shared node has exact -0.
    if(fault==2) ++in.node_count;
    if(fault==3) f.t.back().reference.young_modulus=-1;
    const auto report=b.Initialize(in,limits);
    const Status status[]{Status::IdentityMismatch,Status::PositionMismatch,
                          Status::InvalidConnectivity,Status::InvalidT3Reference};
    EXPECT_EQ(report.status,status[fault])<<fault;EXPECT_EQ(Bytes(b),old);
    if(fault!=2) EXPECT_EQ(report.parent_index,f.t.size()-1);
    if(fault==0) EXPECT_EQ(report.global_node,f.count-1);
    if(fault==2) EXPECT_EQ(report.global_node,f.count);
    Fixture clean;EXPECT_EQ(b.Initialize(clean.input(),limits).status,Status::Success);
  }
}
TEST(VehicleShellHost, EveryIndexBackingAllocationFailsAtomicallyAndCopiesAllocateNothing) {
  Fixture f;auto limits=fe::ShellHostBindingLimits::Vehicle();
  // Six dynamic arrays, each with separately charged shared control: parent
  // index, Q refs, nodes, words, seen flags and node-occurrence index.
  for(std::ptrdiff_t after=0;after<12;++after) {
    Geometry b;const auto old=Bytes(b);fe::ShellBindingReport report;
    {host_shell_test::AllocationFailure fail(after);report=b.Initialize(f.input(),limits);}
    EXPECT_EQ(report.status,Status::ResourceLimit)<<after;EXPECT_EQ(Bytes(b),old);
    EXPECT_EQ(b.Initialize(f.input(),limits).status,Status::Success);
  }
  Geometry b;fe::ShellBindingReport report;
  {host_shell_test::AllocationFailure fail(12);report=b.Initialize(f.input(),limits);}
  ASSERT_EQ(report.status,Status::Success);
  bool copied=false;
  {host_shell_test::AllocationFailure fail;
    Geometry copy(b),moved(std::move(copy));fe::ShellBatchInventory identity(b.inventory());
    copied=identity==moved.inventory()&&copy.inventory()==b.inventory();}
  EXPECT_TRUE(copied);
}
TEST(VehicleShellHost, ExistingCatalogRejectsVehicleBeforeItsFixedScratchOrBorrowedReads) {
  Fixture f;Geometry b;auto limits=fe::ShellHostBindingLimits::Vehicle();
  ASSERT_EQ(b.Initialize(f.input(),limits).status,Status::Success);
  fe::ShellBatchPlasticityBinding catalog;const auto old=Bytes(catalog);
  fe::ShellBatchPlasticityBindingInput in{
    reinterpret_cast<const fe::ShellPlasticityCurveInput*>(1),
    reinterpret_cast<const fe::ShellPlasticityMaterialInput*>(1),
    reinterpret_cast<const fe::ShellPlasticitySectionInput*>(1),
    reinterpret_cast<const fe::ShellPlasticityParentInput*>(1),1,1,1,1};
  EXPECT_EQ(catalog.Initialize(b,in,limits).status,fe::ShellPlasticityBindingStatus::ResourceLimit);
  EXPECT_EQ(Bytes(catalog),old);
  EXPECT_EQ(catalog.Initialize(b,in).status,fe::ShellPlasticityBindingStatus::ResourceLimit);
  EXPECT_EQ(Bytes(catalog),old);
}
} // namespace vehicle_shell_test
