#include "ShellBatchCollectionFixture.h"
#include <type_traits>

namespace shell_collection_test {
static_assert(fe::MaxShellCollectionNodes==128&&fe::MaxShellCollectionParents==128);
static_assert(fe::MaxShellBindingNodes==7,"Collection preparation does not raise old resident scratch bounds");
static_assert(!std::is_copy_assignable_v<Binding>);

TEST(ShellBatchCollection,LegacyPairInventoryAndArithmeticRemainExact) {
  const auto old=Edge(); Binding legacy;
  ASSERT_EQ(legacy.Initialize(old).status,Status::Success);
  std::array<std::uint64_t,49> expected{};
  std::size_t cursor=0;
  auto bits=[](double x) { std::uint64_t value; std::memcpy(&value,&x,sizeof(value)); return value; };
  auto append=[&](const auto& in,const auto& nodes,std::uint64_t family) {
    expected[cursor++]=family; expected[cursor++]=nodes.size();
    for(std::size_t n=0;n<nodes.size();++n) {
      expected[cursor++]=nodes[n]; expected[cursor++]=in.node_ids[n];
      expected[cursor++]=bits(in.position[n].x); expected[cursor++]=bits(in.position[n].y);
      expected[cursor++]=bits(in.position[n].z);
    }
    expected[cursor++]=bits(in.density); expected[cursor++]=bits(in.thickness);
    expected[cursor++]=bits(in.young_modulus); expected[cursor++]=bits(in.poisson_ratio);
  };
  expected[cursor++]=1; expected[cursor++]=5;
  append(old.qeph,old.qeph_nodes,4); append(old.t3,old.t3_nodes,3);
  ASSERT_EQ(cursor,expected.size()); ASSERT_EQ(legacy.inventory().words().size(),expected.size());
  EXPECT_TRUE(std::equal(expected.begin(),expected.end(),legacy.inventory().words().begin()));
  EXPECT_EQ(legacy.qeph_count(),1); EXPECT_EQ(legacy.t3_count(),1);
  EXPECT_EQ(legacy.qeph_source_id(0),0); EXPECT_EQ(legacy.t3_source_id(0),0);
  ASSERT_NO_FATAL_FAILURE(CheckNativeReduction(old,legacy));
  const auto collection=Pair(); Binding current;
  ASSERT_EQ(current.Initialize(collection.input()).status,Status::Success);
  ASSERT_EQ(current.inventory().words().size(),53);
  EXPECT_EQ(current.inventory().words()[0],2);
  EXPECT_NE(current.inventory(),legacy.inventory());
  for(unsigned n=0;n<5;++n) Exact(current.nodes()[n].native,legacy.nodes()[n].native);
  Exact(current.totals(),legacy.totals());
}

TEST(ShellBatchCollection,Connected117Node94ParentUnionRetainsNativeAndAnalyticMass) {
  const auto f=Connected(); Binding b;
  ASSERT_EQ(b.Initialize(f.input()).status,Status::Success);
  EXPECT_EQ(b.qeph_count(),88); EXPECT_EQ(b.t3_count(),6); EXPECT_EQ(b.node_count(),117);
  ASSERT_NO_FATAL_FAILURE(CheckNative(f,b));
  ASSERT_NO_FATAL_FAILURE(CheckAnalytic(f,b));
  for(std::size_t n=0;n<NodeCount;++n) {
    EXPECT_EQ(b.nodes()[n].source_id,NodeId(n)); EXPECT_GT(b.nodes()[n].native.mass,0);
  }
  EXPECT_GT(b.nodes()[116].source_id,std::uint64_t{1}<<53);
  EXPECT_NE(b.nodes()[64].native.mass,b.nodes()[116].native.mass);
  EXPECT_NE(b.t3_reference(5).nodal_mass[0],b.t3_reference(5).nodal_mass[1]);
  EXPECT_EQ(b.inventory().words().size(),4+27*88+22*6);
  EXPECT_FALSE(b.qeph_reference().prepared); EXPECT_FALSE(b.t3_reference().prepared);
  for(std::size_t n=NodeCount;n<fe::MaxShellCollectionNodes;++n) {
    EXPECT_EQ(b.nodes()[n].native.mass,0); EXPECT_EQ(b.nodes()[n].source_id,0);
  }
}

TEST(ShellBatchCollection,MaximumNodeAndParentCountsAreAdmittedWithoutImplicitAttachment) {
  auto f=Connected();
  // Eleven more edge-connected triangles cover every declared node through127.
  for(unsigned i=0;i<11;++i) {
    const unsigned parent=TCount+i,n=NodeCount+i;
    f.t[parent]=f.t[0]; f.t[parent].nodes={0,1,n};
    f.t[parent].source_parent_id=ParentBase+100+i;
    for(unsigned local=0;local<2;++local) {
      f.t[parent].reference.position[local]=f.q[0].reference.position[local];
      f.t[parent].reference.node_ids[local]=NodeId(local);
    }
    f.t[parent].reference.position[2]={.25,-1-i/8.,0};
    f.t[parent].reference.node_ids[2]=NodeId(n);
  }
  f.t_count+=11; f.node_count=128; Binding full;
  ASSERT_EQ(full.Initialize(f.input()).status,Status::Success);
  EXPECT_EQ(full.nodes()[127].source_id,NodeId(127)); EXPECT_GT(full.nodes()[127].native.mass,0);
  ASSERT_NO_FATAL_FAILURE(CheckNative(f,full));
  ASSERT_NO_FATAL_FAILURE(CheckAnalytic(f,full));
  // Repeated geometry with distinct source-parent IDs is allowed. This checks
  // collection capacity, not an inferred geometric-overlap/attachment policy.
  for(unsigned family=0;family<2;++family) {
    SCOPED_TRACE(family);
    auto many=Pair();
    if(family==0) {
      many.node_count=4; many.t_count=0; many.q_count=128;
      for(unsigned i=1;i<128;++i) { many.q[i]=many.q[0]; many.q[i].source_parent_id=ParentBase+i; }
    } else {
      many.node_count=3; many.q_count=0; many.t_count=128; many.t[0].nodes={0,1,2};
      for(unsigned i=1;i<128;++i) { many.t[i]=many.t[0]; many.t[i].source_parent_id=ParentBase+i+1; }
    }
    Binding b;
    ASSERT_EQ(b.Initialize(many.input()).status,Status::Success);
    EXPECT_EQ(b.qeph_count()+b.t3_count(),128);
    ASSERT_NO_FATAL_FAILURE(CheckNative(many,b));
    ASSERT_NO_FATAL_FAILURE(CheckAnalytic(many,b));
  }
}

TEST(ShellBatchCollection,OrderedFullInventoryIncludesLateParentsMaterialsAndHighIds) {
  const auto f=Connected(); Binding baseline;
  ASSERT_EQ(baseline.Initialize(f.input()).status,Status::Success);
  for(unsigned kind=0;kind<9;++kind) {
    SCOPED_TRACE(kind);
    auto changed=f;
    if(kind==0) changed.q[87].source_parent_id+=std::uint64_t{1}<<32;
    if(kind==1) changed.t[5].source_parent_id+=std::uint64_t{1}<<32;
    if(kind==2) changed.q[87].reference.young_modulus*=2;
    if(kind==3) changed.t[5].reference.poisson_ratio+=1./64;
    if(kind==4) std::swap(changed.q[0],changed.q[87]);
    if(kind==5) std::swap(changed.t[0],changed.t[5]);
    if(kind==6) {
      const auto saved=changed.q[87];
      for(unsigned n=0;n<4;++n) {
        changed.q[87].nodes[n]=saved.nodes[(n+1)%4];
        changed.q[87].reference.position[n]=saved.reference.position[(n+1)%4];
        changed.q[87].reference.node_ids[n]=saved.reference.node_ids[(n+1)%4];
      }
    }
    if(kind==7) changed.t[5].reference.node_ids[2]+=std::uint64_t{1}<<32;
    if(kind==8) changed.t[5].reference.position[2].z=-0.; // Unique last node116.
    Binding b;
    ASSERT_EQ(b.Initialize(changed.input()).status,Status::Success);
    EXPECT_NE(b.inventory(),baseline.inventory());
    ASSERT_NO_FATAL_FAILURE(CheckNative(changed,b));
  }
  auto shorter=f; shorter.t_count=5; shorter.node_count=115;
  // The omitted final triangle had unique nodes115/116; its tip114 stays covered.
  Binding b;
  ASSERT_EQ(b.Initialize(shorter.input()).status,Status::Success);
  EXPECT_NE(b.inventory().words().size(),baseline.inventory().words().size());
  EXPECT_NE(b.inventory(),baseline.inventory());
}

TEST(ShellBatchCollection,InvalidRangesAndParentIdentitiesRejectBeforeBorrowedAccess) {
  const auto f=Connected();
  for(unsigned kind=0;kind<9;++kind) {
    SCOPED_TRACE(kind);
    auto in=f.input(); Binding b; const auto before=Bytes(b);
    if(kind==0) in.qeph_count=std::numeric_limits<std::size_t>::max();
    if(kind==1) in.t3_count=std::numeric_limits<std::size_t>::max();
    if(kind==2) { in.qeph_count=128; in.t3_count=1; }
    if(kind==3) in.qeph=nullptr;
    if(kind==4) in.t3=nullptr;
    if(kind==5) in.qeph_count=0;
    if(kind==6) in={};
    if(kind==7) in.node_count=129;
    if(kind==8) in.node_count=2;
    EXPECT_EQ(b.Initialize(in).status,Status::InvalidInput); EXPECT_EQ(Bytes(b),before);
    ASSERT_EQ(b.Initialize(f.input()).status,Status::Success);
  }
  for(unsigned kind=0;kind<5;++kind) {
    SCOPED_TRACE(kind);
    auto bad=f; auto family=fe::ShellBindingFamily::T3; std::size_t parent=5;
    if(kind==0) bad.t[5].source_parent_id=0;
    if(kind==1) bad.t[5].source_parent_id=bad.t[0].source_parent_id;
    if(kind==2) bad.t[5].source_parent_id=bad.q[87].source_parent_id;
    if(kind==3) { bad.q[87].source_parent_id=0; family=fe::ShellBindingFamily::Qeph; parent=87; }
    if(kind==4) { bad.q[87].source_parent_id=bad.q[0].source_parent_id; family=fe::ShellBindingFamily::Qeph; parent=87; }
    ASSERT_NO_FATAL_FAILURE(RejectRetry(bad,Status::InvalidParentIdentity,family,parent));
  }
}

TEST(ShellBatchCollection,LateConnectivityIdentityAndCoverageFailuresPreserveRetry) {
  for(unsigned kind=0;kind<9;++kind) {
    SCOPED_TRACE(kind);
    auto bad=Connected(); auto status=Status::IdentityMismatch;
    auto family=fe::ShellBindingFamily::T3; std::size_t parent=5;
    if(kind==0) { bad.t[5].nodes[2]=117; status=Status::InvalidConnectivity; }
    if(kind==1) { bad.t[5].nodes[2]=bad.t[5].nodes[1]; status=Status::InvalidConnectivity; }
    if(kind==2) bad.t[5].reference.node_ids[0]+=std::uint64_t{1}<<32; // Shared tip114.
    if(kind==3) bad.t[5].reference.node_ids[2]=NodeId(64); // Unique116 aliases high global64.
    if(kind==4) { bad.t[5].reference.position[0].z=-0.; status=Status::PositionMismatch; }
    if(kind==5) { bad.q[87].reference.node_ids[0]+=400; family=fe::ShellBindingFamily::Qeph; parent=87; }
    if(kind==6) { bad.q[87].nodes[0]=128; status=Status::InvalidConnectivity; family=fe::ShellBindingFamily::Qeph; parent=87; }
    if(kind==7) { bad.node_count=118; status=Status::InvalidConnectivity; family=fe::ShellBindingFamily::None; parent=fe::NoShellBindingNode; }
    if(kind==8) { bad.t[5].reference.position[0].x=std::nextafter(bad.t[5].reference.position[0].x,100.); status=Status::PositionMismatch; }
    ASSERT_NO_FATAL_FAILURE(RejectRetry(bad,status,family,parent));
  }
}

TEST(ShellBatchCollection,LateNativeRejectionAndFiniteUnionOverflowNeverPublish) {
  for(unsigned kind=0;kind<4;++kind) {
    SCOPED_TRACE(kind);
    auto bad=Connected(); auto status=Status::InvalidT3Reference;
    auto family=fe::ShellBindingFamily::T3; std::size_t parent=5;
    if(kind==0) bad.t[5].reference.thickness=0;
    if(kind==1) bad.t[5].reference.position[2]=bad.t[5].reference.position[1];
    if(kind==2) { bad.q[87].reference.density=0; status=Status::InvalidQephReference; family=fe::ShellBindingFamily::Qeph; parent=87; }
    if(kind==3) { bad.q[87].reference.young_modulus=std::numeric_limits<double>::infinity(); status=Status::InvalidQephReference; family=fe::ShellBindingFamily::Qeph; parent=87; }
    ASSERT_NO_FATAL_FAILURE(RejectRetry(bad,status,family,parent));
  }
  auto bad=Pair(); bad.q_count=4;
  bad.q[0].reference.density=bad.t[0].reference.density=.24*std::numeric_limits<double>::max();
  bad.q[0].reference.thickness=bad.t[0].reference.thickness=1;
  for(unsigned i=1;i<4;++i) { bad.q[i]=bad.q[0]; bad.q[i].source_parent_id=ParentBase+2+i; }
  fe::qeph::ReferenceData q; fe::t3::ReferenceData t;
  ASSERT_EQ(fe::qeph::InitializeReference(bad.q[0].reference,q),fe::qeph::Status::kSuccess);
  ASSERT_EQ(fe::t3::InitializeReference(bad.t[0].reference,t),fe::t3::Status::kSuccess);
  ASSERT_LT(16*static_cast<long double>(q.nodal_mass[0]),std::numeric_limits<double>::max());
  ASSERT_GT(16*static_cast<long double>(q.nodal_mass[0])+t.element_mass,std::numeric_limits<double>::max());
  ASSERT_NO_FATAL_FAILURE(RejectRetry(bad,Status::NonfiniteMass,fe::ShellBindingFamily::T3,0));
}

TEST(ShellBatchCollection,OwnedImmutableValuesAndInvalidAccessorsDoNotSelectFirstParent) {
  auto f=Connected(); Binding b;
  EXPECT_FALSE(b.qeph_reference(0).prepared); EXPECT_FALSE(b.t3_reference(0).prepared);
  ASSERT_EQ(b.Initialize(f.input()).status,Status::Success);
  const auto before=Bytes(b); Binding copy(b);
  f.q[87].reference.position[0].z=8; f.t[5].source_parent_id=0;
  EXPECT_EQ(Bytes(b),before); EXPECT_EQ(copy.inventory(),b.inventory());
  EXPECT_EQ(b.Initialize(f.input()).status,Status::AlreadyInitialized);
  EXPECT_EQ(b.Initialize(Edge()).status,Status::AlreadyInitialized); EXPECT_EQ(Bytes(b),before);
  for(auto i:{std::size_t{128},fe::NoShellBindingNode}) {
    EXPECT_FALSE(b.qeph_reference(i).prepared); EXPECT_FALSE(b.t3_reference(i).prepared);
    EXPECT_EQ(b.qeph_source_id(i),0); EXPECT_EQ(b.t3_source_id(i),0);
    EXPECT_EQ(b.qeph_nodes(i),(std::array<std::size_t,4>{}));
    EXPECT_EQ(b.t3_nodes(i),(std::array<std::size_t,3>{}));
  }
  EXPECT_FALSE(b.qeph_reference(88).prepared); EXPECT_FALSE(b.t3_reference(6).prepared);
  EXPECT_FALSE(b.qeph_reference().prepared); EXPECT_FALSE(b.t3_reference().prepared);
}
} // namespace shell_collection_test
