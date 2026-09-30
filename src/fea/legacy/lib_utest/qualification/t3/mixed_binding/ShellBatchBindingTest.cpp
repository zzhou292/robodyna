#include "ShellBatchBindingFixture.h"
#include <type_traits>
#include <utility>

namespace shell_binding_test {
static_assert(!std::is_copy_assignable_v<Binding>&&!std::is_move_assignable_v<Binding>,"No replacement of a prepared binding");
static_assert(std::is_same_v<decltype(std::declval<Binding&>().qeph_reference()),const fe::qeph::ReferenceData&>);
static_assert(std::is_same_v<decltype(std::declval<Binding&>().t3_reference()),const fe::t3::ReferenceData&>);

TEST(ShellBatchBinding,SharedEdgePreservesNativeMassPartitionsAndWideIds) {
  const auto in=Edge(); Binding b;
  ASSERT_EQ(b.Initialize(in).status,Status::Success);
  ASSERT_NO_FATAL_FAILURE(CheckTruth(in,b));
  ASSERT_NO_FATAL_FAILURE(CheckNativeReduction(in,b));
  EXPECT_EQ(b.qeph_reference().area,1.); EXPECT_EQ(b.t3_reference().area,.375);
  EXPECT_EQ(b.qeph_nodes(),in.qeph_nodes); EXPECT_EQ(b.t3_nodes(),in.t3_nodes);
  EXPECT_EQ(b.nodes()[4].source_id,WideId); EXPECT_GT(b.nodes()[4].source_id,std::uint64_t{1}<<53);
  const auto& t=b.t3_reference();
  EXPECT_GT(std::abs(t.nodal_mass[0]-t.nodal_mass[1]),.1); // Scalene, not equal thirds/quarters.
  EXPECT_NE(t.isotropic_inertia[0],t.physical_inertia[0]);
  EXPECT_EQ(b.nodes()[1].source_id,101); EXPECT_EQ(b.nodes()[2].source_id,102);
  for(unsigned n=5;n<fe::MaxShellBindingNodes;++n) {
    EXPECT_EQ(b.nodes()[n].native.mass,0); EXPECT_EQ(b.nodes()[n].source_id,0);
  }
}

TEST(ShellBatchBinding,DenseUnionsDoNotInferAttachmentOrMergeCoincidentNodes) {
  for(unsigned shared=0;shared<=3;++shared) {
    SCOPED_TRACE(shared);
    auto in=Overlap(shared); Binding b;
    ASSERT_EQ(b.Initialize(in).status,Status::Success);
    EXPECT_EQ(b.node_count(),7-shared);
    ASSERT_NO_FATAL_FAILURE(CheckTruth(in,b));
    ASSERT_NO_FATAL_FAILURE(CheckNativeReduction(in,b));
  }
  auto in=Overlap(0);
  for(unsigned i=0;i<3;++i) in.t3.position[i]=in.qeph.position[i];
  Binding coincident; ASSERT_EQ(coincident.Initialize(in).status,Status::Success);
  EXPECT_EQ(coincident.node_count(),7); EXPECT_NE(coincident.nodes()[0].source_id,coincident.nodes()[4].source_id);
  ASSERT_NO_FATAL_FAILURE(CheckTruth(in,coincident));
  // Renumber the dense global union without changing source/native order.
  const auto edge=Edge(); in=edge;
  constexpr std::size_t permutation[5]{4,0,3,2,1};
  for(auto& n:in.qeph_nodes) n=permutation[n];
  for(auto& n:in.t3_nodes) n=permutation[n];
  Binding original,renumbered;
  ASSERT_EQ(original.Initialize(edge).status,Status::Success);
  ASSERT_EQ(renumbered.Initialize(in).status,Status::Success);
  EXPECT_NE(original.inventory(),renumbered.inventory());
  for(unsigned n=0;n<5;++n) Exact(original.nodes()[n].native,renumbered.nodes()[permutation[n]].native);
  Exact(original.totals(),renumbered.totals());
}

TEST(ShellBatchBinding,DimensionalScalingAndRigidPoseUseActualStartupValues) {
  const auto base=Edge(); Binding original; ASSERT_EQ(original.Initialize(base).status,Status::Success);
  for(unsigned mode=0;mode<4;++mode) {
    SCOPED_TRACE(mode);
    auto in=base;
    if(mode==0) { in.qeph.density*=3; in.t3.density*=3; }
    if(mode==1) { in.qeph.thickness*=2; in.t3.thickness*=2; }
    if(mode==2) {
      for(auto& p:in.qeph.position) { p.x*=2; p.y*=2; p.z*=2; }
      for(auto& p:in.t3.position) { p.x*=2; p.y*=2; p.z*=2; }
    }
    if(mode==3) {
      for(auto& p:in.qeph.position) p={p.z+8,p.x-4,p.y+2};
      for(auto& p:in.t3.position) p={p.z+8,p.x-4,p.y+2};
    }
    Binding b; ASSERT_EQ(b.Initialize(in).status,Status::Success);
    ASSERT_NO_FATAL_FAILURE(CheckTruth(in,b));
    ASSERT_NO_FATAL_FAILURE(CheckNativeReduction(in,b));
    const long double m[4]{3,2,4,1},physical[4]{3,8,4,1},added[4]{3,2,16,1};
    Near(b.totals().mass,original.totals().mass*m[mode]);
    Near(b.totals().physical_inertia,original.totals().physical_inertia*physical[mode]);
    Near(b.totals().added_inertia,original.totals().added_inertia*added[mode]);
    EXPECT_NE(b.inventory(),original.inventory());
  }
}

TEST(ShellBatchBinding,InventoryIncludesEveryOrderedInputRatherThanOnlyMass) {
  const auto in=Edge(); Binding original,identical;
  ASSERT_EQ(original.Initialize(in).status,Status::Success);
  ASSERT_EQ(identical.Initialize(in).status,Status::Success);
  EXPECT_EQ(original.inventory(),identical.inventory());
  const auto& words=original.inventory().words();
  EXPECT_EQ(words.size(),52); EXPECT_EQ(words[0],6); EXPECT_EQ(words[1],5);
  EXPECT_EQ(words[2],4); EXPECT_EQ(words[3],4); EXPECT_EQ(words[30],3); EXPECT_EQ(words[31],3);
  for(unsigned family=0;family<2;++family) for(unsigned field=0;field<4;++field) {
    SCOPED_TRACE(family);
    SCOPED_TRACE(field);
    auto changed=in;
    double* value=family==0?
        (field==0?&changed.qeph.density:field==1?&changed.qeph.thickness:field==2?&changed.qeph.young_modulus:&changed.qeph.poisson_ratio):
        (field==0?&changed.t3.density:field==1?&changed.t3.thickness:field==2?&changed.t3.young_modulus:&changed.t3.poisson_ratio);
    *value*=1.125; Binding b;
    ASSERT_EQ(b.Initialize(changed).status,Status::Success);
    EXPECT_NE(b.inventory(),original.inventory());
    if(field>=2) Exact(b.totals(),original.totals()); // E/nu still belong to the experiment identity.
  }
  for(unsigned n=0;n<5;++n) {
    auto changed=in;
    for(unsigned i=0;i<4;++i) if(changed.qeph_nodes[i]==n) changed.qeph.node_ids[i]+=400;
    for(unsigned i=0;i<3;++i) if(changed.t3_nodes[i]==n) changed.t3.node_ids[i]+=400;
    Binding b; ASSERT_EQ(b.Initialize(changed).status,Status::Success);
    EXPECT_NE(b.inventory(),original.inventory()); Exact(b.totals(),original.totals());
  }
  // Every represented coordinate belongs to the inventory. Change shared
  // occurrences together so this probes identity, not mismatch rejection.
  for(unsigned n=0;n<5;++n) for(unsigned axis=0;axis<3;++axis) {
    SCOPED_TRACE(n);
    SCOPED_TRACE(axis);
    auto changed=in;
    auto shift=[axis](tl::math::Vec3& p) { if(axis==0)p.x+=1./1024; else if(axis==1)p.y+=1./1024; else p.z+=1./1024; };
    for(unsigned i=0;i<4;++i) if(changed.qeph_nodes[i]==n) shift(changed.qeph.position[i]);
    for(unsigned i=0;i<3;++i) if(changed.t3_nodes[i]==n) shift(changed.t3.position[i]);
    Binding b; ASSERT_EQ(b.Initialize(changed).status,Status::Success);
    EXPECT_NE(b.inventory(),original.inventory());
    ASSERT_NO_FATAL_FAILURE(CheckNativeReduction(changed,b));
    // This small identity probe may warp the Q4; the flat-area oracle above
    // is deliberately not applied to these varied reference shapes.
  }
  // Coordinate signs carry identity even when arithmetic/native mass agrees.
  for(unsigned axis=0;axis<3;++axis) {
    auto changed=in;
    auto negative_zero=[axis](tl::math::Vec3& p) { if(axis==0)p.x=-0.; else if(axis==1)p.y=-0.; else p.z=-0.; };
    negative_zero(changed.qeph.position[0]); Binding b;
    ASSERT_EQ(b.Initialize(changed).status,Status::Success);
    EXPECT_NE(b.inventory(),original.inventory()); Exact(b.totals(),original.totals());
  }
  // Native cyclic slot order is not normalized out of participant identity.
  auto cyclic=in;
  for(unsigned i=0;i<4;++i) {
    cyclic.qeph_nodes[i]=in.qeph_nodes[(i+1)%4]; cyclic.qeph.position[i]=in.qeph.position[(i+1)%4];
    cyclic.qeph.node_ids[i]=in.qeph.node_ids[(i+1)%4];
  }
  Binding cycled; ASSERT_EQ(cycled.Initialize(cyclic).status,Status::Success);
  EXPECT_NE(cycled.inventory(),original.inventory());
  ASSERT_NO_FATAL_FAILURE(CheckTruth(cyclic,cycled));
}

TEST(ShellBatchBinding,ConnectivityAndSourceIdentityFailuresPreserveOutputAndRetry) {
  for(unsigned kind=0;kind<11;++kind) {
    SCOPED_TRACE(kind);
    auto bad=Edge(); auto expected=Status::InvalidConnectivity;
    auto family=fe::ShellBindingFamily::None;
    if(kind==0) { bad.node_count=0; expected=Status::InvalidInput; }
    if(kind==1) { bad.node_count=8; expected=Status::InvalidInput; }
    if(kind==2) { bad.qeph_nodes[3]=5; family=fe::ShellBindingFamily::Qeph; }
    if(kind==3) { bad.t3_nodes[2]=5; family=fe::ShellBindingFamily::T3; }
    if(kind==4) { bad.qeph_nodes[3]=2; family=fe::ShellBindingFamily::Qeph; }
    if(kind==5) { bad.t3_nodes[2]=4; family=fe::ShellBindingFamily::T3; }
    if(kind==6) bad.node_count=6; // Dense declaration contains an unrepresented node.
    if(kind==7) { bad.t3.node_ids[0]+=1000; expected=Status::IdentityMismatch; }
    if(kind==8) { bad.t3.node_ids[0]+=(std::uint64_t{1}<<32); expected=Status::IdentityMismatch; }
    if(kind==9) { bad.t3.node_ids[1]=100; expected=Status::IdentityMismatch; }
    if(kind==10) { bad=Overlap(0); bad.t3.node_ids[0]=101; expected=Status::IdentityMismatch; }
    ASSERT_NO_FATAL_FAILURE(Rejected(bad,expected,family));
  }
}

TEST(ShellBatchBinding,CoordinateAndLateStartupFailuresAreHonestlyRejected) {
  for(unsigned kind=0;kind<10;++kind) {
    SCOPED_TRACE(kind);
    auto bad=Edge(); auto expected=Status::InvalidT3Reference;
    if(kind==0) { bad.t3.position[0].z=-0.; expected=Status::PositionMismatch; }
    if(kind==1) { bad.t3.position[2].y=std::nextafter(1.,2.); expected=Status::PositionMismatch; }
    if(kind==2) bad.t3.position[1].x=std::numeric_limits<double>::quiet_NaN();
    if(kind==3) bad.t3.position[1]={1,.5,0}; // Late collinear T3, Q4 remains admissible.
    if(kind==4) bad.t3.density=0;
    if(kind==5) bad.t3.poisson_ratio=.5;
    if(kind==6) { bad.qeph.young_modulus=std::numeric_limits<double>::infinity(); expected=Status::InvalidQephReference; }
    if(kind==7) { bad.qeph.position[2].y=-1; expected=Status::InvalidQephReference; }
    if(kind==8) { bad.t3.node_ids[1]=bad.t3.node_ids[0]; }
    if(kind==9) { bad.qeph.node_ids[3]=bad.qeph.node_ids[0]; expected=Status::InvalidQephReference; }
    ASSERT_NO_FATAL_FAILURE(Rejected(bad,expected));
  }
  auto bad=Edge(); bad.t3.thickness=0; Binding b;
  const auto report=b.Initialize(bad);
  EXPECT_EQ(report.status,Status::InvalidT3Reference); EXPECT_EQ(report.t3_status,fe::t3::Status::kInvalidInput);
  EXPECT_EQ(report.qeph_status,fe::qeph::Status::kSuccess); EXPECT_EQ(report.family,fe::ShellBindingFamily::T3);
}

TEST(ShellBatchBinding,UnrepresentableUnionRejectsAfterTwoValidNativeProducers) {
  auto in=Edge(); in.qeph.density=in.t3.density=.75*std::numeric_limits<double>::max();
  in.qeph.thickness=in.t3.thickness=1;
  fe::qeph::ReferenceData q; fe::t3::ReferenceData t;
  ASSERT_EQ(fe::qeph::InitializeReference(in.qeph,q),fe::qeph::Status::kSuccess);
  ASSERT_EQ(fe::t3::InitializeReference(in.t3,t),fe::t3::Status::kSuccess);
  const long double combined=4*static_cast<long double>(q.nodal_mass[0])+t.element_mass;
  ASSERT_GT(combined,static_cast<long double>(std::numeric_limits<double>::max()));
  ASSERT_NO_FATAL_FAILURE(Rejected(in,Status::NonfiniteMass,fe::ShellBindingFamily::T3));
}

TEST(ShellBatchBinding,PreparedObjectOwnsInputsAndRefusesReplacement) {
  auto in=Edge(); Binding b;
  EXPECT_FALSE(b.prepared()); EXPECT_EQ(b.node_count(),0);
  ASSERT_EQ(b.Initialize(in).status,Status::Success);
  const auto saved=Bytes(b); Binding copy(b);
  EXPECT_EQ(copy.inventory(),b.inventory()); EXPECT_TRUE(copy.prepared());
  in.qeph.position[0]={99,98,97}; in.t3.node_ids[0]=0;
  in.t3.thickness=0; in.qeph_nodes={3,2,1,0};
  EXPECT_EQ(Bytes(b),saved); EXPECT_EQ(b.Initialize(in).status,Status::AlreadyInitialized);
  EXPECT_EQ(Bytes(b),saved); EXPECT_EQ(b.Initialize(Edge()).status,Status::AlreadyInitialized);
  EXPECT_EQ(Bytes(b),saved); EXPECT_EQ(copy.inventory(),b.inventory());
}
} // namespace shell_binding_test
