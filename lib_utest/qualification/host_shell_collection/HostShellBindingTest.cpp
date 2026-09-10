#include "HostShellFixture.h"
#include "AllocationFailure.h"
#include <memory>
#include <type_traits>

namespace host_shell_test {
using Status=fe::ShellBindingStatus;
static_assert(fe::MaxShellCollectionParents==128&&fe::MaxShellCollectionNodes==128);
static_assert(std::is_nothrow_copy_constructible_v<Geometry> && std::is_nothrow_move_constructible_v<Geometry>);
static_assert(!std::is_copy_assignable_v<Geometry> && !std::is_move_assignable_v<Geometry>);
TEST(HostShellBinding, Complete804Q111T1030NodeInventoryAndNativeMass) {
  Fixture f; Geometry b;
  ASSERT_EQ(b.Initialize(f.geometry(),{}).status,Status::Success);
  EXPECT_EQ(b.qeph_count(),804u); EXPECT_EQ(b.t3_count(),111u); EXPECT_EQ(b.node_count(),1030u);
  ASSERT_EQ(b.inventory().words().size(),24154u);
  EXPECT_EQ(b.active_nodes().size(),1030u); EXPECT_EQ(b.nodes().size(),1030u);
  std::array<shell_binding_test::WideMass,NodeCount> truth{};
  std::array<fe::ShellBindingMass,NodeCount> native{}; fe::ShellBindingMass total;
  auto accumulate=[&](const auto& input,const auto& reference) {
    for(std::size_t local=0;local<input.nodes.size();++local) {
      const fe::ShellBindingMass term{reference.nodal_mass[local],reference.isotropic_inertia[local],
                                    reference.physical_inertia[local],reference.added_inertia[local]};
      shell_binding_test::AddExact(native[input.nodes[local]],term); shell_binding_test::AddExact(total,term);
    }
  };
  for(std::size_t i=0;i<QCount;++i) {
    fe::qeph::ReferenceData r;
    ASSERT_EQ(fe::qeph::InitializeReference(f.q[i].reference,r),fe::qeph::Status::kSuccess);
    EXPECT_EQ(Bytes(b.qeph_reference(i).input),Bytes(r.input));
    EXPECT_EQ(b.qeph_source_id(i),f.q[i].source_parent_id); EXPECT_EQ(b.qeph_nodes(i),f.q[i].nodes);
    accumulate(f.q[i],r);
    const auto& in=f.q[i].reference;
    const long double mass=static_cast<long double>(in.density)*in.thickness/4;
    const long double physical=mass*in.thickness*in.thickness/12,added=mass/12;
    for(auto n:f.q[i].nodes) shell_binding_test::Add(truth[n],{mass,physical+added,physical,added});
  }
  for(std::size_t i=0;i<TCount;++i) {
    fe::t3::ReferenceData r;
    ASSERT_EQ(fe::t3::InitializeReference(f.t[i].reference,r),fe::t3::Status::kSuccess);
    EXPECT_EQ(Bytes(b.t3_reference(i).input),Bytes(r.input));
    EXPECT_EQ(b.t3_source_id(i),f.t[i].source_parent_id); EXPECT_EQ(b.t3_nodes(i),f.t[i].nodes);
    accumulate(f.t[i],r);
    const auto tri=shell_binding_test::independent::Independent(shell_binding_test::NativeInput(f.t[i].reference));
    for(unsigned local=0;local<3;++local) { const auto w=tri.weight[local];
      shell_binding_test::Add(truth[f.t[i].nodes[local]],{w*tri.mass,w*tri.total,w*tri.physical,w*tri.added}); }
  }
  for(std::size_t n=0;n<NodeCount;++n) {
    SCOPED_TRACE(n); const auto& m=b.nodes()[n].native;
    shell_binding_test::Exact(m,native[n]); EXPECT_EQ(b.nodes()[n].source_id,Fixture::NodeId(n));
    shell_binding_test::Near(m.mass,truth[n].mass); shell_binding_test::Near(m.isotropic_inertia,truth[n].total);
    shell_binding_test::Near(m.physical_inertia,truth[n].physical); shell_binding_test::Near(m.added_inertia,truth[n].added);
  }
  shell_binding_test::Exact(b.totals(),total);
  RecordProperty("host_bytes",std::to_string(b.host_bytes()));
  EXPECT_GT(b.host_bytes(),768000u); EXPECT_LT(b.host_bytes(),1024u*1024);
}
TEST(HostShellBinding, DefaultAndByteCapsRejectBeforeBorrowedAccess) {
  Fixture f; Geometry full; ASSERT_EQ(full.Initialize(f.geometry(),{}).status,Status::Success);
  Geometry b; const auto old=Bytes(b); auto input=f.geometry();
  input.qeph=reinterpret_cast<const fe::ShellQephBindingInput*>(1);
  input.t3=reinterpret_cast<const fe::ShellT3BindingInput*>(1);
  EXPECT_EQ(b.Initialize(input).status,Status::InvalidInput); EXPECT_EQ(Bytes(b),old);
  fe::ShellHostBindingLimits limits; limits.max_owned_bytes=full.host_bytes()-1;
  EXPECT_EQ(b.Initialize(input,limits).status,Status::ResourceLimit); EXPECT_EQ(Bytes(b),old);
  limits={}; limits.max_nodes=1029;
  EXPECT_EQ(b.Initialize(input,limits).status,Status::ResourceLimit); EXPECT_EQ(Bytes(b),old);
  limits={}; limits.max_parents=914;
  EXPECT_EQ(b.Initialize(input,limits).status,Status::ResourceLimit); EXPECT_EQ(Bytes(b),old);
  input.qeph_count=std::numeric_limits<std::size_t>::max();
  EXPECT_EQ(b.Initialize(input,{}).status,Status::InvalidInput); EXPECT_EQ(Bytes(b),old);
  limits={}; limits.max_owned_bytes=full.host_bytes();
  ASSERT_EQ(b.Initialize(f.geometry(),limits).status,Status::Success);
  EXPECT_EQ(b.host_bytes(),limits.max_owned_bytes); EXPECT_EQ(b.inventory(),full.inventory());
}
TEST(HostShellBinding, LateReferenceIdentityAndCoverageFailuresPreserveBytesAndRetry) {
  for(unsigned failure=0;failure<4;++failure) {
    Fixture f; Geometry b; const auto old=Bytes(b);
    if(failure==0) f.t.back().reference.young_modulus=-1;
    if(failure==1) f.t.back().source_parent_id=f.q.front().source_parent_id;
    if(failure==2) f.t.back().reference.node_ids[0]+=1;
    if(failure==3) f.t.back().nodes[2]=NodeCount;
    const auto report=b.Initialize(f.geometry(),{});
    const Status expected[]{Status::InvalidT3Reference,Status::InvalidParentIdentity,
                            Status::IdentityMismatch,Status::InvalidConnectivity};
    EXPECT_EQ(report.status,expected[failure]); EXPECT_EQ(report.parent_index,110u);
    EXPECT_EQ(Bytes(b),old); Fixture clean;
    EXPECT_EQ(b.Initialize(clean.geometry(),{}).status,Status::Success);
  }
}
TEST(HostShellBinding, AllAllocationFailurePointsAreAtomicAndRetryable) {
  Fixture f;
  // Q reference array/control, node array/control, inventory array/control.
  for(std::ptrdiff_t after=0;after<6;++after) {
    Geometry b; const auto old=Bytes(b); fe::ShellBindingReport report;
    { AllocationFailure fail(after); report=b.Initialize(f.geometry(),{}); }
    EXPECT_EQ(report.status,Status::ResourceLimit)<<after; EXPECT_EQ(Bytes(b),old);
    EXPECT_EQ(b.Initialize(f.geometry(),{}).status,Status::Success);
  }
}
TEST(HostShellBinding, ImmutableCopyMoveAndInventoryOutliveInputsWithoutAllocating) {
  auto f=std::make_unique<Fixture>(); auto original=std::make_unique<Geometry>();
  ASSERT_EQ(original->Initialize(f->geometry(),{}).status,Status::Success);
  Geometry copy(*original); Geometry moved(std::move(copy));
  fe::ShellBatchInventory identity(original->inventory());
  fe::ShellBatchInventory identity_move(std::move(identity));
  f.reset(); original.reset();
  EXPECT_TRUE(copy.prepared()); EXPECT_EQ(copy.inventory(),moved.inventory());
  EXPECT_EQ(identity,identity_move); EXPECT_EQ(identity,moved.inventory());
  EXPECT_EQ(moved.nodes()[1029].source_id,Fixture::NodeId(1029));
  bool survived=false;
  { AllocationFailure fail;
    Geometry a(copy),b(std::move(a)); fe::ShellBatchInventory inv(std::move(identity));
    survived=a.prepared()&&b.inventory()==inv&&inv==identity;
  }
  EXPECT_TRUE(survived);
  Fixture changed; changed.t.back().reference.position[2].z=-0.; Geometry other;
  ASSERT_EQ(other.Initialize(changed.geometry(),{}).status,Status::Success);
  EXPECT_NE(other.inventory(),moved.inventory()); // Last parent's late coordinate bit participates.
  const auto before=Bytes(moved);
  EXPECT_EQ(moved.Initialize(changed.geometry(),{}).status,Status::AlreadyInitialized);
  EXPECT_EQ(Bytes(moved),before);
}
TEST(HostShellBinding, LegacyPairInitializationAndCopiesAllocateNothing) {
  const auto input=shell_binding_test::Edge(); Geometry legacy;
  fe::ShellBindingReport report; bool copied=false;
  { AllocationFailure fail;
    report=legacy.Initialize(input); Geometry a(legacy),b(std::move(a));
    copied=a.inventory()==b.inventory()&&a.prepared();
  }
  ASSERT_EQ(report.status,Status::Success); EXPECT_TRUE(copied);
  EXPECT_EQ(legacy.inventory().words().size(),49u); EXPECT_EQ(legacy.nodes().size(),128u);
  EXPECT_EQ(legacy.active_nodes().size(),5u); EXPECT_EQ(legacy.host_bytes(),sizeof(Geometry));
  for(std::size_t n=5;n<128;++n) EXPECT_EQ(legacy.nodes()[n].source_id,0u);
  shell_binding_test::CheckNativeReduction(input,legacy);
}
TEST(HostShellBinding, BothPureFamiliesReachIndependentHostParentBound) {
  plasticity_binding_test::Fixture pair;
  for(bool quadrilateral:{false,true}) {
    std::vector<fe::ShellQephBindingInput> q(fe::MaxShellHostParents,pair.qeph);
    std::vector<fe::ShellT3BindingInput> t(fe::MaxShellHostParents,pair.t3);
    for(std::size_t i=0;i<fe::MaxShellHostParents;++i) {
      q[i].source_parent_id=t[i].source_parent_id=10000+i;
      q[i].nodes={0,1,2,3}; t[i].nodes={0,1,2};
    }
    fe::ShellBatchCollectionInput input{quadrilateral?q.data():nullptr,quadrilateral?nullptr:t.data(),
      quadrilateral?q.size():0,quadrilateral?0:t.size(),quadrilateral?4u:3u};
    Geometry b; ASSERT_EQ(b.Initialize(input,{}).status,Status::Success);
    EXPECT_EQ(b.inventory().words().size(),4+(quadrilateral?27:22)*fe::MaxShellHostParents);
    EXPECT_GT(b.nodes()[input.node_count-1].native.mass,0);
    EXPECT_EQ(quadrilateral?b.qeph_source_id(1023):b.t3_source_id(1023),11023u);
    Geometry rejected; const auto old=Bytes(rejected);
    if(quadrilateral) ++input.qeph_count; else ++input.t3_count;
    EXPECT_EQ(rejected.Initialize(input,{}).status,Status::InvalidInput); EXPECT_EQ(Bytes(rejected),old);
  }
}
} // namespace host_shell_test
