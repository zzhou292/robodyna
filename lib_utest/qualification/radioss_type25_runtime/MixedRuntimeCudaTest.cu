// SPDX-License-Identifier: AGPL-3.0-or-later
#include "MixedRuntimeFixture.h"
#include "FullLedgerRig.h"
#include "RuntimeObservation.h"
#include "../radioss_type25_current_normals/NativeOracle.h"
#include "lib_src/collision/radioss_type25/normal_activation/Values.h"
#include <cstring>
namespace type25_source_test {
namespace q=n::runtime_qualification;
namespace c=n::current_normals;
namespace {
void NormalFields(const q::NormalObservation& a,const q::NormalObservation& b) {
  ASSERT_EQ(a.face.size(),b.face.size());ASSERT_EQ(a.references.size(),b.references.size());
  for(std::size_t i=0;i<a.face.size();++i) {
    EXPECT_FLOAT_EQ(a.face[i].x,b.face[i].x);EXPECT_FLOAT_EQ(a.face[i].y,b.face[i].y);EXPECT_FLOAT_EQ(a.face[i].z,b.face[i].z);
  }
  for(std::size_t i=0;i<a.references.size();++i) {
    EXPECT_EQ(a.references[i].boundary,b.references[i].boundary);
    for(unsigned k=0;k<2;++k){const auto x=a.references[i].bisector[k],y=b.references[i].bisector[k];
      EXPECT_FLOAT_EQ(x.x,y.x);EXPECT_FLOAT_EQ(x.y,y.y);EXPECT_FLOAT_EQ(x.z,y.z);}
  }
}
void MatchNative(FullLedgerRig& rig,const MixedRuntimeSource& source,
    const q::NormalObservation& prior,const q::NormalObservation& actual) {
  std::vector<double> x(rig.x.size()),velocity(rig.x.size()),coefficients;
  std::vector<std::uint32_t> free;
  fe::NodalStamp stamp;Check(rig.owner.CopyAccepted({x.data(),velocity.data(),rig.m.size()},&stamp));
  const auto src=source.Source();const auto& t=src.starter;
  for(std::size_t i=0;i<src.selection.main_count;++i) {
    coefficients.push_back(src.selection.mains[i].coefficient);
    if(n::normal_activation::detail::FreeMain(src.selection.mains[i]))free.push_back(std::uint32_t(i+1));
  }
  ASSERT_EQ(actual.active.size(),5u);EXPECT_EQ(actual.active[2],0u);
  c::Input in;in.profile=c::Profile::MixedSurfaceLocal;in.free_roster=n::normal_activation::FreeRosterPolicy::FreshComplete;
  in.topology={t.mains,t.node_count,t.primary_count,t.main_count,t.starter.reference_count,src.selection.normal_to_main};
  in.topology.source_profile=t.profile;in.topology.source_topology=t.topology;
  in.topology.primary_roles=t.primary_roles;in.topology.primary_role_count=t.primary_role_count;
  in.topology.mixed_maps={t.primary_to_partner,t.primary_count};
  in.positions={x.data(),std::uint32_t(rig.m.size()),3,1};in.coordinates=s::Coordinates::Si;in.units=source.Config().units;
  in.main_coefficients=coefficients.data();in.coefficient_count=coefficients.size();
  in.main_active=actual.active.data();in.active_count=actual.active.size();in.node_tag=actual.tags.data();in.tag_count=actual.tags.size();
  in.free_main_ids=free.data();in.free_count=free.size();in.prior_normals=prior.face.data();in.prior_count=prior.face.size();
  const auto expected=type25_current_normals_test::OracleMixed(in);ASSERT_TRUE(expected.finite);
  q::NormalObservation comparison;comparison.face=expected.normals;comparison.references=expected.references;NormalFields(actual,comparison);
}
void Attach(FullLedgerRig& rig,MixedRuntimeSource& source) {
  Check(rig.contact.Initialize(source.Config(),source.Source(),rig.owner,rig.publication,
    rig.fixture.physical,rig.Participants(),rig.Identity()));
  Check(rig.publication.ConfigurePhysicalScratchParticipation(rig.owner,rig.fixture.physical,
    rig.Participants(),rig.Identity(),{{},rig.contact.roster_entry()}));
}
}
TEST(NativeMixedRuntimeCuda, RealFullFamilyOwnerUpdatesFiveMainsAndPublishesActiveContact) {
  FullLedgerRig rig(true);ASSERT_NO_THROW(rig.Initialize(false));MixedRuntimeSource source(rig.fixture);
  ASSERT_NO_THROW(Attach(rig,source));q::NormalObservation prior;
  ASSERT_TRUE(q::Access::ReadAcceptedNormals(rig.contact,&prior));
  std::uint64_t active=0;
  for(unsigned step=0;step<4;++step) {
    FullLedgerAttempt a;ASSERT_NO_THROW(rig.Begin(a));ASSERT_NO_THROW(Check(rig.contact.AssembleAccepted(rig.owner,a.token,a.assembly)));
    q::NormalObservation current;ASSERT_TRUE(q::Access::ReadAttemptNormals(rig.contact,rig.owner,a.token,a.assembly,&current));
    ASSERT_NO_THROW(MatchNative(rig,source,prior,current));active+=rig.contact.last_diagnostics().active_forces;
    ASSERT_NO_THROW(rig.Prepare(a));ASSERT_NO_THROW(rig.Seal(a));ASSERT_NO_THROW(Check(rig.Commit(a)));
    ASSERT_TRUE(q::Access::ReadAcceptedNormals(rig.contact,&prior));NormalFields(prior,current);
    EXPECT_EQ(rig.owner.accepted().epoch,step+1);EXPECT_EQ(rig.contact.accepted().generation,step+1);
  }
  EXPECT_GT(active,0u);
}
TEST(NativeMixedRuntimeCuda, CommonRejectionDiscardsMixedNormalsAndExactlyRetriesForce) {
  FullLedgerRig rig(true);ASSERT_NO_THROW(rig.Initialize(false));MixedRuntimeSource source(rig.fixture);ASSERT_NO_THROW(Attach(rig,source));
  q::NormalObservation original;ASSERT_TRUE(q::Access::ReadAcceptedNormals(rig.contact,&original));
  FullLedgerAttempt a;ASSERT_NO_THROW(rig.Begin(a));ASSERT_NO_THROW(Check(rig.contact.AssembleAccepted(rig.owner,a.token,a.assembly)));
  const auto force=rig.Force(a);q::NormalObservation first;
  ASSERT_TRUE(q::Access::ReadAttemptNormals(rig.contact,rig.owner,a.token,a.assembly,&first));
  ASSERT_NO_THROW(rig.Prepare(a));ASSERT_NO_THROW(rig.Seal(a));EXPECT_NE(rig.Commit(a,false).status,fe::ShellPublicationStatus::Success);
  rig.Discard();q::NormalObservation after;ASSERT_TRUE(q::Access::ReadAcceptedNormals(rig.contact,&after));NormalFields(after,original);
  FullLedgerAttempt retry;ASSERT_NO_THROW(rig.Begin(retry));ASSERT_NO_THROW(Check(rig.contact.AssembleAccepted(rig.owner,retry.token,retry.assembly)));
  const auto repeated=rig.Force(retry);ASSERT_EQ(force.size(),repeated.size());EXPECT_EQ(std::memcmp(force.data(),repeated.data(),force.size()*sizeof(double)),0);
  q::NormalObservation second;ASSERT_TRUE(q::Access::ReadAttemptNormals(rig.contact,rig.owner,retry.token,retry.assembly,&second));NormalFields(second,first);
  ASSERT_NO_THROW(rig.Prepare(retry));ASSERT_NO_THROW(rig.Seal(retry));ASSERT_NO_THROW(Check(rig.Commit(retry)));
}
}
