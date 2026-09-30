#include "GroupTestSupport.h"
#include <cstring>
#include <limits>
#include <vector>

namespace rigid_test {
TEST(NodalRigidGroupModel,NativeTotalAttributionKeepsTensorAndRejectsUnprovenSplits) {
  auto members=Members();
  fe::NodalRigidGroupInput group{1001,2001,members.data(),members.size()};
  fe::NodalRigidGroupModel reference; ASSERT_TRUE(reference.InitializePhysical(Input(&group,1)));
  for(auto& m:members) {
    m.unpartitioned_native_inertia_kg_m2=m.physical_inertia_kg_m2;
    m.physical_inertia_kg_m2=0;
  }
  fe::NodalRigidGroupModel model;
  EXPECT_EQ(model.Initialize(Input(&group,1)).status,fe::NodalRigidGroupStatus::InvalidMass);
  EXPECT_EQ(model.InitializePhysical(Input(&group,1)).status,fe::NodalRigidGroupStatus::InvalidMass);
  ASSERT_FALSE(model.prepared());
  ASSERT_TRUE(model.InitializeNativeTotal(Input(&group,1)));
  EXPECT_TRUE(model.physical_coefficients());
  const auto& actual=model.groups()[0]; const auto& old=reference.groups()[0];
  EXPECT_EQ(actual.physical_inertia_sum,0);
  EXPECT_EQ(actual.unpartitioned_native_inertia_sum,old.physical_inertia_sum);
  EXPECT_EQ(actual.native_total_inertia_sum,old.native_total_inertia_sum);
  EXPECT_EQ(actual.added_inertia_sum,old.added_inertia_sum);
  for(unsigned i=0;i<9;++i) EXPECT_EQ(actual.raw_tensor.v[i],old.raw_tensor.v[i]);
  for(unsigned i=0;i<members.size();++i)
    EXPECT_EQ(model.members()[i].total_inertia_kg_m2,reference.members()[i].total_inertia_kg_m2);
}
TEST(NodalRigidGroupModel,NativeTotalMissingNegativeNonfiniteAndDoubleCountedEvidenceRejectAtomically) {
  const auto original=Members();
  for(unsigned failure=0;failure<4;++failure) {
    auto members=original; auto& last=members.back();
    const double native=last.physical_inertia_kg_m2;
    last.physical_inertia_kg_m2=0; last.unpartitioned_native_inertia_kg_m2=native;
    if(failure==0) last.unpartitioned_native_inertia_kg_m2=0;
    if(failure==1) last.unpartitioned_native_inertia_kg_m2=-native;
    if(failure==2) last.unpartitioned_native_inertia_kg_m2=std::numeric_limits<double>::quiet_NaN();
    if(failure==3) last.physical_inertia_kg_m2=native;
    fe::NodalRigidGroupInput group{1001,2001,members.data(),members.size()};
    fe::NodalRigidGroupModel model;
    EXPECT_EQ(model.InitializeNativeTotal(Input(&group,1)).status,fe::NodalRigidGroupStatus::InvalidMass);
    EXPECT_FALSE(model.prepared()); EXPECT_EQ(model.members(),nullptr);
    last.physical_inertia_kg_m2=0; last.unpartitioned_native_inertia_kg_m2=native;
    ASSERT_TRUE(model.InitializeNativeTotal(Input(&group,1)));
  }
}
TEST(NodalRigidGroupModel,NativeMassAndTensorMatchIndependentLongDoubleCalculation) {
  auto members=Members(); fe::NodalRigidGroupInput group{1001,2001,members.data(),members.size()};
  fe::NodalRigidGroupModel model; ASSERT_TRUE(model.Initialize(Input(&group,1)));
  ASSERT_EQ(model.group_count(),1u); ASSERT_EQ(model.member_count(),4u);
  const auto& g=model.groups()[0];
  long double mass=0,moment[3]{},center[3]{},tensor[3][3]{};
  for(const auto& m:members) {
    mass+=m.mass_kg;
    for(unsigned a=0;a<3;++a) moment[a]+=static_cast<long double>(m.mass_kg)*Get(m.position,a);
  }
  for(unsigned a=0;a<3;++a) center[a]=moment[a]/mass;
  for(const auto& m:members) {
    long double r[3],square=0;
    for(unsigned a=0;a<3;++a) { r[a]=Get(m.position,a)-center[a]; square+=r[a]*r[a]; }
    for(unsigned a=0;a<3;++a) for(unsigned b=0;b<3;++b)
      tensor[a][b]+=m.mass_kg*((a==b?square:0)-r[a]*r[b])+(a==b?m.total_inertia_kg_m2:0);
  }
  EXPECT_DOUBLE_EQ(g.structural_mass_kg,17); EXPECT_DOUBLE_EQ(g.native_total_inertia_sum,.014);
  for(unsigned a=0;a<3;++a) {
    EXPECT_NEAR(Get(g.structural_center,a),static_cast<double>(center[a]),2e-17);
    for(unsigned b=0;b<3;++b) EXPECT_NEAR(g.raw_tensor.v[3*a+b],static_cast<double>(tensor[a][b]),2e-16);
  }
  EXPECT_FALSE(g.regularization.principal_inertia_changed);
  EXPECT_TRUE(rigid::detail::Orthonormal(g.principal.axes));
  for(unsigned a=0;a<3;++a) for(unsigned b=0;b<3;++b) {
    double reconstructed=0;
    for(unsigned k=0;k<3;++k) reconstructed+=g.principal.axes.v[3*a+k]*Get(g.principal.inertia,k)*g.principal.axes.v[3*b+k];
    EXPECT_NEAR(reconstructed,g.effective_tensor.v[3*a+b],2e-15);
  }
}
TEST(NodalRigidGroupModel,OwnsExactMemberDataAndRetainsSourceUnitRegularizers) {
  auto members=Members(); members[0].position.x=-0.;
  fe::NodalRigidGroupInput group{1001,2001,members.data(),members.size()};
  fe::NodalRigidGroupModel model; ASSERT_TRUE(model.Initialize(Input(&group,1)));
  const auto expected=members;
  for(auto& m:members) m={};
  for(unsigned i=0;i<4;++i) {
    EXPECT_EQ(model.members()[i].source_node_id,expected[i].source_node_id);
    EXPECT_EQ(std::memcmp(&model.members()[i].position,&expected[i].position,sizeof(Vec3)),0);
  }
  const auto& ledger=model.groups()[0].regularization;
  EXPECT_DOUBLE_EQ(ledger.primary_mass_kg,1e-20*1000);
  EXPECT_DOUBLE_EQ(ledger.primary_isotropic_inertia_kg_m2,(1e-20*1000)*.001*.001);
  EXPECT_DOUBLE_EQ(model.source_units().mass_to_kg,1000);
  EXPECT_GT(model.owned_payload_bytes(),0u); EXPECT_GT(model.startup_payload_bytes(),model.owned_payload_bytes());
  EXPECT_EQ(model.Initialize({}).status,fe::NodalRigidGroupStatus::AlreadyInitialized);
  EXPECT_EQ(model.members()[3].source_node_id,expected[3].source_node_id);
}
TEST(NodalRigidGroupModel,MeasurablePrimaryChangesMassCenterAndFullTensor) {
  auto members=Members();
  for(auto& member:members) {
    member.mass_kg*=1e-20;
    member.total_inertia_kg_m2*=1e-20;
    member.physical_inertia_kg_m2*=1e-20;
    member.added_inertia_kg_m2*=1e-20;
  }
  fe::NodalRigidGroupInput group{1001,2001,members.data(),members.size()};
  auto input=Input(&group,1); input.source_units={1,1};
  fe::NodalRigidGroupModel model; ASSERT_TRUE(model.Initialize(input));
  const auto& actual=model.groups()[0];

  // Independent five-particle calculation. The primary's position is the
  // geometric centroid, distinct from the unequal-mass structural COM.
  const long double primary_mass=static_cast<long double>(1e-20);
  const long double primary_j=primary_mass;
  long double geometric[3]{},moment[3]{},center[3]{},tensor[3][3]{};
  long double structural_mass=0;
  for(const auto& member:members) {
    structural_mass+=member.mass_kg;
    for(unsigned a=0;a<3;++a) {
      geometric[a]+=Get(member.position,a)/static_cast<long double>(members.size());
      moment[a]+=static_cast<long double>(member.mass_kg)*Get(member.position,a);
    }
  }
  const long double total_mass=structural_mass+primary_mass;
  for(unsigned a=0;a<3;++a) center[a]=(moment[a]+primary_mass*geometric[a])/total_mass;
  for(unsigned i=0;i<=members.size();++i) {
    const bool primary=i==members.size();
    const long double mass=primary?primary_mass:members[i].mass_kg;
    const long double inertia=primary?primary_j:members[i].total_inertia_kg_m2;
    long double r[3]{},square=0;
    for(unsigned a=0;a<3;++a) {
      r[a]=(primary?geometric[a]:Get(members[i].position,a))-center[a]; square+=r[a]*r[a];
    }
    for(unsigned a=0;a<3;++a) for(unsigned b=0;b<3;++b)
      tensor[a][b]+=mass*((a==b?square:0)-r[a]*r[b])+(a==b?inertia:0);
  }
  EXPECT_NEAR(actual.total_mass_kg,static_cast<double>(total_mass),1e-34);
  EXPECT_GT(std::fabs(actual.center.y-actual.structural_center.y),1e-4);
  for(unsigned a=0;a<3;++a) {
    EXPECT_NEAR(Get(actual.generated_primary_position,a),static_cast<double>(geometric[a]),1e-17);
    EXPECT_NEAR(Get(actual.center,a),static_cast<double>(center[a]),2e-17);
    for(unsigned b=0;b<3;++b)
      EXPECT_NEAR(actual.raw_tensor.v[3*a+b],static_cast<double>(tensor[a][b]),1e-34);
  }
}
TEST(NodalRigidGroupModel,NativeTotalJIsNeverReplacedByPartitionSum) {
  auto members=Members();
  members[3].total_inertia_kg_m2=std::nextafter(members[3].total_inertia_kg_m2,1.);
  fe::NodalRigidGroupInput group{1001,2001,members.data(),members.size()};
  fe::NodalRigidGroupModel model; ASSERT_TRUE(model.Initialize(Input(&group,1)));
  EXPECT_DOUBLE_EQ(model.members()[3].total_inertia_kg_m2,members[3].total_inertia_kg_m2);
  EXPECT_NE(model.members()[3].total_inertia_kg_m2,
    model.members()[3].physical_inertia_kg_m2+model.members()[3].added_inertia_kg_m2);
}
TEST(NodalRigidGroupModel,ThinGroupHasExplicitSourcePrincipalInertiaCorrection) {
  auto members=Members();
  for(unsigned i=0;i<4;++i) {
    members[i].position={double(i)-1.5,0,0}; members[i].mass_kg=1;
    members[i].total_inertia_kg_m2=1e-8; members[i].physical_inertia_kg_m2=6e-9; members[i].added_inertia_kg_m2=4e-9;
  }
  fe::NodalRigidGroupInput group{1001,2001,members.data(),members.size()};
  fe::NodalRigidGroupModel model; ASSERT_TRUE(model.Initialize(Input(&group,1)));
  const auto& g=model.groups()[0]; ASSERT_TRUE(g.regularization.principal_inertia_changed);
  EXPECT_NEAR(g.raw_tensor.v[0],4e-8,1e-20);
  EXPECT_NEAR(g.effective_tensor.v[0]-g.raw_tensor.v[0],.1*(5+4e-8),1e-14);
  for(unsigned i=0;i<9;++i) EXPECT_NEAR(g.effective_tensor.v[i]-g.raw_tensor.v[i],g.regularization.tensor_added.v[i],1e-15);
}
TEST(NodalRigidGroupModel,LateMemberFailuresAreAtomicAndRetryable) {
  const auto valid=Members();
  for(unsigned failure=0;failure<8;++failure) {
    auto members=valid;
    switch(failure) {
      case 0: members[3].mass_kg=0; break;
      case 1: members[3].total_inertia_kg_m2=0; break;
      case 2: members[3].added_inertia_kg_m2=.1; break;
      case 3: members[3].position.z=std::numeric_limits<double>::quiet_NaN(); break;
      case 4: members[3].global_node=128; break;
      case 5: members[3].source_node_id=members[0].source_node_id; break;
      case 6: members[3].global_node=members[0].global_node; break;
      case 7: members[3].position.x=std::numeric_limits<double>::max(); break;
    }
    fe::NodalRigidGroupInput group{1001,2001,members.data(),members.size()};
    fe::NodalRigidGroupModel model; EXPECT_FALSE(model.Initialize(Input(&group,1)));
    EXPECT_FALSE(model.prepared()); EXPECT_EQ(model.members(),nullptr); EXPECT_EQ(model.groups(),nullptr);
    EXPECT_EQ(model.owned_payload_bytes(),0u); members=valid;
    ASSERT_TRUE(model.Initialize(Input(&group,1))); EXPECT_EQ(model.member_count(),4u);
  }
}
TEST(NodalRigidGroupModel,RejectsCrossGroupOverlapAndRepeatedGroupIdentity) {
  for(unsigned failure=0;failure<4;++failure) {
    auto a=Members(),b=Members(201,64);
    fe::NodalRigidGroupInput groups[]{{1001,2001,a.data(),4},{1002,2002,b.data(),4}};
    if(failure==0) b[3].global_node=a[0].global_node;
    if(failure==1) b[3].source_node_id=a[0].source_node_id;
    if(failure==2) groups[1].source_group_id=1001;
    if(failure==3) groups[1].source_node_set_id=2001;
    fe::NodalRigidGroupModel model; EXPECT_FALSE(model.Initialize(Input(groups,2))); EXPECT_FALSE(model.prepared());
  }
}
TEST(NodalRigidGroupModel,ActiveStorageSupportsMoreThan128MembersAndLargeGlobalIndices) {
  std::vector<fe::NodalRigidGroupMember> members(257);
  for(std::size_t i=0;i<members.size();++i)
    members[i]={10001+i,100000+i,{double(i)*.001,double(i%3)*.01,double(i%5)*.02},1,.003,.001,.002};
  fe::NodalRigidGroupInput group{1001,2001,members.data(),members.size()};
  auto input=Input(&group,1,100257); input.limits.max_members_per_group=257;
  fe::NodalRigidGroupModel model; ASSERT_TRUE(model.Initialize(input));
  EXPECT_EQ(model.members()[256].global_node,100256u); EXPECT_DOUBLE_EQ(model.groups()[0].structural_mass_kg,257);
  EXPECT_LT(model.startup_payload_bytes(),100000u);
}
TEST(NodalRigidGroupModel,ExplicitCapsAndSourceUnitsRejectBeforePublication) {
  auto members=Members(); fe::NodalRigidGroupInput group{1001,2001,members.data(),4};
  for(unsigned failure=0;failure<8;++failure) {
    auto input=Input(&group,1);
    if(failure==0) input.limits.max_members=3;
    if(failure==1) input.limits.max_members_per_group=3;
    if(failure==2) input.limits.max_host_bytes=1;
    if(failure==3) input.source_units={};
    if(failure==4) input.source_units={1,std::numeric_limits<double>::max()};
    if(failure==5) input.group_count=SIZE_MAX;
    if(failure==6) { input.limits.max_groups=SIZE_MAX; input.group_count=0; }
    if(failure==7) {
      input.limits.max_groups=input.limits.max_members=input.limits.max_host_bytes=SIZE_MAX;
      input.group_count=SIZE_MAX;
    }
    fe::NodalRigidGroupModel model; EXPECT_FALSE(model.Initialize(input)); EXPECT_FALSE(model.prepared());
  }
}
} // namespace rigid_test
