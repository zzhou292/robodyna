// SPDX-License-Identifier: MIT
#include "Flow.h"
#include "lib_src/elements/qeph/mapped/ActivityLayout.h"
#include <type_traits>
#include "../qeph_mapped_activity/FailureValuesFixture.h"
namespace t3_compact_test {
TEST(T3CompactActivityValues,CompleteFrozenLoopsMatchAllRolesEpochsAndRecordFaults) {
  for(unsigned epoch:{0u,1u,2u})for(unsigned mutation=0;mutation<24;++mutation) {
    SCOPED_TRACE(epoch);
    SCOPED_TRACE(mutation);Fixture f(129,epoch);Mutate(f,mutation);
    std::vector<std::uint8_t> old_flags,new_flags;
    const auto original=Serial(f,&old_flags);
    for(bool reverse:{false,true}) {
      const auto current=Candidate(f,[&](Phase p,auto& flags){return HostPhase(f,p,flags,reverse);},&new_flags);
      Same(current,original);if(current.status==t::BatchStatus::Success)EXPECT_EQ(new_flags,old_flags);
    }
  }
}
TEST(T3CompactActivityValues,EarlierGlobalPhaseOutranksEarlierParentOfLaterPhase) {
  for(unsigned fault=0;fault<4;++fault) {
    Fixture f(129,1);const auto point=f.Find(Law::Law44Nip1,true),plastic=f.Find(Law::LayeredLaw44Nip3);
    if(fault==0){f.points[point].point.reported_thickness_m=-1;f.plastic[plastic].history.point[0].stress[0]=NAN;}
    if(fault==1){f.failure[plastic].active=false;f.forces[0].internal_force[0].x=NAN;}
    if(fault==2){f.forces[128].internal_force[0].x=NAN;f.points[0].point.saved.stress[0]=f.points[0].point.current.history.stress[0]=1000;}
    if(fault==3){f.force_time+=1;f.time+=2;}
    const auto original=Serial(f);const auto current=Candidate(f,[&](Phase p,auto& flags){return HostPhase(f,p,flags,true);});
    ASSERT_NE(original.status,t::BatchStatus::Success);Same(current,original);
  }
}
TEST(T3CompactActivityValues,LayoutCountsPrivatePacketsAndKeepsOneShortFailureAtomic) {
  static_assert(std::is_same_v<fe::mapped_shell::ActivityMemory,fe::qeph::mapped::ActivityMemory>);
  static_assert(std::is_same_v<fe::mapped_shell::ActivityLayout,fe::qeph::mapped::ActivityLayout>);
  static_assert(&fe::mapped_shell::ActivityBytes==&fe::qeph::mapped::ActivityBytes);
  for(std::size_t count:{1u,2u,127u,128u,129u,21301u}) {
    fe::mapped_shell::ActivityLayout layout;
    ASSERT_TRUE(layout.Initialize(17,count,1u<<20));
    EXPECT_EQ(layout.active.offset,layout.first_invalid.offset+sizeof(std::uint32_t));
    EXPECT_EQ(layout.active.count,count);EXPECT_EQ(fe::mapped_shell::ActivityBytes(count),count+4);
    const auto exact=layout.bytes;ASSERT_TRUE(layout.Initialize(17,count,exact));
    EXPECT_FALSE(layout.Initialize(17,count,exact-1));EXPECT_EQ(layout.bytes,exact);
  }
}
TEST(T3CompactActivityValues,MalformedOutcomeCannotProduceSuccessfulActivity) {
  for(auto phase:{Phase::OnePoint,Phase::Mixed,Phase::Failure,Phase::Force,Phase::PointIdentity}) {
    EXPECT_NE(m::ActivityErrorReport(phase,0,3).status,t::BatchStatus::Success);
    EXPECT_NE(m::ActivityErrorReport(phase,m::ActivityKey(3,Error::ForceResult),3).status,t::BatchStatus::Success);
    EXPECT_EQ(m::ActivityErrorReport(phase,m::NoActivityFailure,3).status,t::BatchStatus::Success);
  }
}
}

namespace t3_compact_test {
TEST(T3CompactActivityValues,InactiveConstantAndTab1ParentsAndAbsentPointStorageRemainAdmitted) {
  for(unsigned mode=0;mode<3;++mode) {
    Fixture f(129,1);if(mode<2)MakeInactive(f,mode==1);else RemovePointRole(f);
    std::vector<std::uint8_t> old_flags,new_flags;ASSERT_EQ(Serial(f,&old_flags).status,t::BatchStatus::Success);
    const auto result=Candidate(f,[&](Phase phase,auto& flags){return HostPhase(f,phase,flags,true);},&new_flags);
    ASSERT_EQ(result.status,t::BatchStatus::Success);EXPECT_EQ(new_flags,old_flags);
  }
}
TEST(T3CompactActivityValues,ExistingThirtyFailurePolicyAndEncodingCasesRemainExact) {
  std::array<qeph_activity_test::FailureCase,qeph_activity_test::FailureCases> cases;
  qeph_activity_test::FillFailureCases(cases);
  for(const auto& c:cases) {
    const auto law=c.plastic?Law::LayeredLaw44Nip3:Law::LayeredLaw1Nip3;
    EXPECT_EQ(m::CheckFailureActivity(law,c.policy,c.value,c.section,false,c.time)==Error::None,
        qeph_activity_test::SerialFailure(c));
  }
}
}

namespace t3_compact_test {
TEST(T3CompactActivityValues,TabulatedDomainsContinuationAndOptionalRateRemainExact) {
  for(bool continuation:{false,true})for(bool rate:{false,true})for(unsigned fault=0;fault<3;++fault) {
    Fixture f(7,1);f.Table(continuation,rate);const auto point=f.Find(Law::Law44Nip1);
    if(fault==1)f.points[point].point.saved.plastic_strain=f.points[point].point.current.history.plastic_strain=.6;
    if(fault==2)f.points[point].point.saved.filtered_rate_per_s=f.points[point].point.current.history.filtered_rate_per_s=.25;
    std::vector<std::uint8_t> before,after;const auto expected=Serial(f,&before);
    
    const auto actual=Candidate(f,[&](Phase phase,auto& flags){return HostPhase(f,phase,flags,true);},&after);
    Same(actual,expected);if(actual.status==t::BatchStatus::Success)EXPECT_EQ(after,before);
  }
}
}
