// SPDX-License-Identifier: MIT
#include "DeviceFixture.cuh"
namespace t3_compact_test {
TEST(T3CompactActivityCuda,CompleteFrozenPredicatesMatchAllRolesAndAdversarialRecords) {
  for(unsigned epoch:{0u,1u,2u})for(unsigned mutation=0;mutation<24;++mutation) {
    SCOPED_TRACE(epoch);
    SCOPED_TRACE(mutation);Fixture f(129,epoch);Mutate(f,mutation);
    std::vector<std::uint8_t> old_flags,new_flags;const auto expected=Serial(f,&old_flags);
    DeviceFixture device(f);const auto actual=Candidate(f,[&](Phase phase,auto& flags){return device.Read(phase,flags);},&new_flags);
    Same(actual,expected);if(actual.status==t::BatchStatus::Success)EXPECT_EQ(new_flags,old_flags);
  }
}
TEST(T3CompactActivityCuda,BlockBoundariesAndBothSlabSelectorsRemainFresh) {
  for(std::size_t count:{1u,2u,127u,128u,129u})for(unsigned epoch:{0u,1u}) {
    Fixture f(count,epoch);std::vector<std::uint8_t> expected;ASSERT_EQ(Serial(f,&expected).status,t::BatchStatus::Success);
    DeviceFixture device(f);
    for(unsigned slab:{0u,1u}) {
      device.query.slab=slab;std::vector<std::uint8_t> actual;
      ASSERT_EQ(Candidate(f,[&](Phase phase,auto& flags){return device.Read(phase,flags);},&actual).status,t::BatchStatus::Success);
      EXPECT_EQ(actual,expected);
    }
  }
}
TEST(T3CompactActivityCuda,GlobalPhasePriorityMatchesFrozenFullSequenceAcrossParents) {
  for(unsigned fault=0;fault<4;++fault) {
    Fixture f(129,1);const auto point=f.Find(Law::Law44Nip1,true),plastic=f.Find(Law::LayeredLaw44Nip3);
    if(fault==0){f.points[point].point.reported_thickness_m=-1;f.plastic[plastic].history.point[0].stress[0]=NAN;}
    if(fault==1){f.failure[plastic].active=false;f.forces[0].internal_force[0].x=NAN;}
    if(fault==2){f.forces[128].internal_force[0].x=NAN;f.points[0].point.saved.stress[0]=f.points[0].point.current.history.stress[0]=1000;}
    if(fault==3){f.force_time+=1;f.time+=2;}
    const auto expected=Serial(f);ASSERT_NE(expected.status,t::BatchStatus::Success);
    DeviceFixture device(f);Same(Candidate(f,[&](Phase phase,auto& flags){return device.Read(phase,flags);}),expected);
  }
}
TEST(T3CompactActivityCuda,RepeatedQueryCannotReuseAFormerValidVerdict) {
  Fixture f(129,1);DeviceFixture device(f);std::vector<std::uint8_t> flags;
  ASSERT_EQ(Candidate(f,[&](Phase phase,auto& out){return device.Read(phase,out);},&flags).status,t::BatchStatus::Success);
  const auto row=f.Find(Law::Law44Nip1,true);
  device.point->section[0][row].point.reported_thickness_m=-1;
  const auto bad=Candidate(f,[&](Phase phase,auto& out){return device.Read(phase,out);});
  EXPECT_EQ(bad.status,t::BatchStatus::NonfiniteResult);EXPECT_STREQ(bad.message,"One-point saved/current/failure state is invalid");
  device.point->section[0][row]=f.points[row];
  ASSERT_EQ(Candidate(f,[&](Phase phase,auto& out){return device.Read(phase,out);},&flags).status,t::BatchStatus::Success);
}
}

namespace t3_compact_test {
TEST(T3CompactActivityCuda,InactiveConstantAndTab1ParentsAndNoPointPathMatchFrozenFlags) {
  for(unsigned mode=0;mode<3;++mode) {
    Fixture f(129,1);if(mode<2)MakeInactive(f,mode==1);else RemovePointRole(f);
    std::vector<std::uint8_t> before,after;ASSERT_EQ(Serial(f,&before).status,t::BatchStatus::Success);
    DeviceFixture device(f);ASSERT_EQ(Candidate(f,[&](Phase phase,auto& flags){return device.Read(phase,flags);},&after).status,t::BatchStatus::Success);
    EXPECT_EQ(after,before);
  }
}
}

namespace t3_compact_test {
TEST(T3CompactActivityCuda,TabulatedDomainsContinuationAndOptionalRateRemainExact) {
  for(bool continuation:{false,true})for(bool rate:{false,true})for(unsigned fault=0;fault<3;++fault) {
    Fixture f(7,1);f.Table(continuation,rate);const auto point=f.Find(Law::Law44Nip1);
    if(fault==1)f.points[point].point.saved.plastic_strain=f.points[point].point.current.history.plastic_strain=.6;
    if(fault==2)f.points[point].point.saved.filtered_rate_per_s=f.points[point].point.current.history.filtered_rate_per_s=.25;
    std::vector<std::uint8_t> before,after;const auto expected=Serial(f,&before);
    DeviceFixture device(f);
    const auto actual=Candidate(f,[&](Phase phase,auto& flags){return device.Read(phase,flags);},&after);
    Same(actual,expected);if(actual.status==t::BatchStatus::Success)EXPECT_EQ(after,before);
  }
}
}
