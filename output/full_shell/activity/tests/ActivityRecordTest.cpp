#include "Support.h"
#include <algorithm>

namespace crash::output::full_shell::activity::test {
TEST(ParentActivity, FullCountWordsPreserveSourceOrderAndContextLifetime) {
    const auto c=Sized(349645,359785);std::vector<std::uint8_t> flags(c.parents().size(),1);
    for(auto i:{0u,63u,64u,349644u})flags[i]=0;
    const auto stamp=base::Frame().stamp;
    const auto record=ActivityRecord::Create(c,Input(c,stamp,flags),stamp);
    EXPECT_EQ(record.words().size()*sizeof(std::uint64_t),43712);EXPECT_EQ(record.active_count(),349641);
    EXPECT_GT(c.retained_payload_bytes(),c.parents().size()*sizeof(ParentPoints));
    EXPECT_LT(detail::Preflight(c,{}),64*1024*1024);
    auto copy=record;auto moved=std::move(copy);
    EXPECT_EQ(copy.words().data(),record.words().data());EXPECT_EQ(moved.context().parents().data(),c.parents().data());
    base::Directory dir;const auto declaration=WriteDeclaration(dir.path,"parent-activity.json",c);
    EXPECT_NO_THROW(ReadDeclaration(dir.path,c,declaration));
    const auto file=WriteActivity(dir.path,"frame-2",record);const auto loaded=ReadActivity(dir.path,c,file,stamp);
    EXPECT_EQ(loaded.words(),record.words());EXPECT_TRUE(SameStamp(loaded.stamp(),stamp));
    for(std::size_t i=0;i<flags.size();++i)ASSERT_EQ(loaded.active(i),flags[i]!=0);
    flags.assign(flags.size(),0);EXPECT_EQ(record.active_count(),349641);EXPECT_THROW(record.active(flags.size()),std::runtime_error);
    RecordProperty("parents","349645");RecordProperty("activity_payload_bytes","43712");
    RecordProperty("startup_payload_bytes",std::to_string(detail::Preflight(c,{})));
}
TEST(ParentActivity, CountScopePhaseAndByteCapsRejectBeforePoisonedInput) {
    const auto c=base::MakeContext();const auto s=base::Frame().stamp;const std::vector<std::uint8_t> flags{1,0,1,0};
    const auto accepted=ActivityRecord::Create(c,Input(c,s,flags),s);
    for(unsigned fault=0;fault<9;++fault) {
        auto in=Input(c,s,flags);in.parent_active=reinterpret_cast<const std::uint8_t*>(1);Limits limits;
        if(fault==0)in.parents=SIZE_MAX;if(fault==1)in.parents=3;
        if(fault==2)++in.identity.owner;
        if(fault==3)in.point_layout_sha256[0]=in.point_layout_sha256[0]=='a'?'b':'a';
        if(fault==4)++in.stamp.attempt;if(fault==5)in.stamp.time+=.125;
        if(fault==6)limits.host_bytes=0;if(fault==7)limits.host_bytes=SIZE_MAX;
        if(fault==8)limits.host_bytes=detail::Preflight(c,{})-1;
        EXPECT_THROW(ActivityRecord::Create(c,in,s,limits),std::runtime_error);
        EXPECT_EQ(accepted.active_count(),2);EXPECT_EQ(accepted.words()[0],5);
    }
    auto bad=flags;bad.back()=8;EXPECT_THROW(ActivityRecord::Create(c,Input(c,s,bad),s),std::runtime_error);
    const auto retry=ActivityRecord::Create(c,Input(c,s,flags),s,{detail::Preflight(c,{})});
    EXPECT_EQ(retry.words(),accepted.words());
}
TEST(ParentActivity, InitialAndNewerEndpointAreExplicitWithoutInferringFailure) {
    const auto c=base::MakeContext();std::vector<std::uint8_t> flags{1,1,1,1};const FrameStamp initial;
    const auto start=ActivityRecord::Create(c,Input(c,initial,flags),initial);
    const FrameStamp later{1,0,7,.125,0,.0625,.0625};flags[2]=0;
    const auto next=ActivityRecord::Create(c,Input(c,later,flags),later);
    EXPECT_EQ(start.active_count(),4);EXPECT_EQ(next.active_count(),3);
    // Current point failure/native PLA never supplies this explicit parent flag.
    EXPECT_TRUE(next.active(0));EXPECT_FALSE(next.active(2));
    base::Directory dir;const auto file=WriteActivity(dir.path,"frame-1",next);
    EXPECT_THROW(ReadActivity(dir.path,c,file,initial),std::runtime_error);
    EXPECT_TRUE(SameStamp(ReadActivity(dir.path,c,file,later).stamp(),later));
}
} // namespace crash::output::full_shell::activity::test
