#include "Support.h"
namespace crash::output::physical_run::test {
TEST(PhysicalRunRecords, CompleteAndPrefixRoundTripExactGeometryNativePointsAndActivity) {
    for(unsigned accepted:{0u,3u,4u}) {
        ft::Directory dir;const auto c=Context();const auto config=Config(c);
        const auto index=WriteRun(dir.path,c,config,accepted);
        const auto metadata=WriteDocument(dir.path,"frame-index.json",IndexDocument(config,index),MetadataCap);
        auto restored=ReadIndex(c,config,array_json::Parse(ReadFile(dir.path,metadata,MetadataCap),MetadataCap));
        EXPECT_NO_THROW(ValidateRecords(dir.path,c,config,restored,64u<<20));
        EXPECT_EQ(restored.accepted_intervals,accepted);EXPECT_EQ(restored.horizon_complete,accepted==4);
        const auto& saved=restored.frames.back();const auto frame=records::ReadFrame(dir.path,c,saved.frame,saved.stamp);
        EXPECT_EQ(Bits(frame.position_xyz[1]),Bits(-0.));EXPECT_EQ(frame.plastic_points.size(),8u);
        const auto fields=records::ParentPlasticMaxima(c,frame);EXPECT_FALSE(fields[0].has_value());
        EXPECT_EQ(*fields[3],.008*accepted);
        const auto active=records::activity::ReadActivity(dir.path,c,saved.activity,saved.stamp);
        EXPECT_EQ(active.active(2),accepted<3);
    }
}
TEST(PhysicalRunRecords, AllIntervalChunksPreserveActualPhaseAndOptionalColumnShape) {
    for(bool bound:{false,true}) {
        ft::Directory dir;const auto c=Context();const Profile profile{false,bound};
        IntervalWriter writer(dir.path,c,profile,5,624,4096);
        for(unsigned i=1;i<=5;++i)writer.Append(Row(c,i,3*i,bound));
        const auto segments=writer.Finish();ASSERT_EQ(segments.size(),3u);
        EXPECT_EQ(segments[0].reals.layout.columns,bound?5u:4u);
        unsigned seen=0;
        const auto end=ReadIntervals(dir.path,c,profile,5,5,segments,624,4096,[&](const Values& row) {
            ++seen;const auto original=Row(c,seen,3*seen,bound);
            EXPECT_TRUE(records::SameStamp(original.stamp,row.stamp));
            EXPECT_EQ(row.structural_limit_s,original.structural_limit_s);
        });
        EXPECT_EQ(seen,5u);EXPECT_EQ(end.last.epoch,5u);
    }
}
TEST(PhysicalRunRecords, IndexBindsCadenceFinalAcceptedPhaseAndCompleteFlag) {
    ft::Directory dir;const auto c=Context();const auto config=Config(c);
    const auto index=WriteRun(dir.path,c,config,3);
    auto bad=index;bad.horizon_complete=true;bad.stop_reason.clear();EXPECT_THROW(CheckIndex(c,config,bad),std::exception);
    bad=index;bad.frames.erase(bad.frames.begin()+1);EXPECT_THROW(CheckIndex(c,config,bad),std::exception);
    bad=index;bad.frames.back().stamp.attempt+=1;bad.final=bad.frames.back().stamp;
    EXPECT_THROW(ValidateRecords(dir.path,c,config,bad,64u<<20),std::exception);
    bad=index;bad.stop_reason.clear();EXPECT_THROW(CheckIndex(c,config,bad),std::exception);
    auto foreign=config;foreign.identity.owner+=1;EXPECT_THROW(CheckIndex(c,foreign,index),std::exception);
    EXPECT_NO_THROW(ValidateRecords(dir.path,c,config,index,64u<<20));
}
TEST(PhysicalRunRecords, ZeroApplicableNativeFieldsRetainAnEmptyTypedArray) {
    ft::Directory dir;
    const records::ParentPoints parent{91,92,2,1,0,records::PlasticField::NotApplicable};
    const auto c=records::Context::Create(ft::Id(),1,&parent,1,.125);
    const double xyz[]{0.,-0.,1.};
    const auto frame=records::WriteFrame(dir.path,"initial",c,{{},xyz,3,nullptr,0});
    const auto inventory=Inventory(dir.path,records::TotalByteCap);
    ASSERT_EQ(inventory.size(),3u);
    unsigned empty=0;
    for(const auto& file:inventory)if(!file.bytes) {
        ++empty;EXPECT_EQ(file.file,"initial.plastic.bin");EXPECT_EQ(ReadFileRecord(FileDocument(file)).bytes,0u);
    }
    EXPECT_EQ(empty,1u);
    EXPECT_TRUE(records::ReadFrame(dir.path,c,frame,{}).plastic_points.empty());
}
} // namespace crash::output::physical_run::test
