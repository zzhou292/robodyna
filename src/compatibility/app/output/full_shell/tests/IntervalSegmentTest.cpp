#include "IntervalTestSupport.h"
#include "output/full_shell/FullShellVisualizationPlan.h"
#include <iomanip>
#include <limits>
#include <sstream>

namespace crash::output::full_shell::test {
TEST(IntervalSegments,Exact39ValuesCsvOrderBinaryBytesAndRollover) {
    Directory dir;const auto c=Intervals();const auto ledger=Complete(dir.path,c);
    ASSERT_EQ(ledger.segments.size(),3u);EXPECT_TRUE(ledger.horizon_complete);EXPECT_EQ(ledger.accepted_intervals,5u);
    IntervalCursor cursor;
    std::size_t bytes=0;
    for(std::size_t i=0;i<ledger.segments.size();++i) {
        const auto parsed=ParseSegmentDocument(c,SegmentDocument(c,ledger.segments[i]));
        auto read=ReadIntervalSegment(dir.path,c,5,true,i,cursor,parsed);
        for(std::size_t j=0;j<read.rows();++j) {
            const auto expected=Row(c,cursor.epoch+j+1);
            Equal(read.row(j),expected);
            std::ostringstream legacy;
            legacy<<std::setprecision(17)<<expected.integers[0]<<','<<expected.integers[1]<<','<<expected.integers[2]
                <<','<<expected.reals[0]<<','<<expected.integers[3]<<','<<expected.reals[1];
            for(std::size_t k=2;k<35;++k)legacy<<','<<expected.reals[k];
            legacy<<'\n';
            EXPECT_EQ(interval::CsvRow(read.row(j)),legacy.str());
        }
        cursor=read.after();
        bytes+=parsed.integers.bytes+parsed.reals.bytes;
    }
    EXPECT_EQ(bytes,5*312u);
    EXPECT_EQ(cursor.first_contact,2u);EXPECT_EQ(cursor.last_contact,3u);EXPECT_EQ(cursor.contact_intervals,2u);
    EXPECT_EQ(interval::IntegerFields(),(std::vector<std::string>{"owner_id","base_epoch","attempt","accepted_epoch"}));
    EXPECT_EQ(interval::RealFields().size(),35u);
}
TEST(IntervalSegments,ExplicitPrefixAndInvalidInputRetryWithoutBufferGrowth) {
    Directory dir;const auto c=Intervals();IntervalWriter writer(dir.path,"prefix",c);
    const auto bytes=writer.buffer_bytes();
    auto wrong=Row(c,1);++wrong.integers[interval::Attempt];
    EXPECT_THROW(writer.Append(wrong,IntervalStamp(Row(c,1))),std::exception);
    EXPECT_EQ(writer.rows_received(),0u);EXPECT_FALSE(writer.failed());
    Append(writer,Row(c,1));
    EXPECT_THROW(Append(writer,Row(c,3)),std::exception);
    EXPECT_EQ(writer.rows_received(),1u);EXPECT_FALSE(writer.failed());
    EXPECT_THROW(writer.Finish(),std::exception);
    Append(writer,Row(c,2));Append(writer,Row(c,3));
    const auto ledger=writer.FinishPrefix();
    ASSERT_EQ(ledger.segments.size(),2u);EXPECT_FALSE(ledger.horizon_complete);EXPECT_EQ(ledger.accepted_intervals,3u);
    EXPECT_EQ(writer.buffer_bytes(),bytes);
    IntervalCursor cursor;
    for(std::size_t i=0;i<ledger.segments.size();++i)cursor=ReadIntervalSegment(dir.path,c,3,false,i,cursor,ledger.segments[i]).after();
    EXPECT_EQ(cursor.epoch,3u);
    EXPECT_THROW(ReadIntervalSegment(dir.path,c,5,true,1,ledger.segments[1].before,ledger.segments[1]),std::exception);
    EXPECT_THROW(Append(writer,Row(c,4)),std::exception);
}
TEST(IntervalSegments,ExpectedSequenceSourceCountsAndRehashedTailFailuresAreAtomic) {
    Directory dir;const auto c=Intervals();const auto ledger=Complete(dir.path,c);
    const auto& first=ledger.segments.front();
    auto visible=ReadIntervalSegment(dir.path,c,5,true,0,{},first);
    const auto initial_cursor=visible.after();
    const auto& last=ledger.segments.back();
    EXPECT_THROW(visible=ReadIntervalSegment(dir.path,c,5,true,2,{},last),std::exception);
    EXPECT_TRUE(SameCursor(visible.after(),initial_cursor));
    auto wrong=first;++wrong.identity.configuration;
    EXPECT_THROW(ReadIntervalSegment(dir.path,c,5,true,0,{},wrong),std::exception);
    wrong=first;--wrong.row_count;
    EXPECT_THROW(ReadIntervalSegment(dir.path,c,5,true,0,{},wrong),std::exception);
    wrong=first;++wrong.after.attempt;
    EXPECT_THROW(ReadIntervalSegment(dir.path,c,5,true,0,{},wrong),std::exception);
    auto doc=SegmentDocument(c,first);doc.AddMember("undeclared",1,doc.GetAllocator());
    EXPECT_THROW(ParseSegmentDocument(c,doc),std::exception);
    const auto ids=ReadBounded(dir.path/first.integers.file,first.integers.bytes);
    auto gap=ids;
    for(unsigned j=0;j<8;++j)gap[8*(interval::IntegerCount+interval::BaseEpoch)+j]=0;
    wrong=first;wrong.integers.sha256=Sha256(gap);Overwrite(dir.path/wrong.integers.file,gap);
    EXPECT_THROW(visible=ReadIntervalSegment(dir.path,c,5,true,0,{},wrong),std::exception);
    EXPECT_TRUE(SameCursor(visible.after(),initial_cursor));
    Overwrite(dir.path/first.integers.file,ids);
    const auto original=ReadBounded(dir.path/first.reals.file,first.reals.bytes);
    auto checksum=original;checksum.back()^=1;
    Overwrite(dir.path/first.reals.file,checksum);
    EXPECT_THROW(ReadIntervalSegment(dir.path,c,5,true,0,{},first),std::exception);
    auto bad=original;const auto nan=UINT64_C(0x7ff8000000000000);
    for(unsigned j=0;j<8;++j)bad[bad.size()-8+j]=static_cast<char>(nan>>(8*j));
    wrong=first;wrong.reals.sha256=Sha256(bad);Overwrite(dir.path/wrong.reals.file,bad);
    EXPECT_THROW(visible=ReadIntervalSegment(dir.path,c,5,true,0,{},wrong),std::exception);
    EXPECT_TRUE(SameCursor(visible.after(),initial_cursor));
    Overwrite(dir.path/first.reals.file,original.substr(0,original.size()-1));
    EXPECT_THROW(ReadIntervalSegment(dir.path,c,5,true,0,{},first),std::exception);
    Overwrite(dir.path/first.reals.file,original+"x");
    EXPECT_THROW(ReadIntervalSegment(dir.path,c,5,true,0,{},first),std::exception);
    Overwrite(dir.path/first.reals.file,original);
    EXPECT_NO_THROW(visible=ReadIntervalSegment(dir.path,c,5,true,0,{},first));
}
TEST(IntervalSegments,FiniteMagnitudeCursorAndFullForecastShareExactChunkArithmetic) {
    const auto c=Intervals();const auto first=Row(c,1);
    for(int fault=0;fault<6;++fault) {
        auto row=first;
        if(fault==0)row.reals[interval::MaximumThicknessRatio]=std::numeric_limits<double>::quiet_NaN();
        if(fault==1)row.reals[interval::ActiveNodes]=.5;
        if(fault==2)row.reals[interval::NativeResidual]=1;
        if(fault==3)row.reals[interval::MaximumPenetration]=1;
        if(fault==4)row.reals[interval::YieldedPoints]=5;
        if(fault==5)row.reals[interval::KickDt]=c.fixed_dt;
        EXPECT_THROW(AdvanceInterval(c,{},row),std::exception);
    }
    auto previous=AdvanceInterval(c,{},first);previous.velocity_time=std::numeric_limits<double>::infinity();
    EXPECT_THROW(AdvanceInterval(c,previous,Row(c,2)),std::exception);
    auto limits=c;limits.limits.host_bytes=1;
    Directory dir;EXPECT_THROW(IntervalWriter(dir.path,"bad",limits),std::exception);
    limits=c;limits.planned_intervals=1;limits.limits.file_bytes=kArtifactFileCap;limits.limits.host_bytes=4*312;
    IntervalWriter small(dir.path,"small",limits);
    EXPECT_EQ(small.buffer_bytes(),312u);
    Append(small,Row(limits,1));EXPECT_EQ(small.Finish().accepted_intervals,1u);
    const auto max=interval::PlanChunks(64,312);
    EXPECT_EQ(max.chunks,64u);EXPECT_EQ(max.total_bytes,64*312u);
    EXPECT_THROW(interval::PlanChunks(65,312),std::exception);
    EXPECT_THROW(interval::PlanChunks(UINT64_MAX),std::exception);
    const auto bounded=interval::PlanChunks(1342178);
    EXPECT_EQ(bounded.total_bytes,418759536u);
    PlanRequest r;r.nodes=359785;r.parents=349645;r.plastic_points=3*r.parents;r.frames=88;
    r.fixed_dt=0x1p-26;r.requested_duration=.02;r.intervals=1342178;
    r.static_files={{"manifest.json",1024},{"frame-index.json",1024},{"configuration.json",1024}};
    const auto full=PlanArchive(r);
    EXPECT_EQ(full.rows_per_chunk,bounded.rows_per_chunk);EXPECT_EQ(full.interval_chunks,bounded.chunks);
    EXPECT_EQ(full.interval_bytes,bounded.total_bytes);EXPECT_EQ(full.forecast_bytes,2135428608u);
}
} // namespace crash::output::full_shell::test
