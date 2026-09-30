#include "CsvLedgerSegments.h"

#include <gtest/gtest.h>
#include <cstdlib>
#include <limits>

namespace crash::output {
namespace {
namespace fs=std::filesystem;
struct TempDirectory {
    fs::path path;
    TempDirectory() {
        const auto pattern=(fs::temp_directory_path()/"csv-ledger-segments-XXXXXX").string();
        std::vector<char> buffer(pattern.begin(),pattern.end());buffer.push_back(0);
        const auto* made=::mkdtemp(buffer.data());
        Require(made,"Could not create isolated CSV ledger test directory");path=made;
    }
    ~TempDirectory() {std::error_code error;fs::remove_all(path,error);}
};
constexpr const char* Header="a,b\n";
CsvLedgerPlan SmallPlan(std::uint64_t rows=5) {return PlanCsvLedger("contact.csv",Header,rows,10,24);}
Document Metadata(const CsvLedgerPlan& plan) {
    Document result;result.SetObject();AppendCsvLedgerSegments(result,&plan,1);return result;
}

TEST(CsvLedgerSegmentsCheck, DeterministicRowBoundariesIncludeEveryRepeatedHeader) {
    const auto single=SmallPlan(2);
    ASSERT_EQ(single.segments.size(),1u);
    EXPECT_EQ(single.segments[0].file,"contact.csv");
    EXPECT_EQ(single.total_bytes,24u);
    const auto plan=SmallPlan();
    ASSERT_EQ(plan.segments.size(),3u);
    EXPECT_EQ(plan.column_count,2u);
    EXPECT_EQ(plan.total_bytes,62u);
    for(unsigned i=0;i<3;++i) {
        const auto& segment=plan.segments[i];
        EXPECT_EQ(segment.first_epoch,1+2*i);
        EXPECT_EQ(segment.last_epoch,i==2?5:2+2*i);
        EXPECT_EQ(segment.row_count,i==2?1:2);
        EXPECT_EQ(segment.byte_cap,i==2?14:24);
    }
    EXPECT_EQ(plan.segments[1].file,"contact-0001.csv");
    EXPECT_EQ(plan.segments[2].file,"contact-0002.csv");
    EXPECT_EQ(plan.header_sha256,Sha256(Header));
    EXPECT_NO_THROW(ValidateCsvLedgerPlan(plan,Header,24));
}

TEST(CsvLedgerSegmentsCheck, MetadataRoundtripBindsActualHeaderAndStagesAppend) {
    const auto plan=SmallPlan();
    auto document=Metadata(plan);
    auto parsed=ParseCsvLedgerSegments(document[kCsvLedgerSegmentsField]);
    ASSERT_EQ(parsed.size(),1u);
    EXPECT_TRUE(SameCsvLedgerPlan(plan,parsed[0]));
    EXPECT_NO_THROW(ValidateCsvLedgerPlan(parsed[0],Header,24));
    EXPECT_THROW(ValidateCsvLedgerPlan(parsed[0],"b,a\n",24),std::runtime_error);
    EXPECT_THROW(ValidateCsvLedgerPlan(parsed[0],Header),std::runtime_error);
    document[kCsvLedgerSegmentsField][0u].AddMember("future_extension",true,document.GetAllocator());
    parsed=ParseCsvLedgerSegments(document[kCsvLedgerSegmentsField]);
    EXPECT_TRUE(SameCsvLedgerPlan(plan,parsed[0]));
    EXPECT_THROW(AppendCsvLedgerSegments(document,&plan,1),std::runtime_error);
    EXPECT_EQ(document.MemberCount(),1u);
    Document untouched;untouched.SetObject();Integer(untouched,"sentinel",17);
    auto bad=plan;bad.segments[1].file="../unsafe.csv";
    EXPECT_THROW(AppendCsvLedgerSegments(untouched,&bad,1),std::runtime_error);
    EXPECT_EQ(untouched.MemberCount(),1u);
    EXPECT_EQ(untouched["sentinel"].GetUint64(),17u);
}

TEST(CsvLedgerSegmentsCheck, MalformedRequiredMetadataCannotReplacePublishedPlans) {
    const auto plan=SmallPlan();
    const auto original=Metadata(plan);
    for(unsigned fault=0;fault<10;++fault) {
        SCOPED_TRACE(fault);
        Document altered;altered.CopyFrom(original,altered.GetAllocator());
        auto& list=altered[kCsvLedgerSegmentsField];
        auto& item=list[0u];auto& segments=item["segments"];
        switch(fault) {
            case 0:item["interval_count"].SetDouble(5);break;
            case 1:item.AddMember("interval_count",5,altered.GetAllocator());break;
            case 2:segments[1u]["first_epoch"].SetUint64(2);break;
            case 3:segments[1u]["file"].SetString("other.csv",altered.GetAllocator());break;
            case 4:segments.PopBack();break;
            case 5:segments[0u]["byte_cap"].SetUint64(kArtifactFileCap+1);break;
            case 6:item["header_sha256"].SetString("wrong",altered.GetAllocator());break;
            case 7:item["max_row_bytes"].SetUint64(0);break;
            case 8:item["logical_file"].SetString("../contact.csv",altered.GetAllocator());break;
            case 9:item.RemoveMember("column_count");break;
        }
        std::vector<CsvLedgerPlan> published{plan};
        EXPECT_THROW(published=ParseCsvLedgerSegments(list),std::runtime_error);
        ASSERT_EQ(published.size(),1u);
        EXPECT_TRUE(SameCsvLedgerPlan(published[0],plan));
    }
    auto duplicate=Metadata(plan);
    Value copy;copy.CopyFrom(duplicate[kCsvLedgerSegmentsField][0u],duplicate.GetAllocator());
    duplicate[kCsvLedgerSegmentsField].PushBack(copy,duplicate.GetAllocator());
    EXPECT_THROW(ParseCsvLedgerSegments(duplicate[kCsvLedgerSegmentsField]),std::runtime_error);
}

TEST(CsvLedgerSegmentsCheck, StreamRolloverPreservesExactRowsAndRequiresCompleteHorizon) {
    TempDirectory directory;const auto plan=SmallPlan();
    CsvLedgerWriter writer(directory.path,Header,plan,24);
    writer.Append(1,"1,2\n");
    EXPECT_THROW(writer.Finish(),std::runtime_error);
    EXPECT_FALSE(writer.failed());
    writer.Append(2,"3,4\n");writer.Append(3,"5,6\n");
    writer.Append(4,"7,8\n");writer.Append(5,"9,0\n");
    writer.Finish();
    EXPECT_EQ(writer.rows_written(),5u);
    EXPECT_FALSE(writer.failed());
    EXPECT_EQ(ReadBounded(directory.path/plan.segments[0].file,24),"a,b\n1,2\n3,4\n");
    EXPECT_EQ(ReadBounded(directory.path/plan.segments[1].file,24),"a,b\n5,6\n7,8\n");
    EXPECT_EQ(ReadBounded(directory.path/plan.segments[2].file,24),"a,b\n9,0\n");
    EXPECT_THROW(writer.Append(6,"1,2\n"),std::runtime_error);
    EXPECT_THROW(writer.Finish(),std::runtime_error);
}

TEST(CsvLedgerSegmentsCheck, ExplicitAcceptedPrefixClosesOnlyWrittenSegmentsWithoutInventedRows) {
    TempDirectory directory;const auto planned=SmallPlan(5);CsvLedgerWriter writer(directory.path,Header,planned,24);
    EXPECT_THROW(writer.FinishPrefix(),std::runtime_error);EXPECT_FALSE(writer.failed());
    writer.Append(1,"1,2\n");writer.Append(2,"3,4\n");writer.Append(3,"5,6\n");
    EXPECT_THROW(writer.Finish(),std::runtime_error);const auto prefix=writer.FinishPrefix();
    EXPECT_TRUE(SameCsvLedgerPlan(prefix,SmallPlan(3)));EXPECT_EQ(writer.rows_written(),3u);EXPECT_FALSE(writer.failed());
    EXPECT_EQ(ReadBounded(directory.path/"contact.csv",24),"a,b\n1,2\n3,4\n");
    EXPECT_EQ(ReadBounded(directory.path/"contact-0001.csv",24),"a,b\n5,6\n");EXPECT_FALSE(fs::exists(directory.path/"contact-0002.csv"));
    EXPECT_THROW(writer.Append(4,"7,8\n"),std::runtime_error);
    EXPECT_THROW(writer.FinishPrefix(),std::runtime_error);
}

TEST(CsvLedgerSegmentsCheck, InvalidRowsPreserveBytesAndAllowAnUnchangedRetry) {
    TempDirectory directory;const auto plan=SmallPlan(2);
    CsvLedgerWriter writer(directory.path,Header,plan,24);
    writer.Append(1,"1,2\n");writer.Flush();
    const auto before=ReadBounded(directory.path/"contact.csv",24);
    EXPECT_THROW(writer.Append(3,"3,4\n"),std::runtime_error);
    for(const std::string row:{"123456,7890\n","1,2,3\n","1\n,2\n",",2\n","1,2","\"1\",2\n"}) {
        SCOPED_TRACE(row);
        EXPECT_THROW(writer.Append(2,row),std::runtime_error);
        EXPECT_EQ(writer.rows_written(),1u);
        EXPECT_FALSE(writer.failed());
        EXPECT_EQ(ReadBounded(directory.path/"contact.csv",24),before);
    }
    writer.CheckRow(2,"3,4\n");
    EXPECT_EQ(writer.rows_written(),1u);
    writer.Append(2,"3,4\n");writer.Finish();
    EXPECT_EQ(ReadBounded(directory.path/"contact.csv",24),before+"3,4\n");
}

TEST(CsvLedgerSegmentsCheck, ExistingFutureDestinationAndLateRolloverConflictAreCreateOnly) {
    const auto plan=SmallPlan(3);
    TempDirectory occupied;
    WriteBytes(occupied.path/plan.segments[1].file,"Preserved\n");
    EXPECT_THROW(CsvLedgerWriter(occupied.path,Header,plan,24),std::runtime_error);
    EXPECT_FALSE(fs::exists(occupied.path/plan.segments[0].file));
    EXPECT_EQ(ReadBounded(occupied.path/plan.segments[1].file,100),"Preserved\n");
    TempDirectory late;
    CsvLedgerWriter writer(late.path,Header,plan,24);
    writer.Append(1,"1,2\n");writer.Append(2,"3,4\n");writer.Flush();
    const auto before=ReadBounded(late.path/plan.segments[0].file,24);
    WriteBytes(late.path/plan.segments[1].file,"Later conflict\n");
    EXPECT_THROW(writer.Append(3,"5,6\n"),std::runtime_error);
    EXPECT_TRUE(writer.failed());
    EXPECT_EQ(writer.rows_written(),2u);
    EXPECT_EQ(ReadBounded(late.path/plan.segments[0].file,24),before);
    EXPECT_EQ(ReadBounded(late.path/plan.segments[1].file,100),"Later conflict\n");
    EXPECT_THROW(writer.Append(3,"5,6\n"),std::runtime_error);
    EXPECT_THROW(writer.Finish(),std::runtime_error);
}

TEST(CsvLedgerSegmentsCheck, HeaderNamesAndCapacityRejectBeforeAnyOutput) {
    for(const char* name:{"../bad.csv","bad/name.csv","bad.csv/child",".csv","bad.txt"})
        EXPECT_THROW(PlanCsvLedger(name,Header,1,10,24),std::runtime_error);
    for(const char* header:{"a,a\n","a,\n","a,b","a,\nb\n","a,\"b\"\n"})
        EXPECT_THROW(PlanCsvLedger("good.csv",header,1,10,24),std::runtime_error);
    EXPECT_THROW(PlanCsvLedger("good.csv",Header,0,10,24),std::runtime_error);
    EXPECT_THROW(PlanCsvLedger("good.csv",Header,1,0,24),std::runtime_error);
    EXPECT_THROW(PlanCsvLedger("good.csv",Header,1,10,kArtifactFileCap+1),std::runtime_error);
    EXPECT_THROW(PlanCsvLedger("good.csv",Header,17,10,24),std::runtime_error);
    EXPECT_THROW(PlanCsvLedger("good.csv",Header,std::numeric_limits<std::uint64_t>::max(),10,24),std::runtime_error);
    EXPECT_NO_THROW(PlanCsvLedger("good.csv",Header,16,10,24));
}
} // namespace
} // namespace crash::output
