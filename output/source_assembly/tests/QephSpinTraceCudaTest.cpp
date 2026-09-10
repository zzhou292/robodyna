#include "SourceAssemblyWallFieldTestSupport.h"
#include "output/source_assembly/QephSpinTrace.h"
#include "case/source_assembly_dynamics/tests/Fixture.h"
#include <fstream>
#include <cstdlib>

namespace crash::output::assembly::test {
namespace {
struct SpinDirectory {
    std::filesystem::path path;
    SpinDirectory() {std::string p=(std::filesystem::temp_directory_path()/"assembly-spin-XXXXXX").string();
        std::vector<char> c(p.begin(),p.end());c.push_back(0);const auto* made=mkdtemp(c.data());Require(made,"Temporary trace directory failed");path=made;}
    ~SpinDirectory(){std::error_code e;std::filesystem::remove_all(path,e);}
};
std::vector<Document> Rows(const std::filesystem::path& path) {
    const auto bytes=ReadBounded(path,SpinTraceByteCap);std::istringstream input(bytes);std::string line;std::vector<Document> result;
    while(std::getline(input,line)){Require(line.size()<SpinTraceRowCap,"Trace line exceeds limit");Document d;
        d.Parse<rapidjson::kParseFullPrecisionFlag>(line.data(),line.size());Require(!d.HasParseError(),"Trace contains partial JSON");result.push_back(std::move(d));}
    return result;
}
void Initialize(dynamics::SourceAssemblyWallCase& run) {
    auto c=Configuration();c.observe_qeph_spin_node=2181592;
    const auto r=run.Initialize(prepared::WallAssembly(),PreparedWall(),c);Require(bool(r),r.message);
}
}
TEST(QephSpinTraceLive, ActualCompleteContactTraceUsesAcceptedBasePacketsAndBoundedCadence) {
    SpinDirectory dir;dynamics::SourceAssemblyWallCase run;Initialize(run);const auto p=dir.path/"spin.jsonl";
    QephSpinTrace trace(p,run,64,8);
    for(unsigned i=0;i<64;++i){const auto r=run.Step();ASSERT_TRUE(r)<<r.message;trace.RecordInterval(run);}
    trace.Finish(run,true,"");const auto rows=Rows(p);ASSERT_EQ(rows.size(),11u);
    EXPECT_STREQ(rows[0]["record"].GetString(),"header");EXPECT_EQ(rows[0]["source_node_id"].GetUint64(),2181592u);
    EXPECT_EQ(rows[0]["source_inventory_sha256"].GetString(),run.bindings()->source().data().identity.sha256);
    EXPECT_FALSE(rows[1]["parents"][0u]["retained_kinematics"]["available"].GetBool());
    for(std::size_t i=1;i+1<rows.size();++i){const auto e=i==1?1u:8u*static_cast<unsigned>(i-1);
        EXPECT_EQ(rows[i]["enclosing_accepted_stamp"]["epoch"].GetUint64(),e);
        EXPECT_EQ(rows[i]["base_stamp"]["epoch"].GetUint64(),e-1);EXPECT_EQ(rows[i]["parents"].Size(),2u);}
    EXPECT_TRUE(rows.back()["horizon_complete"].GetBool());EXPECT_EQ(rows.back()["accepted_epoch"].GetUint64(),64u);
    EXPECT_GT(run.diagnostics()->active_contact_nodes,0u);
}
TEST(QephSpinTraceLive, RejectedAttemptRetainsAcceptedTerminalAndInitialPrefixIsExplicit) {
    using Access=cases::source_assembly_dynamics::SourceAssemblyDynamicsTestAccess;
    SpinDirectory dir;dynamics::SourceAssemblyWallCase run;Initialize(run);const auto p=dir.path/"prefix.jsonl";
    QephSpinTrace trace(p,run,8,4);
    for(unsigned i=0;i<3;++i){ASSERT_TRUE(run.Step());trace.RecordInterval(run);}
    const auto stamp=run.owner()->accepted();const auto* retained=run.accepted_qeph_spin();const auto attempt=retained->attempt;
    const auto failure=Access::RejectLate(run,Access::Fault::LastWallFace);ASSERT_FALSE(failure);
    EXPECT_EQ(run.accepted_qeph_spin(),retained);trace.Finish(run,false,failure.message);
    EXPECT_TRUE(tl::fea::trial_identity::SameStamp(stamp,run.owner()->accepted()));const auto rows=Rows(p);ASSERT_EQ(rows.size(),4u);
    EXPECT_EQ(rows[2]["enclosing_accepted_stamp"]["epoch"].GetUint64(),3u);EXPECT_EQ(rows[2]["attempt"].GetUint64(),attempt);
    EXPECT_FALSE(rows.back()["horizon_complete"].GetBool());EXPECT_EQ(rows.back()["accepted_epoch"].GetUint64(),3u);
    dynamics::SourceAssemblyWallCase initial;Initialize(initial);QephSpinTrace empty(dir.path/"empty.jsonl",initial,8,4);
    empty.Finish(initial,false,"Explicit pre-step checkpoint");const auto zero=Rows(dir.path/"empty.jsonl");ASSERT_EQ(zero.size(),2u);
    EXPECT_FALSE(zero.back()["has_force_stage"].GetBool());EXPECT_EQ(zero.back()["accepted_epoch"].GetUint64(),0u);
}
TEST(QephSpinTraceLive, DuplicateIntervalPoisonsAndCreateOnlyForecastPreservesExistingBytes) {
    SpinDirectory dir;dynamics::SourceAssemblyWallCase run;Initialize(run);const auto p=dir.path/"duplicate.jsonl";
    QephSpinTrace trace(p,run,8,4);ASSERT_TRUE(run.Step());trace.RecordInterval(run);
    EXPECT_THROW(trace.RecordInterval(run),std::runtime_error);
    EXPECT_THROW(trace.Finish(run,false,"duplicate"),std::runtime_error);
    const auto rows=Rows(p);ASSERT_EQ(rows.size(),2u);EXPECT_STREQ(rows.back()["record"].GetString(),"accepted_force_stage");
    dynamics::SourceAssemblyWallCase initial;Initialize(initial);const auto sentinel=dir.path/"sentinel.jsonl";WriteBytes(sentinel,"retained\n");
    EXPECT_ANY_THROW(QephSpinTrace(sentinel,initial,8,4));EXPECT_EQ(ReadBounded(sentinel,64),"retained\n");
    EXPECT_THROW(QephSpinTrace(dir.path/"absent.jsonl",initial,8192,1),std::runtime_error);
    EXPECT_FALSE(std::filesystem::exists(dir.path/"absent.jsonl"));EXPECT_EQ(initial.owner()->accepted().epoch,0u);
}
}
