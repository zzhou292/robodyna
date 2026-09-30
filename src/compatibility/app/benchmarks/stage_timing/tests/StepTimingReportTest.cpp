#include "case/source_assembly_wall/StepTimingReport.h"
#include <gtest/gtest.h>
#include <fstream>
#include <iterator>
#include <unistd.h>

namespace crash::cases::source_assembly_wall {
namespace {
class StepTimingReportTest:public ::testing::Test {
  protected:
    void SetUp() override {char pattern[]="/tmp/robo-dyna-step-timing-XXXXXX";auto* made=mkdtemp(pattern);ASSERT_NE(made,nullptr);root=made;}
    void TearDown() override {std::filesystem::remove_all(root);}
    std::string Read(const std::filesystem::path& p) {std::ifstream in(p);return {std::istreambuf_iterator<char>(in),{}};}
    std::filesystem::path root;
};
TEST_F(StepTimingReportTest, RejectsArchiveContainmentIncludingParentSymlinkBeforeCreatingAnything) {
    const auto archive=root/"accepted";std::filesystem::create_directory(archive);
    std::filesystem::create_directory_symlink(archive,root/"alias");
    EXPECT_THROW(CheckStepTimingPath(archive,archive),std::invalid_argument);
    EXPECT_THROW(CheckStepTimingPath(archive/"new.json",archive),std::invalid_argument);
    EXPECT_THROW(CheckStepTimingPath(root/"alias"/"new.json",archive),std::invalid_argument);
    EXPECT_THROW(CheckStepTimingPath(root/"new-archive"/"new.json",root/"new-archive"),std::invalid_argument);
    EXPECT_NO_THROW(CheckStepTimingPath(root/"accepted-timing.json",archive));
    EXPECT_TRUE(std::filesystem::is_empty(archive));EXPECT_FALSE(std::filesystem::exists(root/"new-archive"));
}
TEST_F(StepTimingReportTest, WritesBoundedCompleteDiagnosticOnlyJsonAndPreservesCreateOnlyDestinations) {
    source_assembly_dynamics::StepTimingSnapshot s;s.enabled=true;s.total[0]={5,1,5,125,30};
    const auto p=root/"timing.json";WriteStepTiming(p,s,2);const auto bytes=Read(p);
    EXPECT_LT(bytes.size(),32u*1024);EXPECT_NE(bytes.find("\"execution_status\":2"),std::string::npos);
    EXPECT_NE(bytes.find("\"stage\":\"step_inclusive\",\"calls\":5,\"failures\":1,\"valid_samples\":5,\"wall_ns\":125"),std::string::npos);
    EXPECT_NE(bytes.find("not kernel time"),std::string::npos);EXPECT_NE(bytes.find("\"last_step\":"),std::string::npos);
    EXPECT_THROW(WriteStepTiming(p,s,0),std::system_error);EXPECT_EQ(Read(p),bytes);
    std::filesystem::create_symlink(p,root/"alias.json");
    EXPECT_THROW(WriteStepTiming(root/"alias.json",s,0),std::system_error);EXPECT_EQ(Read(p),bytes);
    EXPECT_THROW(WriteStepTiming(root/"missing"/"timing.json",s,0),std::system_error);
    EXPECT_FALSE(std::filesystem::exists(root/"missing"));
}
}
}
