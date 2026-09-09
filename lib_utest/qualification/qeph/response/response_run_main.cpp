#include "ResponseRuntime.h"
#include "ResponseProtocol.h"
#include <gtest/gtest.h>
#include <iostream>

namespace response=tl::qualification::qeph::response;
namespace io=crash::output;
namespace {
response::Run* selected=nullptr;
TEST(QephResponse, FullFrozenPulse) {
  ASSERT_NE(selected,nullptr);
  ASSERT_NO_FATAL_FAILURE(response::Execute(*selected));
  EXPECT_TRUE(selected->completed);
}
}
int main(int argc,char** argv) {
  try {
    io::Require(argc>=7,"Usage: qeph_response_run CELLS REFINEMENT DECISION.json RAW.json PROVENANCE.json NEW-DIRECTORY [--gtest_output=xml:PATH]");
    const std::string cells=argv[1],refinement=argv[2];
    io::Require((cells=="1"||cells=="2")&&(refinement=="1"||refinement=="2"||refinement=="4"),"Frozen fixture/refinement required");
    response::Run run; run.config={static_cast<unsigned>(cells[0]-'0'),static_cast<unsigned>(refinement[0]-'0')};
    std::string error; const bool valid=response::BuildModel(run.config.cells,run.model,error); io::Require(valid,error.c_str());
    run.fields=response::Dictionary(run.model); io::Require(run.fields.size()<=response::MaxFields,"Field capacity exceeded");
    run.samples.reserve(response::SampleCount);
    const auto decision=io::ReadBounded(argv[3],response::FileCap),raw=io::ReadBounded(argv[4],response::FileCap);
    const auto source=io::ReadBounded(argv[5],1024*1024); auto provenance=response::protocol::Parse(source);
    auto binding=response::CheckAdmission(decision,raw,provenance);
    const auto binary=std::filesystem::read_symlink("/proc/self/exe");
    const auto binary_bytes=io::ReadBounded(binary,64*1024*1024); binding.runtime_sha256=io::Sha256(binary_bytes);
    io::Document bound; bound.SetObject(); io::String(bound,"input_provenance_sha256",io::Sha256(source));
    io::Integer(bound,"input_provenance_bytes",source.size()); io::String(bound,"observed_runtime_binary_path",binary.string());
    response::protocol::Object(bound,"input_provenance",provenance);
    const std::filesystem::path destination=argv[6];
    io::Require(!std::filesystem::exists(std::filesystem::symlink_status(destination)),"Output directory must be new");
    const auto parent=destination.has_parent_path()?destination.parent_path():std::filesystem::path(".");
    io::Require(std::filesystem::is_directory(parent),"Output parent must exist");
    std::vector<char*> arguments{argv[0]};
    for(int i=7;i<argc;++i) {
      io::Require(std::string(argv[i]).rfind("--gtest_output=xml:",0)==0,"Only an explicit GTest XML output is accepted");
      arguments.push_back(argv[i]);
    }
    int count=arguments.size(); ::testing::InitGoogleTest(&count,arguments.data());
    ::testing::GTEST_FLAG(filter)="QephResponse.FullFrozenPulse";
    io::Require(std::filesystem::create_directory(destination),"Cannot create new response directory");
    // Existing artifact writes are externally serialized, not atomic exclusive.
    selected=&run; const int tests=RUN_ALL_TESTS(); selected=nullptr;
    const auto report=response::Encode(response::DescribeRun(run,binding,bound));
    io::WriteBytes(destination/"run.json",report);
    std::cout<<"Retained "<<run.accepted_steps<<" accepted endpoints; single_run_complete="<<run.completed
             <<"; refinement_admitted=false\n";
    return tests||!run.completed?2:0;
  } catch(const std::exception& e) { std::cerr<<"Response report failed: "<<e.what()<<'\n'; return 1; }
}
