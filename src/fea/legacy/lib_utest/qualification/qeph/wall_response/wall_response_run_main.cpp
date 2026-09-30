#include "WallResponseCommand.h"
#include "WallResponseRuntime.h"
#include <gtest/gtest.h>
#include <iostream>

namespace response=tl::qualification::qeph::wall_response;
namespace io=crash::output;
namespace {
response::Run* selected=nullptr;
TEST(QephWallResponse, FrozenBroadsideResponse) {
  ASSERT_NE(selected,nullptr);
  ASSERT_NO_FATAL_FAILURE(response::Execute(*selected)); EXPECT_TRUE(selected->completed);
}
}
int main(int argc,char** argv) {
  try {
    io::Require(argc>=6,"Usage: qeph_wall_response_run CELLS REFINEMENT PROVENANCE.json EXPECTED_SHA256 NEW-DIRECTORY [--gtest_output=xml:PATH]");
    const std::string cells=argv[1],refinement=argv[2];
    io::Require((cells=="1"||cells=="2")&&(refinement=="1"||refinement=="2"||refinement=="4"),"Frozen cells/refinement required");
    const auto launch=response::command::ReadLaunch(argv[3],argv[4]);
    response::Run run; run.config={static_cast<unsigned>(cells[0]-'0'),static_cast<unsigned>(refinement[0]-'0'),launch.selected_h,launch.binding.screen_index_sha256};
    std::string error; const bool prepared=response::BuildModel(run.config.cells,run.model,error); io::Require(prepared,error.c_str());
    run.samples.reserve(response::SampleCount);
    const auto destination=response::command::shared::Path(argv[5]); response::command::shared::NewDirectory(destination);
    std::vector<char*> arguments{argv[0]};
    for(int i=6;i<argc;++i) {
      io::Require(std::string(argv[i]).rfind("--gtest_output=xml:",0)==0,"Only explicit GTest XML output is accepted"); arguments.push_back(argv[i]);
    }
    int count=static_cast<int>(arguments.size()); ::testing::InitGoogleTest(&count,arguments.data());
    ::testing::GTEST_FLAG(filter)="QephWallResponse.FrozenBroadsideResponse";
    io::Require(std::filesystem::create_directory(destination),"Cannot create new response directory");
    io::WriteBytes(destination/"provenance.json",launch.provenance_bytes);
    io::Document pending; pending.SetObject(); io::String(pending,"stage","prepared-not-executed");
    io::Integer(pending,"planned_intervals",response::Steps(run.config)); io::Boolean(pending,"completed",false);
    io::String(pending,"screen_index_sha256",run.config.screen_index_sha); io::WriteJson(destination/"initial.json",pending);
    selected=&run; const int tests=RUN_ALL_TESTS(); selected=nullptr;
    response::command::WriteRunRecord(destination,run,launch);
    std::cout<<"Retained "<<run.accepted_steps<<" accepted intervals; broadside_run_complete="<<run.completed<<"; refinement_admitted=false\n";
    return tests||!run.completed?2:0;
  } catch(const std::exception& e) { std::cerr<<"Wall-response run failed: "<<e.what()<<'\n'; return 1; }
}
