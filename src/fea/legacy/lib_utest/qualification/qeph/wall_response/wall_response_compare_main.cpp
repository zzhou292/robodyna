#include "WallResponseJson.h"
#include <iostream>

namespace r=tl::qualification::qeph::wall_response;
namespace io=crash::output;
int main(int argc,char** argv) {
  try {
    io::Require(argc==8,"Usage: qeph_wall_response_compare H/run.json H_SHA H2/run.json H2_SHA H4/run.json H4_SHA NEW-COMPARISON.json");
    io::Require(!std::filesystem::exists(std::filesystem::symlink_status(argv[7])),"Comparison destination must be new");
    std::array<r::Run,3> runs; std::array<r::ReportBinding,3> bindings; std::array<std::string,3> hashes;
    for(unsigned i=0;i<3;++i) {
      const auto bytes=io::ReadBounded(argv[1+2*i],r::ReportByteCap); hashes[i]=argv[2+2*i];
      runs[i]=r::ReadRun(bytes,hashes[i],bindings[i]);
      io::Require(i==0||r::SameBinding(bindings[0],bindings[i]),"Refinements differ in exact screen/runtime/provenance binding");
    }
    const auto document=r::DescribeComparison(runs,hashes,bindings[0]);
    io::WriteBytes(argv[7],r::EncodeReport(document));
    const bool passed=document["broadside_refinement_passed"].GetBool();
    std::cout<<(passed?"Frozen broadside refinement passed\n":"Frozen broadside refinement rejected\n"); return passed?0:2;
  } catch(const std::exception& e) { std::cerr<<"Wall-response comparison failed: "<<e.what()<<'\n'; return 1; }
}
