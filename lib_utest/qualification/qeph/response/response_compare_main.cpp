#include "ResponseReport.h"
#include <iostream>

int main(int argc,char** argv) {
  namespace r=tl::qualification::qeph::response; namespace io=crash::output;
  try {
    io::Require(argc==5,"Usage: qeph_response_compare H/run.json H2/run.json H4/run.json NEW-COMPARISON.json");
    io::Require(!std::filesystem::exists(std::filesystem::symlink_status(argv[4])),"Comparison output must be new");
    std::array<r::Run,3> runs; std::array<r::Binding,3> bindings; std::array<std::string,3> hashes;
    for(unsigned i=0;i<3;++i) {
      const auto bytes=io::ReadBounded(argv[i+1],r::FileCap); runs[i]=r::ReadRun(bytes,bindings[i]); hashes[i]=io::Sha256(bytes);
      if(i) io::Require(bindings[i].decision_sha256==bindings[0].decision_sha256&&bindings[i].raw_sha256==bindings[0].raw_sha256&&
        bindings[i].runtime_sha256==bindings[0].runtime_sha256,"Refinements use different matrix or runtime evidence");
    }
    const auto comparison=r::Compare(runs);
    io::WriteBytes(argv[4],r::Encode(r::DescribeComparison(comparison,runs,hashes,bindings[0])));
    std::cout<<comparison.diagnostic<<'\n'; return comparison.passed?0:2;
  } catch(const std::exception& e) { std::cerr<<"Response comparison rejected: "<<e.what()<<'\n'; return 1; }
}
