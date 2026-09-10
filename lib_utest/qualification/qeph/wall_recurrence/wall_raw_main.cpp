#include "WallCommandLine.h"
#include <iostream>

namespace wall=tl::qualification::qeph::wall_recurrence;
namespace command=wall::command;
namespace io=crash::output;
namespace fs=std::filesystem;

// Linux qualification CLI only. Exact provenance is retained unchanged;
// executable identity is checked here, source/build input authentication stays
// with the root launcher. All argument/input checks precede directory creation.
// Collection completion is independent of local checks or spectral admission.
int main(int argc,char** argv) {
  try {
    io::Require(argc==7,"Usage: qeph_wall_raw CELLS BOOST PROVENANCE_FILE EXPECTED_PROVENANCE_SHA256 NEW_DIRECTORY REMAINING_SET_BYTES");
    const std::string_view cells_arg=argv[1],boost_arg=argv[2],expected_hash=argv[4];
    io::Require(cells_arg=="1"||cells_arg=="2","CELLS must be exactly 1 or 2");
    io::Require(boost_arg=="-8"||boost_arg=="0"||boost_arg=="8","BOOST must be exactly -8, 0 or 8");
    io::Require(command::Hash(expected_hash),"Expected provenance SHA256 must be 64 lowercase hexadecimal characters");
    const unsigned cells=static_cast<unsigned>(cells_arg.front()-'0');
    const double boost=boost_arg=="-8"?-8.:(boost_arg=="8"?8.:0.);
    const auto remaining=command::Budget(argv[6]);
    const auto provenance_path=fs::canonical(command::Path(argv[3]));
    const auto destination=command::Path(argv[5]);
    io::Require(fs::is_regular_file(provenance_path),"Provenance must be a regular file");
    command::NewDirectory(destination);
    const auto provenance=command::ReadProvenance(provenance_path,expected_hash,remaining);
    const auto executable=command::CheckExecutable(provenance.parsed);
    std::cout<<"Observed executable: "<<executable.path<<" sha256="<<executable.sha256
             <<"; provenance_sha256="<<expected_hash<<'\n'<<std::flush;
    wall::RawJobWriter writer(destination,cells,boost,provenance.bytes,provenance_path.string(),remaining);
    const auto job=wall::CollectRawJob(cells,boost,[&writer](const wall::RawJob& state,wall::RawProgress event) {
      writer(state,event);
    });
    const auto& receipt=writer.receipt();
    io::Require(receipt.final_index_present&&receipt.collection_complete==job.collection_complete,
                "Raw collector and retained final index disagree");
    std::cout<<"Retained "<<receipt.files.size()<<" files, "<<receipt.total_bytes
             <<" bytes; raw_collection_complete="<<job.collection_complete
             <<"; screen_admitted=false; trajectory_admitted=false\n";
    return job.collection_complete?0:2;
  } catch(const std::exception& error) {
    std::cerr<<"Raw collection failed; any prior artifacts are retained: "<<error.what()<<'\n';
    return 1;
  }
}
