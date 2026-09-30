#include "WallCommandLine.h"
#include "WallJobReport.h"
#include <iostream>
#include <stdexcept>

namespace wall=tl::qualification::qeph::wall_recurrence;
namespace command=wall::command;
namespace io=crash::output;
namespace fs=std::filesystem;

// The exact externally hashed configuration is also the retained analysis
// provenance. The launcher authenticates its opaque source/build bindings.
// This command reads existing raw evidence and never evaluates a native map.
int main(int argc,char** argv) {
  try {
    io::Require(argc==5,"Usage: qeph_wall_analysis JOB_CONFIG_JSON EXPECTED_CONFIG_SHA256 NEW_DIRECTORY REMAINING_SET_BYTES");
    const std::string_view expected_hash=argv[2];
    io::Require(command::Hash(expected_hash),"Expected configuration SHA256 must be 64 lowercase hexadecimal characters");
    const auto remaining=command::Budget(argv[4]);
    const auto config_path=fs::canonical(command::Path(argv[1]));
    const auto destination=command::Path(argv[3]);
    io::Require(fs::is_regular_file(config_path),"Job configuration must be a regular file");
    command::NewDirectory(destination);
    const auto config=command::ReadProvenance(config_path,expected_hash,remaining);
    const auto executable=command::CheckExecutable(config.parsed);
    io::Require(config.parsed.HasMember("raw")&&config.parsed["raw"].IsObject(),"Missing raw job binding object");
    const auto raw=command::ParseRawInput(config.parsed["raw"]);
    wall::RawReadBinding binding;
    binding.cells=raw.cells; binding.normal_velocity=raw.normal_velocity;
    binding.index_sha256=raw.index_sha256; binding.provenance_sha256=raw.provenance_sha256;
    binding.remaining_screen_byte_budget=raw.bytes;
    wall::RawReadResult read; std::string error;
    if(!wall::ReadRawJob(raw.directory,binding,read,error)) throw std::runtime_error(error);
    io::Require(read.receipt.total_bytes==binding.remaining_screen_byte_budget,"Raw receipt differs from exact configured byte count");
    std::cout<<"Observed executable: "<<executable.path<<" sha256="<<executable.sha256
             <<"; configuration_sha256="<<expected_hash<<"; raw_index_sha256="<<binding.index_sha256
             <<"; raw_bytes="<<read.receipt.total_bytes<<'\n'<<std::flush;
    wall::WallJobReportWriter writer(destination,read,binding,config.bytes,config_path.string(),remaining);
    const auto analysis=wall::AnalyzeWallRawJob(read.job,[&writer](const wall::WallJobAnalysis& state,wall::WallJobProgress event) {
      writer(state,event);
    });
    const auto& receipt=writer.receipt();
    io::Require(receipt.final_index_present&&receipt.analysis_complete==analysis.complete&&receipt.analysis_passed==analysis.passed,
      "Analysis and retained final index disagree");
    std::cout<<"Retained "<<receipt.files.size()<<" files, "<<receipt.total_bytes
             <<" bytes; analysis_complete="<<analysis.complete<<"; analysis_passed="<<analysis.passed
             <<"; screen_admitted=false; timestep_selected=false; trajectory_admitted=false\n";
    return analysis.complete?0:2;
  } catch(const std::exception& error) {
    std::cerr<<"Wall analysis failed; any prior artifacts are retained: "<<error.what()<<'\n';
    return 1;
  }
}
