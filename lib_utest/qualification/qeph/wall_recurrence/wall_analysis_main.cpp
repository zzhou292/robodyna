#include "WallCommandLine.h"
#include "WallJobReport.h"
#include <cmath>
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
    const auto& raw=config.parsed["raw"];
    io::Require(raw.MemberCount()==6&&raw.HasMember("directory")&&raw.HasMember("cells")&&
      raw.HasMember("normal_velocity_m_s")&&raw.HasMember("index_sha256")&&
      raw.HasMember("provenance_sha256")&&raw.HasMember("bytes"),"Raw binding requires exactly its declared fields");
    io::Require(raw["cells"].IsUint()&&(raw["cells"].GetUint()==1||raw["cells"].GetUint()==2),
      "Raw cells must be the JSON unsigned integer 1 or 2");
    io::Require(raw["normal_velocity_m_s"].IsDouble(),"Raw normal velocity must be a JSON double -8.0, 0.0 or 8.0");
    const auto velocity=raw["normal_velocity_m_s"].GetDouble();
    io::Require(std::isfinite(velocity)&&wall::FrozenVelocity(velocity)&&!(velocity==0&&std::signbit(velocity)),
      "Raw normal velocity is outside the frozen tuple or is negative zero");
    io::Require(raw["bytes"].IsUint64()&&raw["bytes"].GetUint64()>0&&raw["bytes"].GetUint64()<=wall::ScreenSetByteCap,
      "Raw bytes must be a positive bounded JSON unsigned integer");
    wall::RawReadBinding binding;
    binding.cells=raw["cells"].GetUint(); binding.normal_velocity=velocity;
    binding.index_sha256=command::Text(raw,"index_sha256");
    binding.provenance_sha256=command::Text(raw,"provenance_sha256");
    io::Require(command::Hash(binding.index_sha256)&&command::Hash(binding.provenance_sha256),"Raw hashes must be 64 lowercase hexadecimal characters");
    binding.remaining_screen_byte_budget=static_cast<std::size_t>(raw["bytes"].GetUint64());
    const auto raw_path=command::Path(command::Text(raw,"directory"));
    io::Require(fs::is_directory(fs::symlink_status(raw_path)),"Raw input must be a real directory");
    const auto raw_directory=fs::canonical(raw_path);
    wall::RawReadResult read; std::string error;
    if(!wall::ReadRawJob(raw_directory,binding,read,error)) throw std::runtime_error(error);
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
