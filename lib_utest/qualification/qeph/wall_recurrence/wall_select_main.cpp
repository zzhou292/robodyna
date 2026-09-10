#include "WallCommandLine.h"
#include "WallSelectionReport.h"
#include <algorithm>
#include <iostream>
#include <stdexcept>

namespace wall=tl::qualification::qeph::wall_recurrence;
namespace recurrence=tl::qualification::qeph::recurrence;
namespace command=wall::command;
namespace io=crash::output;
namespace fs=std::filesystem;
namespace {
std::array<wall::WallSelectionInput,6> Inputs(const io::Document& config,const fs::path& destination) {
  io::Require(config.HasMember("jobs")&&config["jobs"].IsArray()&&config["jobs"].Size()==6,"Selection requires exactly six job bindings");
  std::array<wall::WallSelectionInput,6> result{}; std::array<bool,6> seen{};
  const auto parent=destination.has_parent_path()?destination.parent_path():fs::path(".");
  const auto target=(fs::canonical(parent)/destination.filename()).lexically_normal();
  for(const auto& job:config["jobs"].GetArray()) {
    io::Require(job.IsObject()&&job.MemberCount()==2&&job.HasMember("raw")&&job.HasMember("derived"),"Each selection job requires raw and derived bindings");
    const auto raw=command::ParseRawInput(job["raw"]); const auto& derived=job["derived"];
    io::Require(derived.IsObject()&&derived.MemberCount()==4&&derived.HasMember("directory")&&derived.HasMember("index_sha256")&&
      derived.HasMember("provenance_sha256")&&derived.HasMember("bytes"),"Derived binding requires exactly four fields");
    io::Require(derived["bytes"].IsUint64()&&derived["bytes"].GetUint64()>0&&derived["bytes"].GetUint64()<=wall::ScreenSetByteCap,
      "Derived bytes must be a positive bounded JSON unsigned integer");
    const auto slot=wall::WallSelectionInputSlot(raw.cells,raw.normal_velocity);
    io::Require(!seen[slot],"Duplicate selection job tuple"); seen[slot]=true;
    auto& p=result[slot]; p.cells=raw.cells; p.normal_velocity=raw.normal_velocity;
    p.raw_directory=raw.directory; p.raw_index_sha256=raw.index_sha256; p.raw_provenance_sha256=raw.provenance_sha256; p.raw_bytes=raw.bytes;
    const auto path=command::Path(command::Text(derived,"directory"));
    io::Require(fs::is_directory(fs::symlink_status(path)),"Derived input must be a real directory"); p.derived_directory=fs::canonical(path);
    p.derived_index_sha256=command::Text(derived,"index_sha256"); p.derived_provenance_sha256=command::Text(derived,"provenance_sha256");
    p.derived_bytes=static_cast<std::size_t>(derived["bytes"].GetUint64());
    // Creating an output below an input would alter that authenticated closed
    // inventory before its streaming read. Keep output outside every input.
    for(const auto& source:{p.raw_directory,p.derived_directory})
      io::Require(std::mismatch(source.begin(),source.end(),target.begin(),target.end()).first!=source.end(),
        "Selection output cannot be inside an input archive");
  }
  (void)wall::ValidateWallSelectionInputs(result); return result;
}
wall::WallBoostComparison Unavailable(unsigned cells,double velocity) {
  wall::WallBoostComparison b; b.cells=cells; b.dimension=12*2*(cells+1)+61*cells; b.normal_velocity=velocity;
  for(unsigned s=0;s<6;++s) b.steps[s].h=recurrence::Steps[s];
  b.diagnostic="A validated selectable zero or signed-boost summary is unavailable"; return b;
}
}
// Linux qualification command: one raw/derived job at a time, one immutable
// zero cache per fixture. Source/producer trust is pinned by the root launcher.
int main(int argc,char** argv) {
  try {
    io::Require(argc==5,"Usage: qeph_wall_select JOB_SET_CONFIG_JSON EXPECTED_CONFIG_SHA256 NEW_DIRECTORY REMAINING_SET_BYTES");
    const std::string_view expected_hash=argv[2]; io::Require(command::Hash(expected_hash),"Invalid expected selection configuration SHA256");
    const auto remaining=command::Budget(argv[4]); const auto path=fs::canonical(command::Path(argv[1]));
    io::Require(fs::is_regular_file(path),"Selection configuration must be a regular file");
    const auto destination=command::Path(argv[3]); command::NewDirectory(destination);
    const auto provenance=command::ReadProvenance(path,expected_hash,remaining);
    const auto executable=command::CheckExecutable(provenance.parsed);
    const auto inputs=Inputs(provenance.parsed,destination);
    std::cout<<"Observed executable: "<<executable.path<<" sha256="<<executable.sha256
             <<"; configuration_sha256="<<expected_hash<<'\n'<<std::flush;
    wall::WallSelectionReportWriter writer(destination,inputs,provenance.bytes,path.string(),remaining);
    std::array<wall::WallJobSummary,6> summaries{}; std::array<bool,6> available{};
    std::array<wall::WallBoostComparison,4> comparisons{};
    for(unsigned cells=1;cells<=2;++cells) {
      wall::WallZeroBoostReference zero;
      for(unsigned sign=0;sign<3;++sign) {
        const unsigned slot=3*(cells-1)+sign; const auto& input=inputs[slot];
        const wall::RawReadBinding raw_binding{cells,input.normal_velocity,input.raw_provenance_sha256,input.raw_index_sha256,input.raw_bytes};
        wall::RawReadResult raw; std::string error;
        if(!wall::ReadRawJob(input.raw_directory,raw_binding,raw,error)) throw std::runtime_error(error);
        io::Require(raw.receipt.total_bytes==input.raw_bytes,"Raw selection receipt differs from exact configured bytes");
        const wall::WallDerivedReadBinding derived_binding{input.derived_index_sha256,input.derived_provenance_sha256,input.derived_bytes};
        wall::WallDerivedReadResult derived;
        if(!wall::ReadWallJobSummary(input.derived_directory,raw,derived_binding,derived,error)) throw std::runtime_error(error);
        io::Require(derived.receipt.total_bytes==input.derived_bytes,"Derived selection receipt differs from exact configured bytes");
        writer.RecordJob(slot,raw,derived); available[slot]=derived.summary_available;
        if(available[slot]) summaries[slot]=derived.summary;
        if(sign==0) {
          if(available[slot]&&!wall::PrepareWallZeroBoostReference(raw.job,derived.summary,zero,error)) throw std::runtime_error(error);
        } else {
          auto comparison=Unavailable(cells,input.normal_velocity);
          if(zero.prepared()&&available[slot]) comparison=wall::CompareWallBoost(zero,raw.job,derived.summary);
          writer.RecordBoost(comparison); comparisons[2*(cells-1)+sign-1]=std::move(comparison);
        }
        // The raw tree and temporary reader state are released here.
      }
    }
    auto selection=wall::SelectWallScreen(summaries,comparisons);
    if(!selection.input_valid) for(unsigned s=0;s<6;++s) {
      // Preserve available contributors as diagnostics without inventing a
      // selection from a partial set or changing the qualified selector.
      for(unsigned j=0;j<6;++j) selection.steps[s].jobs[j]=available[j]&&summaries[j].steps[s].complete&&summaries[j].steps[s].passed;
      for(unsigned b=0;b<4;++b) selection.steps[s].boosts[b]=comparisons[b].input_valid&&comparisons[b].steps[s].complete&&comparisons[b].steps[s].passed;
    }
    writer.Finish(selection); const auto& receipt=writer.receipt();
    io::Require(receipt.final_index_present&&receipt.decision_complete==selection.input_valid&&receipt.selected_h==selection.selected_h&&
      receipt.passed==selection.passed,"Selection and final retained index disagree");
    std::cout<<"Retained "<<receipt.files.size()<<" files, "<<receipt.total_bytes<<" bytes; decision_complete="<<receipt.decision_complete
             <<"; passed="<<receipt.passed<<"; selected_h="<<receipt.selected_h<<"; trajectory_admitted=false\n";
    return receipt.decision_complete?0:2;
  } catch(const std::exception& error) {
    std::cerr<<"Wall selection failed; any prior artifacts are retained: "<<error.what()<<'\n'; return 1;
  }
}
