#pragma once
#include "WallResponseJson.h"
#include "../wall_recurrence/WallCommandLine.h"

namespace tl::qualification::qeph::wall_response::command {
namespace shared=wr::command;
namespace p=json::p;
namespace fs=std::filesystem;
constexpr std::size_t ProvenanceCap=1024*1024,SelectionCap=2*1024*1024;
struct Launch {
  std::string provenance_bytes;
  ReportBinding binding;
  double selected_h=0;
};
// This verifies the exact root-provided launch bytes, running image, selection
// index and every payload named by that index. Issuer/source review is the
// external launcher's responsibility, exactly as for the incoming prefix.
inline Launch ReadLaunch(const fs::path& file,const std::string& expected_sha256) {
  io::Require(p::Hash(expected_sha256),"Expected provenance SHA256 must be lowercase hexadecimal");
  Launch result; result.provenance_bytes=io::ReadBounded(file,ProvenanceCap);
  io::Require(io::Sha256(result.provenance_bytes)==expected_sha256,"Launch provenance SHA256 mismatch");
  const auto source=p::Parse(result.provenance_bytes);
  io::Require(p::String(source,"schema")=="robo-dyna.qeph-wall-response-launch.v1","Foreign wall-response launch schema");
  const auto executable=shared::CheckExecutable(source);
  const auto& screen=p::Member(source,"screen");
  io::Require(screen.IsObject()&&screen.MemberCount()==3,"Screen binding requires index path, exact hash and selected h");
  const auto index_path=shared::Path(shared::Text(screen,"index_path"));
  const auto index_sha=shared::Text(screen,"index_sha256"); io::Require(p::Hash(index_sha),"Invalid wall-screen hash");
  result.selected_h=p::Number(screen,"selected_h");
  io::Require(result.selected_h==H0||result.selected_h==H0/2,"Wall screen did not select a supported step");
  io::Require(fs::is_regular_file(fs::symlink_status(index_path)),"Screen index must be a regular file");
  const auto bytes=io::ReadBounded(index_path,SelectionCap); io::Require(io::Sha256(bytes)==index_sha,"Wall-screen index SHA256 mismatch");
  const auto index=p::Parse(bytes);
  io::Require(p::String(index,"schema")=="robo-dyna-qeph-wall-selection-v1"&&p::String(index,"kind")=="final-index"&&
    p::Boolean(index,"final_index")&&p::Boolean(index,"decision_complete")&&p::Boolean(index,"passed")&&
    !p::Boolean(index,"trajectory_admitted")&&!p::Boolean(index,"simulation_ready")&&
    p::Number(index,"selected_h")==result.selected_h,"Incomplete, rejected or changed wall-screen decision");
  const auto& files=p::Member(index,"files"); io::Require(files.IsArray()&&!files.Empty()&&files.Size()<=32,"Unbounded selection inventory");
  std::size_t total=bytes.size(); bool selection_seen=false;
  for(const auto& f:files.GetArray()) {
    const auto name=shared::Text(f,"name");
    io::Require(name.size()<=128&&name!="."&&name!=".."&&name.find_first_of("/\\")==std::string::npos,"Invalid selection payload name");
    const auto hash=shared::Text(f,"sha256"); io::Require(p::Hash(hash),"Invalid selection payload hash");
    const auto path=index_path.parent_path()/name; io::Require(fs::is_regular_file(fs::symlink_status(path)),"Missing selection payload");
    const auto payload=io::ReadBounded(path,SelectionCap);
    io::Require(payload.size()==p::Integer(f,"bytes")&&io::Sha256(payload)==hash,"Selection payload binding mismatch");
    io::Require(payload.size()<=SelectionCap-total,"Selection archive exceeds bounded bytes"); total+=payload.size();
    if(name=="selection.json") {
      io::Require(!selection_seen,"Duplicate selection payload"); selection_seen=true;
      const auto decision=p::Parse(payload); const auto& d=p::Member(decision,"decision");
      io::Require(p::Boolean(d,"input_valid")&&p::Boolean(d,"passed")&&p::Number(d,"selected_h")==result.selected_h,"Changed final selector result");
    }
  }
  io::Require(selection_seen,"Missing final selection payload");
  result.binding={index_sha,executable.sha256,expected_sha256}; return result;
}
inline void WriteRunRecord(const fs::path& directory,const Run& run,const Launch& launch) {
  io::WriteBytes(directory/"run.json",EncodeReport(DescribeRun(run,launch.binding,launch.provenance_bytes)));
}
} // namespace tl::qualification::qeph::wall_response::command
