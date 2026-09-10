#include "WallJobReport.h"
#include "WallJobJson.h"
#include <cmath>
#include <set>

namespace tl::qualification::qeph::wall_recurrence {
namespace io=crash::output;
namespace {
bool Hash(const std::string& s) { return s.size()==64&&s.find_first_not_of("0123456789abcdef")==std::string::npos; }
}
WallJobReportWriter::WallJobReportWriter(const std::filesystem::path& directory,const RawReadResult& raw,
    const RawReadBinding& binding,const std::string& provenance,const std::string& origin,std::size_t budget)
    :raw_(raw),binding_(binding),directory_(directory),provenance_origin_(origin),budget_(budget) {
  const auto& job=raw.job; const auto& receipt=raw.receipt;
  io::Require((binding.cells==1||binding.cells==2)&&FrozenVelocity(binding.normal_velocity)&&
    !(binding.normal_velocity==0&&std::signbit(binding.normal_velocity))&&Hash(binding.index_sha256)&&Hash(binding.provenance_sha256)&&
    job.cells==binding.cells&&io::Bits(job.normal_velocity)==io::Bits(binding.normal_velocity)&&
    receipt.cells==job.cells&&io::Bits(receipt.normal_velocity)==io::Bits(job.normal_velocity)&&receipt.final_index_present&&
    receipt.collection_complete==job.collection_complete,"Invalid derived raw identity/binding");
  io::Require(!receipt.files.empty()&&receipt.files.size()<=80&&budget>0&&budget<=ScreenSetByteCap&&
    !origin.empty()&&origin.size()<=4096,"Unbounded derived receipt/provenance/budget");
  std::size_t total=0; std::set<std::string> names;
  for(const auto& f:receipt.files) {
    io::Require(!f.name.empty()&&f.name.size()<=64&&f.name!="."&&f.name!=".."&&
      f.name.find_first_not_of("abcdefghijklmnopqrstuvwxyz0123456789-.")==std::string::npos&&
      Hash(f.sha256)&&names.insert(f.name).second&&f.bytes>0&&f.bytes<=RawFileByteCap&&
      f.bytes<=ScreenSetByteCap-total,"Malformed bound raw receipt"); total+=f.bytes;
  }
  io::Require(total==receipt.total_bytes&&RawHash("index.json")==binding.index_sha256&&
    RawHash("provenance.json")==binding.provenance_sha256,"Derived raw index/provenance receipt mismatch");
  for(unsigned s=0;s<6;++s) io::Require(job.steps[s].h==recurrence::Steps[s],"Changed derived raw step grid");
  (void)ParseRawProvenance(provenance); io::Require(provenance.size()<budget,"Analysis provenance exhausts remaining bytes");
  io::Require(!std::filesystem::exists(directory)&&std::filesystem::create_directory(directory),"Derived directory must be new");
  receipt_.cells=binding.cells; receipt_.normal_velocity=binding.normal_velocity;
  receipt_.raw_index_sha256=binding.index_sha256; receipt_.raw_provenance_sha256=binding.provenance_sha256;
  receipt_.files.reserve(128); receipt_.analysis_provenance_sha256=Write("provenance.json",provenance).sha256;
  Inventory(nullptr,false);
}
std::string WallJobReportWriter::RawHash(const std::string& name) const {
  for(const auto& f:raw_.receipt.files) if(f.name==name) return f.sha256; return {};
}
void WallJobReportWriter::RawLink(io::Document& d,const char* label,const std::string& name) const {
  io::Document link; link.SetObject(); const auto hash=RawHash(name);
  io::Boolean(link,"present",!hash.empty()); io::String(link,"file",name);
  if(!hash.empty()) io::String(link,"sha256",hash); raw_detail::Field(d,label,link);
}
RawFileReceipt WallJobReportWriter::Write(const std::string& name,const std::string& bytes) {
  io::Require(!bytes.empty()&&bytes.size()<=RawFileByteCap&&bytes.size()<=budget_-receipt_.total_bytes,
    "Derived artifact exceeds file or remaining shared byte budget");
  io::Require(receipt_.files.size()<128,"Derived file inventory capacity exceeded");
  for(const auto& f:receipt_.files) io::Require(f.name!=name,"Duplicate derived file");
  RawFileReceipt file{name,io::Sha256(bytes),bytes.size()}; io::WriteBytes(directory_/name,bytes);
  receipt_.files.push_back(file); receipt_.total_bytes+=bytes.size(); return file;
}
io::Document WallJobReportWriter::Header(const char* kind) const {
  io::Document d; d.SetObject(); io::String(d,"schema","robo-dyna-qeph-wall-derived-v1"); io::String(d,"kind",kind);
  io::Integer(d,"cells",binding_.cells); io::Number(d,"normal_velocity_m_s",binding_.normal_velocity);
  io::Integer(d,"normal_velocity_binary64_bits",io::Bits(binding_.normal_velocity));
  io::String(d,"raw_index_sha256",binding_.index_sha256); io::String(d,"raw_provenance_sha256",binding_.provenance_sha256);
  RawLink(d,"raw_model","model.json"); io::String(d,"analysis_provenance_file","provenance.json");
  io::String(d,"analysis_provenance_sha256",receipt_.analysis_provenance_sha256);
  io::Integer(d,"analysis_provenance_bytes",receipt_.files.front().bytes); io::String(d,"analysis_provenance_origin",provenance_origin_);
  io::String(d,"authentication_scope","Externally authenticated raw reader result and exact analysis provenance; listed source/build trust remains external");
  io::Boolean(d,"simulation_ready",false); io::Boolean(d,"six_job_screen_decision_included",false);
  io::String(d,"qualification_scope","One prescribed native/contact recurrence analysis; no startup or trajectory admission");
  io::Number(d,"matrix_tolerance",recurrence::MatrixTolerance); io::Number(d,"decomposition_tolerance",recurrence::DecompositionTolerance);
  io::Number(d,"weighted_mean_gain_limit",recurrence::MaximumGramGain);
  io::Number(d,"gain_relative_tolerance",WallGainRelativeTolerance); io::Number(d,"gain_absolute_tolerance",WallGainAbsoluteTolerance);
  return d;
}
} // namespace tl::qualification::qeph::wall_recurrence
