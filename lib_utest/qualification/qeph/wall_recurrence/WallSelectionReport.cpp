#include "WallSelectionReport.h"
#include "WallCommandLine.h"
#include "WallRawJson.h"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <set>
#include <sstream>
#include <stdexcept>

namespace tl::qualification::qeph::wall_recurrence {
namespace io=crash::output;
namespace {
std::string HashOf(const std::vector<RawFileReceipt>& files,const char* name) {
  for(const auto& f:files) if(f.name==name) return f.sha256; return {};
}
std::size_t ReceiptBytes(const std::vector<RawFileReceipt>& files,unsigned cap) {
  io::Require(!files.empty()&&files.size()<=cap,"Unbounded selection input receipt");
  std::size_t total=0; std::set<std::string> names;
  for(const auto& f:files) {
    io::Require(!f.name.empty()&&f.name.size()<=64&&f.name!="."&&f.name!=".."&&
      f.name.find_first_not_of("abcdefghijklmnopqrstuvwxyz0123456789-.")==std::string::npos&&
      names.insert(f.name).second&&command::Hash(f.sha256)&&f.bytes>0&&f.bytes<=RawFileByteCap&&
      f.bytes<=ScreenSetByteCap-total,"Malformed selection input receipt");
    total+=f.bytes;
  }
  return total;
}
}
unsigned WallSelectionInputSlot(unsigned cells,double velocity) {
  io::Require((cells==1||cells==2)&&FrozenVelocity(velocity)&&!(velocity==0&&std::signbit(velocity)),
    "Invalid selection fixture/boost tuple");
  return 3*(cells-1)+(velocity==0?0:velocity<0?1:2);
}
std::size_t ValidateWallSelectionInputs(const std::array<WallSelectionInput,6>& inputs) {
  std::size_t total=0;
  for(unsigned i=0;i<6;++i) {
    const auto& p=inputs[i];
    io::Require(WallSelectionInputSlot(p.cells,p.normal_velocity)==i,"Selection bindings must cover the six canonical tuples exactly once");
    for(const auto& path:{p.raw_directory,p.derived_directory})
      io::Require(!path.empty()&&path.native().size()<=4096&&path.native().find('\0')==std::string::npos,"Invalid selection input path");
    for(const auto* hash:{&p.raw_index_sha256,&p.raw_provenance_sha256,&p.derived_index_sha256,&p.derived_provenance_sha256})
      io::Require(command::Hash(*hash),"Invalid selection input hash");
    for(const auto bytes:{p.raw_bytes,p.derived_bytes}) {
      io::Require(bytes>0&&bytes<=ScreenSetByteCap-total,"Selection inputs exceed the shared 96 MiB cap"); total+=bytes;
    }
  }
  return total;
}
WallSelectionReportWriter::WallSelectionReportWriter(const std::filesystem::path& path,
    const std::array<WallSelectionInput,6>& inputs,const std::string& provenance,
    const std::string& origin,std::size_t remaining):directory_(path),inputs_(inputs),provenance_origin_(origin) {
  input_bytes_=ValidateWallSelectionInputs(inputs_);
  io::Require(remaining>0&&remaining<=ScreenSetByteCap&&!origin.empty()&&origin.size()<=4096,"Invalid selection provenance/budget");
  budget_=std::min({remaining,WallSelectionReportByteCap,ScreenSetByteCap-input_bytes_});
  (void)ParseRawProvenance(provenance);
  io::Require(provenance.size()<budget_,"Selection provenance exhausts available report bytes");
  command::NewDirectory(directory_);
  io::Require(std::filesystem::create_directory(directory_),"Selection directory must be new");
  receipt_.files.reserve(24); provenance_hash_=Write("provenance.json",provenance).sha256;
  Inventory(false);
}
RawFileReceipt WallSelectionReportWriter::Write(const std::string& name,const std::string& bytes) {
  io::Require(!bytes.empty()&&bytes.size()<=RawFileByteCap&&bytes.size()<=budget_-receipt_.total_bytes&&
    receipt_.files.size()<24,"Selection artifact exceeds reserved file/set bytes or inventory capacity");
  for(const auto& f:receipt_.files) io::Require(f.name!=name,"Duplicate selection artifact");
  RawFileReceipt f{name,io::Sha256(bytes),bytes.size()}; io::WriteBytes(directory_/name,bytes);
  receipt_.files.push_back(f); receipt_.total_bytes+=bytes.size(); return f;
}
io::Document WallSelectionReportWriter::Header(const char* kind) const {
  io::Document d; d.SetObject(); io::String(d,"schema","robo-dyna-qeph-wall-selection-v1"); io::String(d,"kind",kind);
  io::String(d,"provenance_file","provenance.json"); io::String(d,"provenance_sha256",provenance_hash_);
  io::Integer(d,"provenance_bytes",receipt_.files.front().bytes); io::String(d,"provenance_origin",provenance_origin_);
  io::String(d,"trust_scope","Externally pinned config and reviewed producer/reader; no repeated native, Schur or Gram solve");
  io::String(d,"selection_rule","Six jobs and four signed-boost comparisons per h; unchanged SelectWallScreenStep factor-two margin; 4H0 diagnostic");
  io::Boolean(d,"trajectory_admitted",false); io::Boolean(d,"simulation_ready",false);
  io::Integer(d,"source_input_bytes",input_bytes_); io::Integer(d,"report_byte_cap",WallSelectionReportByteCap);
  io::Integer(d,"available_report_bytes",budget_); io::Integer(d,"shared_byte_cap",ScreenSetByteCap);
  return d;
}
void WallSelectionReportWriter::RecordJob(unsigned slot,const RawReadResult& raw,const WallDerivedReadResult& derived) {
  io::Require(!receipt_.final_index_present&&slot<6&&!jobs_seen_[slot],"Closed or duplicate selection job capture");
  const auto& expected=inputs_[slot]; const auto& r=raw.receipt; const auto& d=derived.receipt;
  io::Require(r.cells==expected.cells&&d.cells==expected.cells&&io::Bits(r.normal_velocity)==io::Bits(expected.normal_velocity)&&
    io::Bits(d.normal_velocity)==io::Bits(expected.normal_velocity)&&r.final_index_present&&d.final_index_present&&
    ReceiptBytes(r.files,80)==expected.raw_bytes&&r.total_bytes==expected.raw_bytes&&
    ReceiptBytes(d.files,128)==expected.derived_bytes&&d.total_bytes==expected.derived_bytes&&
    HashOf(r.files,"index.json")==expected.raw_index_sha256&&HashOf(r.files,"provenance.json")==expected.raw_provenance_sha256&&
    HashOf(d.files,"index.json")==expected.derived_index_sha256&&HashOf(d.files,"provenance.json")==expected.derived_provenance_sha256&&
    d.raw_index_sha256==expected.raw_index_sha256&&d.raw_provenance_sha256==expected.raw_provenance_sha256&&
    d.analysis_provenance_sha256==expected.derived_provenance_sha256,"Selection receipt differs from external input binding");
  if(derived.summary_available) {
    std::string error;
    if(!ValidateWallJobSummary(derived.summary,error)) throw std::runtime_error(error);
    io::Require(WallSelectionInputSlot(derived.summary.cells,derived.summary.normal_velocity)==slot,"Foreign selectable summary");
  }
  // Stage allocating receipt copies before updating captured state.
  auto staged_raw=r; auto staged_derived=d;
  raw_[slot]=std::move(staged_raw); derived_[slot]=std::move(staged_derived);
  summaries_available_[slot]=derived.summary_available;
  for(unsigned s=0;s<6;++s) job_pass_[slot][s]=derived.summary_available&&derived.summary.steps[s].complete&&derived.summary.steps[s].passed;
  jobs_seen_[slot]=true; Inventory(false);
}
void WallSelectionReportWriter::RecordBoost(const WallBoostComparison& comparison) {
  const auto job=WallSelectionInputSlot(comparison.cells,comparison.normal_velocity);
  io::Require(job%3!=0,"Zero is not a signed-boost comparison");
  const unsigned index=2*(comparison.cells-1)+job%3-1,zero=3*(comparison.cells-1);
  io::Require(!receipt_.final_index_present&&!boosts_seen_[index]&&jobs_seen_[zero]&&jobs_seen_[job],"Missing or duplicate boost input captures");
  io::Require(!comparison.input_valid||(summaries_available_[zero]&&summaries_available_[job]),"Available boost lacks validated summaries");
  auto d=Header("boost-comparison"); raw_detail::Field(d,"zero_input",selection_json::Input(inputs_[zero]));
  raw_detail::Field(d,"boost_input",selection_json::Input(inputs_[job]));
  raw_detail::Field(d,"comparison",selection_json::Boost(comparison,raw_[zero],raw_[job],derived_[zero],derived_[job]));
  Write("boost-c"+std::to_string(comparison.cells)+(comparison.normal_velocity<0?"-minus8.json":"-plus8.json"),EncodeRawJson(d));
  boosts_seen_[index]=true; boosts_available_[index]=comparison.input_valid;
  for(unsigned s=0;s<6;++s) boost_pass_[index][s]=comparison.input_valid&&comparison.steps[s].complete&&comparison.steps[s].passed;
  Inventory(false);
}
void WallSelectionReportWriter::Finish(const WallScreenSelection& selection) {
  io::Require(!receipt_.final_index_present,"Selection report is closed");
  for(bool seen:jobs_seen_) io::Require(seen,"Selection final lacks an authenticated job receipt");
  for(bool seen:boosts_seen_) io::Require(seen,"Selection final lacks a signed comparison outcome");
  if(selection.input_valid) {
    for(bool available:summaries_available_) io::Require(available,"Selection uses an unavailable summary");
    for(bool available:boosts_available_) io::Require(available,"Selection uses an unavailable comparison");
  }
  for(unsigned s=0;s<6;++s) {
    for(unsigned j=0;j<6;++j) io::Require(selection.steps[s].jobs[j]==job_pass_[j][s],"Selection changed a captured job verdict");
    for(unsigned b=0;b<4;++b) io::Require(selection.steps[s].boosts[b]==boost_pass_[b][s],"Selection changed a captured boost verdict");
  }
  std::size_t derived_bytes=0; for(const auto& d:derived_) derived_bytes+=d.total_bytes;
  io::Require(RawSetBytes(raw_,derived_bytes)==input_bytes_,"Actual selection inputs differ from declared aggregate bytes");
  auto d=Header("selection"); raw_detail::Field(d,"decision",selection_json::Selection(selection));
  Write("selection.json",EncodeRawJson(d)); Inventory(true,&selection);
}
void WallSelectionReportWriter::Inventory(bool final,const WallScreenSelection* selection) {
  auto d=Header(final?"final-index":"progress-index");
  io::Boolean(d,"final_index",final); io::Boolean(d,"decision_complete",final&&selection&&selection->input_valid);
  io::Boolean(d,"passed",final&&selection&&selection->passed);
  io::Number(d,"selected_h",final&&selection?selection->selected_h:0);
  io::Integer(d,"bytes_before_this_index",receipt_.total_bytes);
  io::String(d,"inventory_scope","All prior files, including progress; current index excluded from itself");
  io::Value inputs(rapidjson::kArrayType),boosts(rapidjson::kArrayType),files(rapidjson::kArrayType);
  for(unsigned j=0;j<6;++j) {
    auto p=selection_json::Input(inputs_[j]); io::Boolean(p,"authenticated",jobs_seen_[j]);
    io::Boolean(p,"summary_available",summaries_available_[j]);
    raw_detail::Append(d,inputs,p);
  }
  for(unsigned b=0;b<4;++b) {
    io::Document p; p.SetObject(); io::Boolean(p,"captured",boosts_seen_[b]); io::Boolean(p,"input_valid",boosts_available_[b]);
    io::Integer(p,"cells",b/2+1); io::Number(p,"normal_velocity_m_s",b%2?8.:-8.); raw_detail::Append(d,boosts,p);
  }
  for(const auto& file:receipt_.files) {
    io::Document f; f.SetObject(); io::String(f,"name",file.name); io::String(f,"sha256",file.sha256); io::Integer(f,"bytes",file.bytes);
    raw_detail::Append(d,files,f);
  }
  d.AddMember("inputs",inputs,d.GetAllocator()); d.AddMember("boosts",boosts,d.GetAllocator()); d.AddMember("files",files,d.GetAllocator());
  std::ostringstream name; if(final) name<<"index.json"; else name<<"progress-"<<std::setw(3)<<std::setfill('0')<<sequence_<<".json";
  Write(name.str(),EncodeRawJson(d));
  if(final) {
    receipt_.final_index_present=true; receipt_.decision_complete=selection&&selection->input_valid;
    receipt_.passed=selection&&selection->passed; receipt_.selected_h=selection?selection->selected_h:0;
  } else ++sequence_;
}
} // namespace tl::qualification::qeph::wall_recurrence
