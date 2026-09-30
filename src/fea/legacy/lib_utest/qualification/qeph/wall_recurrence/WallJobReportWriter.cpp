#include "WallJobReport.h"
#include "WallJobJson.h"

namespace tl::qualification::qeph::wall_recurrence {
namespace io=crash::output;
void WallJobReportWriter::Context(const WallJobAnalysis& job,unsigned step,unsigned amplitude) {
  const auto& a=job.steps[step].amplitudes[amplitude];
  if(!a.input_complete) return;
  const auto body=job_json::Context(a.branches,12*2*(binding_.cells+1)+61*binding_.cells);
  const auto hash=io::Sha256(EncodeRawJson(body));
  if(!context_hash_[step].empty()) {
    io::Require(hash==context_data_hash_[step],"Metric/scalar schedule changed between amplitudes"); return;
  }
  auto d=Header("context"); io::Integer(d,"step_index",step); raw_detail::Field(d,"context",body);
  const auto file=Write("context-h"+std::to_string(step)+".json",EncodeRawJson(d));
  context_hash_[step]=file.sha256; context_data_hash_[step]=hash;
  // This context precedes publication of the triggering amplitude. Preserve
  // it immediately without claiming that amplitude has already been captured.
  Inventory(nullptr,false);
}
void WallJobReportWriter::operator()(const WallJobAnalysis& job,WallJobProgress event) {
  const unsigned dimension=12*2*(binding_.cells+1)+61*binding_.cells;
  io::Require(!receipt_.final_index_present&&job.cells==binding_.cells&&
    io::Bits(job.normal_velocity)==io::Bits(binding_.normal_velocity)&&
    ((job.input_valid&&job.dimension==dimension)||(!job.input_valid&&job.dimension==0)),"Closed derived writer or foreign analysis");
  io::Require(job.diagnostic.size()<=4096,"Unbounded derived job diagnostic");
  for(unsigned s=0;s<6;++s) {
    io::Require(job.steps[s].h==recurrence::Steps[s],"Changed derived step grid");
    for(unsigned a=0;a<3;++a) io::Require(job.steps[s].amplitudes[a].amplitude==recurrence::Amplitudes[a],"Changed derived amplitude grid");
  }
  ValidateCaptured(job);
  if(event.kind==WallJobProgressKind::Finished) { Inventory(&job,true); return; }
  io::Require(job.input_valid&&event.step<6,"Invalid derived callback step/input");
  const auto& step=job.steps[event.step]; auto d=Header(event.kind==WallJobProgressKind::Amplitude?"amplitude":
    (event.kind==WallJobProgressKind::Contact?"contact-recheck":"step"));
  io::Integer(d,"step_index",event.step); io::Number(d,"fixed_dt_s",step.h);
  std::string name;
  if(event.kind==WallJobProgressKind::Amplitude) {
    io::Require(event.index<3&&!amplitude_seen_[event.step][event.index]&&!step_seen_[event.step],"Duplicate derived amplitude");
    const auto& a=step.amplitudes[event.index];
    const auto& source=raw_.job.steps[event.step].native[event.index];
    io::Require(a.attempted==raw_.job.steps[event.step].native_attempted[event.index]&&
      a.input_complete==(a.attempted&&source.baseline_complete&&source.derivative.complete),
      "Derived amplitude raw-input association mismatch");
    const auto body=job_json::Amplitude(a,raw_.job,event.step,event.index);
    Context(job,event.step,event.index); io::Integer(d,"amplitude_index",event.index);
    RawLink(d,"raw_native_matrix","native-h"+std::to_string(event.step)+"-a"+std::to_string(event.index)+".json");
    raw_detail::Field(d,"analysis",body); name="amplitude-h"+std::to_string(event.step)+"-a"+std::to_string(event.index)+".json";
    io::Boolean(d,"context_available",!context_hash_[event.step].empty());
    if(!context_hash_[event.step].empty()) {
      io::String(d,"context_file","context-h"+std::to_string(event.step)+".json"); io::String(d,"context_sha256",context_hash_[event.step]);
    }
    Write(name,EncodeRawJson(d)); amplitude_seen_[event.step][event.index]=true;
    amplitude_complete_[event.step][event.index]=a.complete; amplitude_passed_[event.step][event.index]=a.passed;
  } else if(event.kind==WallJobProgressKind::Contact) {
    io::Require(event.index<2&&!contact_seen_[event.step][event.index]&&!step_seen_[event.step],"Duplicate derived contact recheck");
    for(bool seen:amplitude_seen_[event.step]) io::Require(seen,"Contact recheck precedes amplitude captures");
    const auto& c=step.contact[event.index]; io::Require(c.branch==(event.index?ContactBranch::Active:ContactBranch::Inactive),"Changed recomputed contact role");
    io::Integer(d,"branch_index",event.index);
    RawLink(d,"raw_contact_branch","contact-h"+std::to_string(event.step)+"-b"+std::to_string(event.index)+".json");
    RawLink(d,"raw_native_matrix","native-h"+std::to_string(event.step)+"-a2.json");
    raw_detail::Field(d,"analysis",job_json::Contact(c,dimension,2*(binding_.cells+1)));
    Write("contact-h"+std::to_string(event.step)+"-b"+std::to_string(event.index)+".json",EncodeRawJson(d));
    contact_seen_[event.step][event.index]=true; contact_complete_[event.step][event.index]=c.complete;
    contact_passed_[event.step][event.index]=c.passed;
  } else if(event.kind==WallJobProgressKind::Step) {
    io::Require(!step_seen_[event.step],"Duplicate derived step");
    for(bool seen:amplitude_seen_[event.step]) io::Require(seen,"Step precedes amplitude capture");
    for(bool seen:contact_seen_[event.step]) io::Require(seen,"Step precedes contact capture");
    auto link=[&](io::Value& array,const std::string& file_name) {
      io::Document p; p.SetObject(); io::String(p,"file",file_name); std::string hash;
      for(const auto& file:receipt_.files) if(file.name==file_name) hash=file.sha256;
      io::Require(!hash.empty(),"Missing prior derived step operand"); io::String(p,"sha256",hash); raw_detail::Append(d,array,p);
    };
    io::Value amplitudes(rapidjson::kArrayType),contacts(rapidjson::kArrayType);
    for(unsigned a=0;a<3;++a) link(amplitudes,"amplitude-h"+std::to_string(event.step)+"-a"+std::to_string(a)+".json");
    for(unsigned b=0;b<2;++b) link(contacts,"contact-h"+std::to_string(event.step)+"-b"+std::to_string(b)+".json");
    d.AddMember("derived_amplitudes",amplitudes,d.GetAllocator()); d.AddMember("derived_contact",contacts,d.GetAllocator());
    raw_detail::Field(d,"analysis",job_json::Step(step));
    Write("step-h"+std::to_string(event.step)+".json",EncodeRawJson(d));
    step_seen_[event.step]=true; step_complete_[event.step]=step.complete; step_passed_[event.step]=step.passed;
  } else io::Require(false,"Unknown derived callback kind");
  Inventory(&job,false);
}
} // namespace tl::qualification::qeph::wall_recurrence
