#include "WallJobReport.h"
#include "WallJobJson.h"
#include <iomanip>
#include <sstream>

namespace tl::qualification::qeph::wall_recurrence {
namespace io=crash::output;
void WallJobReportWriter::ValidateCaptured(const WallJobAnalysis& job) const {
  for(unsigned s=0;s<6;++s) {
    for(unsigned a=0;a<3;++a) if(amplitude_seen_[s][a])
      io::Require(job.steps[s].amplitudes[a].complete==amplitude_complete_[s][a]&&
        job.steps[s].amplitudes[a].passed==amplitude_passed_[s][a],"Derived amplitude status changed after capture");
    for(unsigned b=0;b<2;++b) if(contact_seen_[s][b])
      io::Require(job.steps[s].contact[b].complete==contact_complete_[s][b]&&
        job.steps[s].contact[b].passed==contact_passed_[s][b],"Derived contact status changed after capture");
    if(step_seen_[s]) io::Require(job.steps[s].complete==step_complete_[s]&&job.steps[s].passed==step_passed_[s],
      "Derived step status changed after capture");
  }
}
void WallJobReportWriter::Inventory(const WallJobAnalysis* job,bool final) {
  if(job) ValidateCaptured(*job);
  auto d=Header(final?"final-index":"progress-index");
  io::Boolean(d,"final_index",final); io::Boolean(d,"analysis_complete",final&&job&&job->complete);
  io::Boolean(d,"analysis_passed",final&&job&&job->passed);
  io::Integer(d,"bytes_before_this_index",receipt_.total_bytes); io::Integer(d,"remaining_set_budget_at_job_start",budget_);
  io::Integer(d,"file_byte_cap",RawFileByteCap); io::Integer(d,"shared_raw_and_derived_byte_cap",ScreenSetByteCap);
  io::String(d,"inventory_scope","All prior files including progress; current index is excluded from its own inventory");
  io::String(d,"selection_scope","Per-step results are inputs to future six-job selection; aggregate job.passed includes diagnostic 4H0");
  unsigned complete_amplitudes=0,complete_contacts=0,complete_steps=0;
  bool all_steps=true,all_passed=true; io::Value steps(rapidjson::kArrayType);
  for(unsigned s=0;s<6;++s) {
    io::Document step; step.SetObject(); io::Number(step,"fixed_dt_s",recurrence::Steps[s]);
    io::Boolean(step,"context_available",!context_hash_[s].empty());
    if(!context_hash_[s].empty()) { io::String(step,"context_file","context-h"+std::to_string(s)+".json"); io::String(step,"context_sha256",context_hash_[s]); }
    io::Value amplitudes(rapidjson::kArrayType),contacts(rapidjson::kArrayType);
    for(unsigned a=0;a<3;++a) {
      const bool seen=amplitude_seen_[s][a];
      complete_amplitudes+=seen&&amplitude_complete_[s][a];
      io::Document p; p.SetObject(); io::Boolean(p,"captured",seen); io::Boolean(p,"complete",seen&&amplitude_complete_[s][a]);
      io::Boolean(p,"passed",seen&&amplitude_passed_[s][a]);
      if(seen) io::String(p,"file","amplitude-h"+std::to_string(s)+"-a"+std::to_string(a)+".json");
      raw_detail::Append(step,amplitudes,p);
    }
    for(unsigned b=0;b<2;++b) {
      const bool seen=contact_seen_[s][b];
      complete_contacts+=seen&&contact_complete_[s][b];
      io::Document p; p.SetObject(); io::Boolean(p,"captured",seen); io::Boolean(p,"complete",seen&&contact_complete_[s][b]);
      io::Boolean(p,"passed",seen&&contact_passed_[s][b]);
      if(seen) io::String(p,"file","contact-h"+std::to_string(s)+"-b"+std::to_string(b)+".json");
      raw_detail::Append(step,contacts,p);
    }
    complete_steps+=step_seen_[s]&&step_complete_[s]; all_steps=all_steps&&step_seen_[s]&&step_complete_[s];
    all_passed=all_passed&&step_seen_[s]&&step_passed_[s];
    io::Boolean(step,"captured",step_seen_[s]); io::Boolean(step,"complete",step_seen_[s]&&step_complete_[s]);
    io::Boolean(step,"passed",step_seen_[s]&&step_passed_[s]);
    if(step_seen_[s]) io::String(step,"file","step-h"+std::to_string(s)+".json");
    step.AddMember("amplitudes",amplitudes,step.GetAllocator()); step.AddMember("contact",contacts,step.GetAllocator());
    raw_detail::Append(d,steps,step);
  }
  d.AddMember("steps",steps,d.GetAllocator());
  io::Integer(d,"completed_amplitudes",complete_amplitudes); io::Integer(d,"completed_contacts",complete_contacts);
  io::Integer(d,"completed_steps",complete_steps);
  if(job) {
    io::Require(job->completed_amplitudes==complete_amplitudes&&job->completed_contacts==complete_contacts&&
      job->completed_steps==complete_steps,"Derived counters disagree with captured evidence");
    io::Require(!job->passed||job->complete,"Passed job is incomplete");
    if(final) io::Require((!job->complete||all_steps)&&(!job->passed||all_passed),"False completed/passed derived job");
    io::Boolean(d,"input_valid",job->input_valid); io::String(d,"diagnostic",job->diagnostic);
  }
  io::Value files(rapidjson::kArrayType);
  for(const auto& file:receipt_.files) {
    io::Document f; f.SetObject(); io::String(f,"name",file.name); io::String(f,"sha256",file.sha256); io::Integer(f,"bytes",file.bytes);
    raw_detail::Append(d,files,f);
  }
  d.AddMember("files",files,d.GetAllocator()); std::ostringstream name;
  if(final) name<<"index.json";
  else name<<"progress-"<<std::setw(3)<<std::setfill('0')<<sequence_<<".json";
  Write(name.str(),EncodeRawJson(d));
  if(final) {
    receipt_.final_index_present=true; receipt_.analysis_complete=job&&job->complete; receipt_.analysis_passed=job&&job->passed;
  } else ++sequence_;
}
} // namespace tl::qualification::qeph::wall_recurrence
