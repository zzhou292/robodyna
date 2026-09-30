#include "WallDerivedReadFields.h"

namespace tl::qualification::qeph::wall_recurrence::derived_read {
void Inventory(const io::Document& d,const State& state,bool final) {
  io::Require(rd::Boolean(rd::Field(d,"final_index"))==final,"Changed derived final-index flag");
  bool all_complete=true,all_passed=true;
  for(unsigned s=0;s<6;++s) {
    all_complete=all_complete&&state.step_seen[s]&&state.analysis.steps[s].complete;
    all_passed=all_passed&&state.step_seen[s]&&state.analysis.steps[s].passed;
  }
  io::Require(rd::Boolean(rd::Field(d,"analysis_complete"))==(final&&all_complete)&&
    rd::Boolean(rd::Field(d,"analysis_passed"))==(final&&all_complete&&all_passed),"Changed aggregate analysis status");
  io::Require(rd::Unsigned(rd::Field(d,"bytes_before_this_index"))==state.receipt.total_bytes&&
    rd::Unsigned(rd::Field(d,"remaining_set_budget_at_job_start"))==state.producer_budget&&
    rd::Unsigned(rd::Field(d,"file_byte_cap"))==RawFileByteCap&&
    rd::Unsigned(rd::Field(d,"shared_raw_and_derived_byte_cap"))==ScreenSetByteCap,"Changed derived byte/prefix contract");
  io::Require(rd::Text(rd::Field(d,"inventory_scope"))=="All prior files including progress; current index is excluded from its own inventory"&&
    rd::Text(rd::Field(d,"selection_scope"))=="Per-step results are inputs to future six-job selection; aggregate job.passed includes diagnostic 4H0",
    "Changed derived inventory/selection scope");
  const auto& summaries=rd::Field(d,"steps"); io::Require(summaries.IsArray()&&summaries.Size()==6,"Changed derived grid count");
  for(unsigned s=0;s<6;++s) {
    const auto& saved=summaries[s]; const auto& actual=state.analysis.steps[s];
    io::Document expected; expected.SetObject(); io::Number(expected,"fixed_dt_s",recurrence::Steps[s]);
    io::Boolean(expected,"context_available",state.context_seen[s]);
    if(state.context_seen[s]) {
      const auto name="context-h"+std::to_string(s)+".json";
      io::String(expected,"context_file",name); io::String(expected,"context_sha256",state.hashes.at(name));
    }
    auto children=[&](const char* label,unsigned count) {
      io::Value values(rapidjson::kArrayType);
      for(unsigned i=0;i<count;++i) {
        const bool amplitude=count==3,seen=amplitude?state.amplitude_seen[s][i]:state.contact_seen[s][i];
        const bool complete=amplitude?actual.amplitudes[i].complete:actual.contact[i].complete;
        const bool passed=amplitude?actual.amplitudes[i].passed:actual.contact[i].passed;
        io::Document child; child.SetObject(); io::Boolean(child,"captured",seen);
        io::Boolean(child,"complete",seen&&complete); io::Boolean(child,"passed",seen&&passed);
        if(seen) io::String(child,"file",std::string(amplitude?"amplitude-h":"contact-h")+std::to_string(s)+
          (amplitude?"-a":"-b")+std::to_string(i)+".json");
        raw_detail::Append(expected,values,child);
      }
      expected.AddMember(io::Value(label,expected.GetAllocator()),values,expected.GetAllocator());
    };
    children("amplitudes",3); children("contact",2);
    io::Boolean(expected,"captured",state.step_seen[s]); io::Boolean(expected,"complete",state.step_seen[s]&&actual.complete);
    io::Boolean(expected,"passed",state.step_seen[s]&&actual.passed);
    if(state.step_seen[s]) io::String(expected,"file","step-h"+std::to_string(s)+".json");
    Same(saved,expected,"Derived inventory changed a captured child/context status");
  }
  io::Require(rd::Unsigned(rd::Field(d,"completed_amplitudes"))==state.analysis.completed_amplitudes&&
    rd::Unsigned(rd::Field(d,"completed_contacts"))==state.analysis.completed_contacts&&
    rd::Unsigned(rd::Field(d,"completed_steps"))==state.analysis.completed_steps,"Changed derived completion counts");
  if(final||!state.progress_without_job) {
    io::Require(rd::Boolean(rd::Field(d,"input_valid"))==state.analysis.input_valid,"Changed derived model/input validity");
    (void)Diagnostic(d);
  } else io::Require(!d.HasMember("input_valid")&&!d.HasMember("diagnostic"),"Premature derived callback state");
  const auto& files=rd::Field(d,"files");
  io::Require(files.IsArray()&&files.Size()==state.receipt.files.size(),"Changed derived inventory prefix length");
  for(unsigned i=0;i<files.Size();++i) {
    rd::Keys(files[i],{"name","sha256","bytes"}); const auto& f=state.receipt.files[i];
    io::Require(rd::Text(rd::Field(files[i],"name"),64)==f.name&&rd::Text(rd::Field(files[i],"sha256"),64)==f.sha256&&
      rd::Unsigned(rd::Field(files[i],"bytes"))==f.bytes,"Changed derived prefix file/hash/bytes");
  }
}
} // namespace tl::qualification::qeph::wall_recurrence::derived_read
