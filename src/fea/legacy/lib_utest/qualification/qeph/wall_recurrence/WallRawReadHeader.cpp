#include "WallRawReadFields.h"
#include <set>

namespace tl::qualification::qeph::wall_recurrence::read_detail {
void Header(const io::Document& d,const char* kind,const RawReadBinding& binding,
            const RawFileReceipt& provenance,const std::string& origin,const std::string& model_hash) {
  std::set<std::string> allowed;
  auto text=[&](const char* name,const std::string& expected) {
    allowed.insert(name); io::Require(Text(Field(d,name))==expected,(std::string("Raw header mismatch: ")+name).c_str());
  };
  auto integer=[&](const char* name,std::uint64_t expected) {
    allowed.insert(name); io::Require(Unsigned(Field(d,name))==expected,(std::string("Raw header integer mismatch: ")+name).c_str());
  };
  text("schema","robo-dyna-qeph-wall-raw-v1"); text("kind",kind); integer("cells",binding.cells);
  allowed.insert("normal_velocity_m_s");
  io::Require(io::Bits(Number(Field(d,"normal_velocity_m_s")))==io::Bits(binding.normal_velocity),"Foreign raw physical boost");
  integer("normal_velocity_binary64_bits",io::Bits(binding.normal_velocity));
  for(const char* name:{"simulation_ready","screen_decision_included"}) {
    allowed.insert(name); io::Require(!Boolean(Field(d,name)),"Raw capture asserts unauthorized admission");
  }
  text("probe_phase","Prescribed base epoch1/timeh to endpoint epoch2/time2h; no runtime owner or startup");
  text("source_work_scope","Native EINT0/EINT1/EVIS are source work, not elastic potential");
  text("provenance_file","provenance.json"); text("provenance_sha256",binding.provenance_sha256);
  integer("provenance_bytes",provenance.bytes); text("provenance_origin",origin);
  text("provenance_validation","Exact bytes/hash and structural JSON; listed sources require separate authentication");
  if(!model_hash.empty()) { text("model_file","model.json"); text("model_sha256",model_hash); }
  const std::string type=kind;
  if(type=="model") for(const char* name:{"prepared","diagnostic","model"}) allowed.insert(name);
  else if(type=="native-matrix") for(const char* name:{"step_index","amplitude_index","probe"}) allowed.insert(name);
  else if(type=="contact-branch") for(const char* name:{"step_index","branch_index","native_matrix_file","native_amplitude_index","native_matrix_sha256","probe"}) allowed.insert(name);
  else if(type=="progress-index"||type=="final-index") {
    for(const char* name:{"final_index","collection_complete","model_prepared","inventory_scope","bytes_before_this_index",
        "remaining_screen_budget_at_job_start","file_byte_cap","shared_raw_and_derived_byte_cap","planned_native_cell_intervals",
        "native_cell_interval_count_exact","completed_native_cell_intervals","physical_scope","fixed_dt_grid_s","amplitudes","diagnostic","steps","files"})
      allowed.insert(name);
  } else io::Require(false,"Unknown raw payload kind");
  for(auto it=d.MemberBegin();it!=d.MemberEnd();++it)
    io::Require(allowed.count(std::string(it->name.GetString(),it->name.GetStringLength()))==1,"Unexpected raw header field");
}
void Inventory(const io::Document& d,const RawJob* job,const RawJobReceipt& prefix,
               const RawReadBinding& binding,bool final) {
  io::Require(Boolean(Field(d,"final_index"))==final,"Raw inventory final flag mismatch");
  const bool claimed=Boolean(Field(d,"collection_complete"));
  io::Require(!claimed||(final&&job&&job->collection_complete),"False raw collection-complete claim");
  io::Require(Boolean(Field(d,"model_prepared"))==(job&&job->model.prepared()),"Raw inventory model flag mismatch");
  io::Require(Text(Field(d,"inventory_scope"))=="All files written before this index; this index is excluded from its own inventory",
              "Changed raw inventory scope");
  io::Require(Unsigned(Field(d,"bytes_before_this_index"))==prefix.total_bytes,"Raw prefix byte count mismatch");
  const auto producer_budget=Unsigned(Field(d,"remaining_screen_budget_at_job_start"));
  io::Require(producer_budget>0&&producer_budget<=ScreenSetByteCap,"Invalid raw producer budget");
  io::Require(Unsigned(Field(d,"file_byte_cap"))==RawFileByteCap&&
    Unsigned(Field(d,"shared_raw_and_derived_byte_cap"))==ScreenSetByteCap,"Changed raw byte caps");
  io::Require(Unsigned(Field(d,"planned_native_cell_intervals"))==PlannedNativeCellIntervals(binding.cells),"Changed raw call forecast");
  io::Require(Boolean(Field(d,"native_cell_interval_count_exact"))==claimed,"Raw call-count completeness mismatch");
  if(claimed) io::Require(Unsigned(Field(d,"completed_native_cell_intervals"))==PlannedNativeCellIntervals(binding.cells),"Raw exact call count mismatch");
  else io::Require(!d.HasMember("completed_native_cell_intervals"),"Incomplete raw job asserts exact call count");
  io::Require(Text(Field(d,"physical_scope"))=="Prescribed full native/contact probes; no startup, trajectory or spectral admission","Changed raw scope");
  const auto& steps=Field(d,"fixed_dt_grid_s"); const auto& amplitudes=Field(d,"amplitudes");
  io::Require(steps.IsArray()&&steps.Size()==6&&amplitudes.IsArray()&&amplitudes.Size()==3,"Changed raw grid dimensions");
  for(unsigned s=0;s<6;++s) io::Require(Number(steps[s])==recurrence::Steps[s],"Changed raw fixed step");
  for(unsigned a=0;a<3;++a) io::Require(Number(amplitudes[a])==recurrence::Amplitudes[a],"Changed raw amplitude");
  if(job) {
    (void)Text(Field(d,"diagnostic")); const auto& summaries=Field(d,"steps");
    io::Require(summaries.IsArray()&&summaries.Size()==6,"Changed raw step summary count");
    for(unsigned s=0;s<6;++s) {
      const auto& step=summaries[s]; Keys(step,{"fixed_dt_s","native_probes","contact_branches"});
      io::Require(Number(Field(step,"fixed_dt_s"))==job->steps[s].h,"Raw summary step mismatch");
      const auto& n=Field(step,"native_probes"); const auto& c=Field(step,"contact_branches");
      io::Require(n.IsArray()&&n.Size()==3&&c.IsArray()&&c.Size()==2,"Raw summary probe counts mismatch");
      for(unsigned a=0;a<3;++a) {
        const bool attempted=job->steps[s].native_attempted[a]; const auto& p=job->steps[s].native[a].derivative;
        if(attempted) Keys(n[a],{"attempted","matrix_complete","completed_columns","file"});
        else Keys(n[a],{"attempted","matrix_complete","completed_columns"});
        io::Require(Boolean(Field(n[a],"attempted"))==attempted&&Boolean(Field(n[a],"matrix_complete"))==(attempted&&p.complete)&&
          Count(Field(n[a],"completed_columns"),194)==p.completed_columns,"Raw native summary mismatch");
        if(attempted) io::Require(Text(Field(n[a],"file"))=="native-h"+std::to_string(s)+"-a"+std::to_string(a)+".json","Raw native summary link mismatch");
      }
      for(unsigned b=0;b<2;++b) {
        const bool attempted=job->steps[s].contact_attempted[b]; const auto& p=job->steps[s].contact[b];
        if(attempted) Keys(c[b],{"attempted","local_checks_complete","local_checks_passed","file"});
        else Keys(c[b],{"attempted","local_checks_complete","local_checks_passed"});
        io::Require(Boolean(Field(c[b],"attempted"))==attempted&&Boolean(Field(c[b],"local_checks_complete"))==(attempted&&p.complete)&&
          Boolean(Field(c[b],"local_checks_passed"))==(attempted&&p.passed),"Raw contact summary mismatch");
        if(attempted) io::Require(Text(Field(c[b],"file"))=="contact-h"+std::to_string(s)+"-b"+std::to_string(b)+".json","Raw contact summary link mismatch");
      }
    }
  } else io::Require(!d.HasMember("steps")&&!d.HasMember("diagnostic"),"Premature raw initial inventory data");
  const auto& files=Field(d,"files"); io::Require(files.IsArray()&&files.Size()==prefix.files.size(),"Raw inventory prefix size mismatch");
  for(unsigned i=0;i<files.Size();++i) {
    Keys(files[i],{"name","sha256","bytes"}); const auto& f=prefix.files[i];
    io::Require(Text(Field(files[i],"name"),64)==f.name&&Text(Field(files[i],"sha256"),64)==f.sha256&&
      Unsigned(Field(files[i],"bytes"))==f.bytes,"Raw inventory prefix hash/bytes mismatch");
  }
}
} // namespace tl::qualification::qeph::wall_recurrence::read_detail
