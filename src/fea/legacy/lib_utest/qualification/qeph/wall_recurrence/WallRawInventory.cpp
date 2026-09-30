#include "WallRawReport.h"
#include "WallRawJson.h"
#include <iomanip>
#include <sstream>

namespace tl::qualification::qeph::wall_recurrence {
namespace io=crash::output;
void RawJobWriter::Inventory(const RawJob* job,bool final) {
  auto d=Header(final?"final-index":"progress-index");
  io::Boolean(d,"final_index",final); io::Boolean(d,"collection_complete",final&&job&&job->collection_complete);
  io::Boolean(d,"model_prepared",job&&job->model.prepared());
  io::String(d,"inventory_scope","All files written before this index; this index is excluded from its own inventory");
  io::Integer(d,"bytes_before_this_index",receipt_.total_bytes); io::Integer(d,"remaining_screen_budget_at_job_start",budget_);
  io::Integer(d,"file_byte_cap",RawFileByteCap); io::Integer(d,"shared_raw_and_derived_byte_cap",ScreenSetByteCap);
  io::Integer(d,"planned_native_cell_intervals",PlannedNativeCellIntervals(receipt_.cells));
  io::Boolean(d,"native_cell_interval_count_exact",final&&job&&job->collection_complete);
  if(final&&job&&job->collection_complete) io::Integer(d,"completed_native_cell_intervals",PlannedNativeCellIntervals(receipt_.cells));
  io::String(d,"physical_scope","Prescribed full native/contact probes; no startup, trajectory or spectral admission");
  io::FiniteArray(d,"fixed_dt_grid_s",recurrence::Steps.data(),6); io::FiniteArray(d,"amplitudes",recurrence::Amplitudes.data(),3);
  if(job) {
    bool complete=job->model.prepared(); io::String(d,"diagnostic",job->diagnostic);
    io::Value steps(rapidjson::kArrayType);
    for(unsigned s=0;s<6;++s) {
      const auto& source=job->steps[s]; complete=complete&&CompleteRawStep(source,job->cells);
      io::Document step; step.SetObject(); io::Number(step,"fixed_dt_s",source.h);
      io::Value native(rapidjson::kArrayType),contact(rapidjson::kArrayType);
      for(unsigned a=0;a<3;++a) {
        io::Require(source.native_attempted[a]==native_seen_[s][a],"Native payload/inventory association mismatch");
        io::Require(source.native[a].derivative.complete==native_complete_[s][a]&&
          source.native[a].derivative.completed_columns==native_columns_[s][a],"Native completion changed after raw capture");
        complete=complete&&native_complete_[s][a];
        io::Document p; p.SetObject(); io::Boolean(p,"attempted",source.native_attempted[a]);
        io::Boolean(p,"matrix_complete",source.native_attempted[a]&&source.native[a].derivative.complete);
        io::Integer(p,"completed_columns",source.native[a].derivative.completed_columns);
        if(source.native_attempted[a]) io::String(p,"file","native-h"+std::to_string(s)+"-a"+std::to_string(a)+".json");
        raw_detail::Append(step,native,p);
      }
      for(unsigned b=0;b<2;++b) {
        io::Require(source.contact_attempted[b]==contact_seen_[s][b],"Contact payload/inventory association mismatch");
        io::Require(source.contact[b].complete==contact_checks_complete_[s][b]&&
          source.contact[b].passed==contact_checks_passed_[s][b],"Contact check status changed after raw capture");
        complete=complete&&contact_raw_complete_[s][b];
        io::Document p; p.SetObject(); io::Boolean(p,"attempted",source.contact_attempted[b]);
        io::Boolean(p,"local_checks_complete",source.contact_attempted[b]&&source.contact[b].complete);
        io::Boolean(p,"local_checks_passed",source.contact_attempted[b]&&source.contact[b].passed);
        if(source.contact_attempted[b]) io::String(p,"file","contact-h"+std::to_string(s)+"-b"+std::to_string(b)+".json");
        raw_detail::Append(step,contact,p);
      }
      step.AddMember("native_probes",native,step.GetAllocator()); step.AddMember("contact_branches",contact,step.GetAllocator());
      raw_detail::Append(d,steps,step);
    }
    io::Require(!job->collection_complete||complete,"False completed raw collection");
    d.AddMember("steps",steps,d.GetAllocator());
  }
  io::Value files(rapidjson::kArrayType);
  for(const auto& file:receipt_.files) {
    io::Document f; f.SetObject(); io::String(f,"name",file.name); io::String(f,"sha256",file.sha256); io::Integer(f,"bytes",file.bytes);
    raw_detail::Append(d,files,f);
  }
  d.AddMember("files",files,d.GetAllocator());
  std::ostringstream name;
  if(final) name<<"index.json";
  else name<<"progress-"<<std::setw(3)<<std::setfill('0')<<sequence_<<".json";
  Write(name.str(),EncodeRawJson(d));
  if(final) { receipt_.final_index_present=true; receipt_.collection_complete=job&&job->collection_complete; }
  else ++sequence_;
}
} // namespace tl::qualification::qeph::wall_recurrence
