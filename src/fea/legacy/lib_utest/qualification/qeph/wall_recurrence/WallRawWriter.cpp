#include "WallRawReport.h"
#include "WallRawJson.h"
#include <cmath>

namespace tl::qualification::qeph::wall_recurrence {
namespace io=crash::output;
namespace detail=raw_detail;
RawJobWriter::RawJobWriter(const std::filesystem::path& directory,unsigned cells,double velocity,
                           const std::string& provenance,const std::string& origin,std::size_t budget)
    :directory_(directory),budget_(budget),provenance_origin_(origin) {
  io::Require((cells==1||cells==2)&&FrozenVelocity(velocity)&&!(velocity==0&&std::signbit(velocity)),"Invalid raw writer job identity");
  io::Require(!origin.empty()&&origin.size()<=4096&&budget>0&&budget<=ScreenSetByteCap,"Invalid raw writer provenance/budget");
  (void)ParseRawProvenance(provenance);
  io::Require(provenance.size()<budget,"Provenance exhausts remaining screen byte budget");
  io::Require(!std::filesystem::exists(directory)&&std::filesystem::create_directory(directory),"Raw directory must be new");
  receipt_.cells=cells; receipt_.normal_velocity=velocity; receipt_.files.reserve(80);
  provenance_hash_=Write("provenance.json",provenance).sha256;
  Inventory(nullptr,false);
}
RawFileReceipt RawJobWriter::Write(const std::string& name,const std::string& bytes) {
  io::Require(!bytes.empty()&&bytes.size()<=RawFileByteCap&&bytes.size()<=budget_-receipt_.total_bytes,
              "Raw artifact exceeds file or remaining shared screen budget");
  io::Require(receipt_.files.size()<80,"Raw artifact inventory capacity exceeded");
  for(const auto& file:receipt_.files) io::Require(file.name!=name,"Duplicate raw artifact name");
  RawFileReceipt file{name,io::Sha256(bytes),bytes.size()};
  io::WriteBytes(directory_/name,bytes);
  receipt_.total_bytes+=bytes.size(); receipt_.files.push_back(file); return file;
}
io::Document RawJobWriter::Header(const char* kind) const {
  io::Document d; d.SetObject(); io::String(d,"schema","robo-dyna-qeph-wall-raw-v1");
  io::String(d,"kind",kind); io::Integer(d,"cells",receipt_.cells);
  io::Number(d,"normal_velocity_m_s",receipt_.normal_velocity);
  io::Integer(d,"normal_velocity_binary64_bits",io::Bits(receipt_.normal_velocity));
  io::Boolean(d,"simulation_ready",false); io::Boolean(d,"screen_decision_included",false);
  io::String(d,"probe_phase","Prescribed base epoch1/timeh to endpoint epoch2/time2h; no runtime owner or startup");
  io::String(d,"source_work_scope","Native EINT0/EINT1/EVIS are source work, not elastic potential");
  io::String(d,"provenance_file","provenance.json"); io::String(d,"provenance_sha256",provenance_hash_);
  io::Integer(d,"provenance_bytes",receipt_.files.front().bytes); io::String(d,"provenance_origin",provenance_origin_);
  io::String(d,"provenance_validation","Exact bytes/hash and structural JSON; listed sources require separate authentication");
  if(!model_hash_.empty()) { io::String(d,"model_file","model.json"); io::String(d,"model_sha256",model_hash_); }
  return d;
}
void RawJobWriter::operator()(const RawJob& job,RawProgress event) {
  io::Require(!receipt_.final_index_present&&job.cells==receipt_.cells&&
    io::Bits(job.normal_velocity)==io::Bits(receipt_.normal_velocity),"Closed writer or foreign raw job");
  for(unsigned s=0;s<6;++s) io::Require(job.steps[s].h==recurrence::Steps[s],"Changed raw step grid");
  if(event.kind==RawProgressKind::Model) {
    io::Require(!model_seen_,"Duplicate raw model event");
    auto d=Header("model"); io::Boolean(d,"prepared",job.model.prepared()); io::String(d,"diagnostic",job.diagnostic);
    if(job.model.prepared()) {
      io::Require(job.model.native().elements==job.cells,"Foreign raw model fixture");
      detail::Field(d,"model",detail::DescribeModel(job.model));
    }
    model_hash_=Write("model.json",EncodeRawJson(d)).sha256; model_seen_=true;
  } else if(event.kind==RawProgressKind::NativeMatrix) {
    io::Require(model_seen_&&job.model.prepared()&&event.step<6&&event.index<3&&
      !native_seen_[event.step][event.index]&&job.steps[event.step].native_attempted[event.index],"Invalid raw native progress event");
    auto d=Header("native-matrix"); io::Integer(d,"step_index",event.step); io::Integer(d,"amplitude_index",event.index);
    detail::Field(d,"probe",detail::DescribeNative(job.steps[event.step].native[event.index],job.cells,
      job.steps[event.step].h,job.normal_velocity,event.index));
    Write("native-h"+std::to_string(event.step)+"-a"+std::to_string(event.index)+".json",EncodeRawJson(d));
    native_seen_[event.step][event.index]=true;
    native_complete_[event.step][event.index]=job.steps[event.step].native[event.index].derivative.complete;
    native_columns_[event.step][event.index]=job.steps[event.step].native[event.index].derivative.completed_columns;
  } else if(event.kind==RawProgressKind::ContactBranch) {
    io::Require(model_seen_&&job.model.prepared()&&event.step<6&&event.index<2&&
      !contact_seen_[event.step][event.index]&&job.steps[event.step].contact_attempted[event.index]&&
      native_seen_[event.step][2]&&native_complete_[event.step][2],"Invalid raw contact progress event");
    auto d=Header("contact-branch"); io::Integer(d,"step_index",event.step); io::Integer(d,"branch_index",event.index);
    const auto native_name="native-h"+std::to_string(event.step)+"-a2.json";
    io::String(d,"native_matrix_file",native_name); io::Integer(d,"native_amplitude_index",2);
    for(const auto& file:receipt_.files) if(file.name==native_name) io::String(d,"native_matrix_sha256",file.sha256);
    io::Require(d.HasMember("native_matrix_sha256"),"Missing retained finest native matrix");
    detail::Field(d,"probe",detail::DescribeBranch(job.steps[event.step].contact[event.index],job.cells,
      job.steps[event.step].h,job.normal_velocity,event.index));
    Write("contact-h"+std::to_string(event.step)+"-b"+std::to_string(event.index)+".json",EncodeRawJson(d));
    contact_seen_[event.step][event.index]=true;
    const auto& probe=job.steps[event.step].contact[event.index];
    bool collected=probe.baseline_complete&&probe.directions.size()==2*(job.cells+1)+7;
    for(const auto& direction:probe.directions) collected=collected&&direction.completed_samples==3;
    contact_raw_complete_[event.step][event.index]=collected;
    contact_checks_complete_[event.step][event.index]=probe.complete;
    contact_checks_passed_[event.step][event.index]=probe.passed;
  } else if(event.kind==RawProgressKind::Finished) {
    io::Require(model_seen_,"Raw final event precedes model capture"); Inventory(&job,true); return;
  } else io::Require(false,"Unknown raw progress event");
  Inventory(&job,false);
}
} // namespace tl::qualification::qeph::wall_recurrence
