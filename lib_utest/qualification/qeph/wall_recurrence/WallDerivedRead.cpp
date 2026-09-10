#include "WallDerivedReadFields.h"
#include <cmath>
#include <iomanip>
#include <set>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace tl::qualification::qeph::wall_recurrence {
namespace dr=derived_read;
namespace rd=read_detail;
namespace io=crash::output;
namespace {
std::string ProgressName(unsigned sequence) {
  std::ostringstream name; name<<"progress-"<<std::setw(3)<<std::setfill('0')<<sequence<<".json"; return name.str();
}
std::vector<RawFileReceipt> Files(const io::Value& list) {
  io::Require(list.IsArray()&&list.Size()>=2&&list.Size()<128,"Invalid derived inventory size");
  std::vector<RawFileReceipt> files; std::set<std::string> names;
  for(const auto& value:list.GetArray()) {
    rd::Keys(value,{"name","sha256","bytes"});
    RawFileReceipt file{rd::Text(rd::Field(value,"name"),64),rd::Text(rd::Field(value,"sha256"),64),0};
    const auto bytes=rd::Unsigned(rd::Field(value,"bytes"));
    io::Require(!file.name.empty()&&file.name!="."&&file.name!=".."&&file.name!="index.json"&&
      file.name.find_first_not_of("abcdefghijklmnopqrstuvwxyz0123456789-.")==std::string::npos&&names.insert(file.name).second&&
      rd::Hash(file.sha256)&&bytes>0&&bytes<=RawFileByteCap,"Malformed derived file receipt");
    file.bytes=bytes; files.push_back(std::move(file));
  }
  io::Require(files[0].name=="provenance.json"&&files[1].name=="progress-000.json","Missing initial derived provenance/inventory");
  return files;
}
std::string ReadFile(const std::filesystem::path& directory,const RawFileReceipt& file) {
  const auto path=directory/file.name;
  io::Require(std::filesystem::is_regular_file(std::filesystem::symlink_status(path))&&std::filesystem::file_size(path)==file.bytes,
              "Derived file kind/size mismatch");
  const auto bytes=io::ReadBounded(path,RawFileByteCap);
  io::Require(bytes.size()==file.bytes&&io::Sha256(bytes)==file.sha256,"Derived file hash/bytes mismatch"); return bytes;
}
void BindRaw(const RawReadResult& raw) {
  io::Require((raw.job.cells==1||raw.job.cells==2)&&FrozenVelocity(raw.job.normal_velocity)&&
    !(raw.job.normal_velocity==0&&std::signbit(raw.job.normal_velocity))&&raw.receipt.final_index_present&&
    raw.receipt.cells==raw.job.cells&&io::Bits(raw.receipt.normal_velocity)==io::Bits(raw.job.normal_velocity)&&
    raw.receipt.collection_complete==raw.job.collection_complete,"Invalid supplied authenticated raw result");
  std::set<std::string> names; std::size_t total=0;
  for(const auto& f:raw.receipt.files) {
    io::Require(names.insert(f.name).second&&rd::Hash(f.sha256)&&f.bytes>0&&f.bytes<=RawFileByteCap&&
      f.bytes<=ScreenSetByteCap-total,"Malformed authenticated raw receipt"); total+=f.bytes;
  }
  io::Require(raw.receipt.files.size()<=80&&total==raw.receipt.total_bytes&&
    rd::Hash(dr::RawHash(raw,"index.json"))&&rd::Hash(dr::RawHash(raw,"provenance.json")),"Raw receipt lacks bound index/provenance");
  for(unsigned s=0;s<6;++s) io::Require(raw.job.steps[s].h==recurrence::Steps[s],"Changed authenticated raw step");
  if(raw.job.model.prepared()) {
    WallRecurrenceModel expected; std::string error;
    if(!BuildWallRecurrenceModel(raw.job.cells,expected,error)) throw std::runtime_error(error);
    dr::Same(raw_detail::DescribeModel(raw.job.model),raw_detail::DescribeModel(expected),"Authenticated model changed after read");
  }
}
}
bool ReadWallJobSummary(const std::filesystem::path& directory,const RawReadResult& raw,
    const WallDerivedReadBinding& binding,WallDerivedReadResult& output,std::string& error) {
  try {
    io::Require(rd::Hash(binding.analysis_index_sha256)&&rd::Hash(binding.analysis_provenance_sha256)&&
      binding.remaining_set_bytes>0&&binding.remaining_set_bytes<=ScreenSetByteCap,"Invalid external derived binding/budget");
    BindRaw(raw);
    io::Require(std::filesystem::is_directory(std::filesystem::symlink_status(directory)),"Derived directory must be real");
    const auto path=directory/"index.json";
    io::Require(std::filesystem::is_regular_file(std::filesystem::symlink_status(path)),"Missing closed derived index");
    const auto bytes=io::ReadBounded(path,RawFileByteCap);
    io::Require(io::Sha256(bytes)==binding.analysis_index_sha256,"External derived index hash mismatch");
    const auto index=rd::Parse(bytes); const auto files=Files(rd::Field(index,"files"));
    std::size_t total=bytes.size(); std::set<std::string> names{"index.json"};
    for(const auto& f:files) {
      io::Require(f.bytes<=binding.remaining_set_bytes&&total<=binding.remaining_set_bytes-f.bytes,"Derived job exceeds remaining shared bytes");
      total+=f.bytes; names.insert(f.name);
    }
    for(const auto& item:std::filesystem::directory_iterator(directory))
      io::Require(names.erase(item.path().filename().string())==1,"Uninventoried derived artifact");
    io::Require(names.empty(),"Missing inventoried derived artifact");
    const auto provenance=ReadFile(directory,files.front()); (void)ParseRawProvenance(provenance);
    io::Require(files.front().sha256==binding.analysis_provenance_sha256,"External analysis provenance hash mismatch");
    dr::State state{raw,binding}; state.origin=rd::Text(rd::Field(index,"analysis_provenance_origin"));
    io::Require(!state.origin.empty(),"Empty analysis provenance origin");
    state.producer_budget=rd::Unsigned(rd::Field(index,"remaining_set_budget_at_job_start"));
    io::Require(total<=state.producer_budget&&state.producer_budget<=ScreenSetByteCap,"Derived final size exceeds producer budget");
    auto& receipt=state.receipt; receipt.cells=raw.job.cells; receipt.normal_velocity=raw.job.normal_velocity;
    receipt.raw_index_sha256=dr::RawHash(raw,"index.json"); receipt.raw_provenance_sha256=dr::RawHash(raw,"provenance.json");
    receipt.analysis_provenance_sha256=binding.analysis_provenance_sha256;
    receipt.files.push_back(files.front()); receipt.total_bytes=files.front().bytes;
    auto& analysis=state.analysis; analysis.cells=raw.job.cells; analysis.normal_velocity=raw.job.normal_velocity;
    analysis.input_valid=raw.job.model.prepared();
    analysis.dimension=analysis.input_valid?static_cast<unsigned>(raw.job.model.native().dictionary.size()):0;
    for(unsigned s=0;s<6;++s) {
      analysis.steps[s].h=recurrence::Steps[s]; analysis.steps[s].contact[1].branch=ContactBranch::Active;
      for(unsigned a=0;a<3;++a) analysis.steps[s].amplitudes[a].amplitude=recurrence::Amplitudes[a];
    }
    bool needs_progress=true; unsigned progress=0;
    for(unsigned i=1;i<files.size();++i) {
      const auto& file=files[i]; const auto d=rd::Parse(ReadFile(directory,file)); const auto kind=rd::Text(rd::Field(d,"kind"));
      dr::Header(d,kind.c_str(),state);
      if(kind=="progress-index") {
        io::Require(needs_progress&&file.name==ProgressName(progress++),"Changed derived progress order");
        dr::Inventory(d,state,false); needs_progress=false;
      } else {
        io::Require(!needs_progress&&analysis.input_valid,"Payload precedes its required progress/model");
        const unsigned s=rd::Count(rd::Field(d,"step_index"),5);
        io::Require(s==state.next_step,"Changed chronological derived callback order");
        state.progress_without_job=false;
        if(kind=="context") {
          io::Require(file.name=="context-h"+std::to_string(s)+".json","Renamed derived context");
          dr::Context(d,s,state); state.progress_without_job=true;
        } else {
          io::Require(rd::Number(rd::Field(d,"fixed_dt_s"))==recurrence::Steps[s],"Changed derived payload step");
          if(kind=="amplitude") {
            const unsigned a=rd::Count(rd::Field(d,"amplitude_index"),2);
            io::Require(file.name=="amplitude-h"+std::to_string(s)+"-a"+std::to_string(a)+".json","Renamed derived amplitude");
            dr::Amplitude(d,s,a,state);
          } else if(kind=="contact-recheck") {
            const unsigned b=rd::Count(rd::Field(d,"branch_index"),1);
            io::Require(file.name=="contact-h"+std::to_string(s)+"-b"+std::to_string(b)+".json","Renamed derived contact");
            dr::Contact(d,s,b,state);
          } else if(kind=="step") {
            io::Require(file.name=="step-h"+std::to_string(s)+".json","Renamed derived step");
            dr::Step(d,s,state); dr::ReleaseDense(analysis.steps[s]); state.contexts[s].schedule.scalar.clear();
          } else io::Require(false,"Unexpected derived payload kind");
        }
        needs_progress=true;
      }
      receipt.files.push_back(file); receipt.total_bytes+=file.bytes; state.hashes[file.name]=file.sha256;
    }
    io::Require(!needs_progress,"Closed derived inventory lacks final progress");
    dr::Header(index,"final-index",state); dr::Inventory(index,state,true);
    WallDerivedReadResult result; result.diagnostic=rd::Text(rd::Field(index,"diagnostic"));
    receipt.files.push_back({"index.json",binding.analysis_index_sha256,bytes.size()}); receipt.total_bytes=total;
    receipt.final_index_present=true; receipt.analysis_complete=rd::Boolean(rd::Field(index,"analysis_complete"));
    receipt.analysis_passed=rd::Boolean(rd::Field(index,"analysis_passed"));
    bool available=analysis.input_valid;
    for(unsigned s=0;s<6;++s) available=available&&state.step_seen[s]&&state.context_usable[s];
    if(available) {
      std::string detail;
      if(!CompactWallJobAnalysis(raw.job,analysis,result.summary,detail)) throw std::runtime_error(detail);
      result.summary_available=true;
    }
    result.receipt=std::move(receipt); output=std::move(result); error.clear(); return true;
  } catch(const std::exception& e) { error=e.what(); return false; }
}
} // namespace tl::qualification::qeph::wall_recurrence
