#include "WallRawReadFields.h"
#include <cmath>
#include <iomanip>
#include <set>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace tl::qualification::qeph::wall_recurrence {
namespace rd=read_detail;
namespace io=crash::output;
namespace {
std::string ProgressName(unsigned sequence) {
  std::ostringstream name; name<<"progress-"<<std::setw(3)<<std::setfill('0')<<sequence<<".json"; return name.str();
}
std::string ReadFile(const std::filesystem::path& directory,const RawFileReceipt& f) {
  const auto path=directory/f.name;
  io::Require(std::filesystem::is_regular_file(std::filesystem::symlink_status(path))&&
    std::filesystem::file_size(path)==f.bytes,"Raw file kind/size mismatch");
  const auto bytes=io::ReadBounded(path,RawFileByteCap);
  io::Require(bytes.size()==f.bytes&&io::Sha256(bytes)==f.sha256,"Raw file hash/bytes mismatch"); return bytes;
}
std::vector<RawFileReceipt> Files(const io::Value& list) {
  io::Require(list.IsArray()&&list.Size()>=2&&list.Size()<80,"Invalid closed raw inventory size");
  std::vector<RawFileReceipt> files; std::set<std::string> names;
  for(const auto& item:list.GetArray()) {
    rd::Keys(item,{"name","sha256","bytes"});
    RawFileReceipt f{rd::Text(rd::Field(item,"name"),64),rd::Text(rd::Field(item,"sha256"),64),0};
    const auto bytes=rd::Unsigned(rd::Field(item,"bytes"));
    io::Require(!f.name.empty()&&f.name!="."&&f.name!=".."&&f.name!="index.json"&&
      f.name.find_first_not_of("abcdefghijklmnopqrstuvwxyz0123456789-.")==std::string::npos&&
      names.insert(f.name).second&&rd::Hash(f.sha256)&&bytes>0&&bytes<=RawFileByteCap,"Malformed retained file receipt");
    f.bytes=bytes; files.push_back(std::move(f));
  }
  io::Require(files[0].name=="provenance.json"&&files[1].name=="progress-000.json","Missing initial raw provenance/inventory"); return files;
}
void Model(const io::Document& d,RawJob& job) {
  job.diagnostic=rd::Text(rd::Field(d,"diagnostic"));
  if(!rd::Boolean(rd::Field(d,"prepared"))) {
    io::Require(!d.HasMember("model"),"Unprepared raw model contains data"); return;
  }
  std::string error;
  if(!BuildWallRecurrenceModel(job.cells,job.model,error)) throw std::runtime_error("Current owning model rejected: "+error);
  io::Require(rd::Same(raw_detail::DescribeModel(job.model),rd::Field(d,"model")),"Retained model differs from exact frozen owning model");
}
void Contact(const io::Document& d,RawJob& job,const RawJobReceipt& prefix) {
  const unsigned s=rd::Count(rd::Field(d,"step_index"),5),b=rd::Count(rd::Field(d,"branch_index"),1);
  auto& step=job.steps[s]; io::Require(!step.contact_attempted[b]&&step.native_attempted[2]&&step.native[2].derivative.complete,
    "Contact payload precedes complete finest native matrix or repeats");
  const auto name="native-h"+std::to_string(s)+"-a2.json";
  io::Require(rd::Text(rd::Field(d,"native_matrix_file"))==name&&rd::Unsigned(rd::Field(d,"native_amplitude_index"))==2,
    "Contact payload names wrong native matrix");
  std::string hash; for(const auto& file:prefix.files) if(file.name==name) hash=file.sha256;
  io::Require(!hash.empty()&&rd::Text(rd::Field(d,"native_matrix_sha256"))==hash,"Contact/native payload hash mismatch");
  auto probe=rd::Branch(rd::Field(d,"probe"),job.cells,step.h,job.normal_velocity,b);
  if(probe.full.size()) {
    Eigen::MatrixXd expected; std::string error;
    if(!BuildContactBranch(job.model,step.h,probe.branch,step.native[2].derivative.full,expected,error)) throw std::runtime_error(error);
    io::Require(rd::Same(raw_detail::Matrix(expected),raw_detail::Matrix(probe.full)),"Retained contact operator does not match native matrix/kick");
  }
  std::vector<ContactDirection> directions; std::string error;
  if(!SignConeDirections(job.model,probe.branch,directions,error)) throw std::runtime_error(error);
  for(unsigned i=0;i<probe.directions.size();++i) {
    const auto& sampled=probe.directions[i]; const auto& actual=sampled.direction;
    io::Require(actual.name==directions[i].name&&actual.value.size()==directions[i].value.size(),"Changed physical sign-cone direction identity");
    for(Eigen::Index n=0;n<actual.value.size();++n)
      io::Require(io::Bits(actual.value[n])==io::Bits(directions[i].value[n]),"Changed physical sign-cone direction/scales");
    for(unsigned a=0;a<sampled.completed_samples;++a) for(const auto& node:sampled.samples[a].nodes)
      io::Require(node.touching_or_penetrating==(probe.branch==ContactBranch::Active),"Completed sample leaves declared physical sign cone");
  }
  step.contact[b]=std::move(probe); step.contact_attempted[b]=true;
}
}
bool ReadRawJob(const std::filesystem::path& directory,const RawReadBinding& binding,
                RawReadResult& output,std::string& error) {
  try {
    io::Require((binding.cells==1||binding.cells==2)&&FrozenVelocity(binding.normal_velocity)&&
      !(binding.normal_velocity==0&&std::signbit(binding.normal_velocity))&&rd::Hash(binding.provenance_sha256)&&
      rd::Hash(binding.index_sha256)&&binding.remaining_screen_byte_budget>0&&binding.remaining_screen_byte_budget<=ScreenSetByteCap,
      "Invalid externally supplied raw binding/budget");
    io::Require(std::filesystem::is_directory(std::filesystem::symlink_status(directory)),"Raw directory must be a real directory");
    const auto index_path=directory/"index.json";
    io::Require(std::filesystem::is_regular_file(std::filesystem::symlink_status(index_path)),"Missing closed raw index");
    const auto index_bytes=io::ReadBounded(index_path,RawFileByteCap);
    io::Require(io::Sha256(index_bytes)==binding.index_sha256,"External raw index hash mismatch");
    const auto index=rd::Parse(index_bytes); auto files=Files(rd::Field(index,"files"));
    std::size_t total=index_bytes.size(); std::set<std::string> names{"index.json"};
    for(const auto& file:files) {
      io::Require(file.bytes<=binding.remaining_screen_byte_budget&&total<=binding.remaining_screen_byte_budget-file.bytes,
                  "Retained raw job exceeds remaining shared budget"); total+=file.bytes; names.insert(file.name);
    }
    for(const auto& entry:std::filesystem::directory_iterator(directory))
      io::Require(names.erase(entry.path().filename().string())==1,"Uninventoried raw artifact consumes bytes");
    io::Require(names.empty(),"Missing inventoried raw artifact");
    const auto provenance=ReadFile(directory,files.front()); (void)ParseRawProvenance(provenance);
    io::Require(files.front().sha256==binding.provenance_sha256,"External raw provenance hash mismatch");
    const auto origin=rd::Text(rd::Field(index,"provenance_origin")); io::Require(!origin.empty(),"Empty raw provenance origin");
    const auto producer_budget=rd::Unsigned(rd::Field(index,"remaining_screen_budget_at_job_start"));
    io::Require(total<=producer_budget&&producer_budget<=ScreenSetByteCap,"Raw final bytes exceed producer budget");
    RawReadResult result; auto& job=result.job; auto& receipt=result.receipt;
    job.cells=binding.cells; job.normal_velocity=binding.normal_velocity;
    receipt.cells=binding.cells; receipt.normal_velocity=binding.normal_velocity;
    for(unsigned s=0;s<6;++s) job.steps[s].h=recurrence::Steps[s];
    receipt.files.push_back(files.front()); receipt.total_bytes=files.front().bytes;
    unsigned progress=0; bool model_seen=false; std::string model_hash;
    for(unsigned i=1;i<files.size();++i) {
      const auto& file=files[i]; const auto d=rd::Parse(ReadFile(directory,file)); const auto kind=rd::Text(rd::Field(d,"kind"));
      rd::Header(d,kind.c_str(),binding,files.front(),origin,model_hash);
      if(kind=="progress-index") {
        io::Require(file.name==ProgressName(progress++)&&rd::Unsigned(rd::Field(d,"remaining_screen_budget_at_job_start"))==producer_budget,
                    "Changed raw progress order/budget");
        rd::Inventory(d,model_seen?&job:nullptr,receipt,binding,false);
      } else if(kind=="model") {
        io::Require(!model_seen&&file.name=="model.json","Duplicate or renamed raw model");
        Model(d,job); model_hash=file.sha256; model_seen=true;
      } else if(kind=="native-matrix") {
        io::Require(model_seen&&job.model.prepared(),"Native payload precedes prepared raw model");
        const auto s=rd::Count(rd::Field(d,"step_index"),5),a=rd::Count(rd::Field(d,"amplitude_index"),2);
        io::Require(file.name=="native-h"+std::to_string(s)+"-a"+std::to_string(a)+".json"&&!job.steps[s].native_attempted[a],
          "Renamed or duplicate raw native matrix");
        job.steps[s].native[a]=rd::Native(rd::Field(d,"probe"),binding.cells,job.steps[s].h,binding.normal_velocity,a);
        job.steps[s].native_attempted[a]=true;
      } else if(kind=="contact-branch") {
        io::Require(model_seen&&job.model.prepared(),"Contact payload precedes prepared raw model");
        const auto s=rd::Count(rd::Field(d,"step_index"),5),b=rd::Count(rd::Field(d,"branch_index"),1);
        io::Require(file.name=="contact-h"+std::to_string(s)+"-b"+std::to_string(b)+".json","Renamed raw contact branch");
        Contact(d,job,receipt);
      } else io::Require(false,"Unexpected payload kind before final raw index");
      receipt.files.push_back(file); receipt.total_bytes+=file.bytes;
    }
    io::Require(model_seen,"Closed raw index lacks model outcome");
    job.collection_complete=rd::Boolean(rd::Field(index,"collection_complete"));
    if(job.collection_complete) for(const auto& step:job.steps)
      io::Require(CompleteRawStep(step,job.cells),"False complete raw collection");
    job.diagnostic=rd::Text(rd::Field(index,"diagnostic"));
    rd::Header(index,"final-index",binding,files.front(),origin,model_hash);
    rd::Inventory(index,&job,receipt,binding,true);
    receipt.files.push_back({"index.json",binding.index_sha256,index_bytes.size()}); receipt.total_bytes=total;
    receipt.final_index_present=true; receipt.collection_complete=job.collection_complete;
    output=std::move(result); error.clear(); return true;
  } catch(const std::exception& e) { error=e.what(); return false; }
}
} // namespace tl::qualification::qeph::wall_recurrence
