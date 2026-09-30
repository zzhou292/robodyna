#pragma once
#include "WallRawRead.h"
#include "WallRawReportTestFixture.h"
#include "WallRawJson.h"
#include <limits>
#include <utility>

namespace tl::qualification::qeph::wall_recurrence::read_test {
namespace io=crash::output;
namespace rt=raw_test;
inline RawReadBinding Binding(const std::filesystem::path& path) {
  return {1,0,io::Sha256(io::ReadBounded(path/"provenance.json",ProvenanceByteCap)),
    io::Sha256(io::ReadBounded(path/"index.json",RawFileByteCap)),ScreenSetByteCap};
}
// Real host contact fields, deliberately synthetic zero native output. These
// are report-protocol fixtures, never supplied as an admitted physical map.
inline ContactMapSample HostSample(const WallRecurrenceModel& model,const Eigen::VectorXd& input) {
  const auto& m=model.native(); std::array<double,18> x{},v{}; std::array<double,6> inverse{};
  std::array<std::uint8_t,6> fixed{};
  for(unsigned n=0;n<m.nodes;++n) {
    const double base[]{m.position[n].x,m.position[n].y,m.position[n].z};
    for(unsigned a=0;a<3;++a) {
      const auto p=model.coordinate(recurrence::Group::Position,n,a),q=model.coordinate(recurrence::Group::Velocity,n,a);
      x[3*n+a]=base[a]+m.dictionary[p].scale*input[p]; v[3*n+a]=m.dictionary[q].scale*input[q];
    }
    inverse[n]=1/m.mass[n];
  }
  contact::NodalWallResult physical;
  const auto status=contact::EvaluateNodalWallContact(model.weights(),{x.data(),m.nodes,3,1},{v.data(),m.nodes,3,1},
    {inverse.data(),fixed.data(),m.nodes,1,contact::TranslationMassModel::kIsotropicLumped},model.law(),1,&physical);
  if(status.status!=contact::NodalWallStatus::Ok) throw std::runtime_error("Host report fixture contact rejected");
  ContactMapSample out; out.state=Eigen::VectorXd::Zero(m.dictionary.size());
  out.nodes.assign(physical.nodes.begin(),physical.nodes.begin()+physical.node_count);
  out.parents.assign(physical.parents.begin(),physical.parents.begin()+physical.parent_count);
  out.resultant=physical.resultant; out.potential=physical.potential; out.wall_reaction=physical.wall_reaction;
  out.wall_moment=physical.wall_moment; out.surface_power=physical.surface_power; out.base_epoch=physical.base_epoch; out.attempt=physical.attempt;
  return out;
}
inline ContactBranchProbe Contact(RawJob& job,unsigned step,unsigned branch,bool all_directions) {
  ContactBranchProbe p; p.h=job.steps[step].h; p.branch=branch?ContactBranch::Active:ContactBranch::Inactive;
  p.normal_velocity=0; std::string error;
  if(!BuildContactBranch(job.model,p.h,p.branch,job.steps[step].native[2].derivative.full,p.full,error)) throw std::runtime_error(error);
  p.baseline=HostSample(job.model,Eigen::VectorXd::Zero(job.model.native().dictionary.size())); p.baseline_complete=true;
  std::vector<ContactDirection> directions;
  if(!SignConeDirections(job.model,p.branch,directions,error)) throw std::runtime_error(error);
  for(unsigned i=0;i<(all_directions?directions.size():1);++i) {
    ContactDirectionProbe d; d.direction=directions[i];
    for(unsigned a=0;a<3;++a) { d.samples[a]=HostSample(job.model,recurrence::Amplitudes[a]*d.direction.value); ++d.completed_samples; }
    if(i==0) d.quotients[0]=Eigen::VectorXd::Constant(d.direction.value.size(),std::numeric_limits<double>::infinity());
    d.diagnostic="Synthetic rejected derivative, physical host fields retained"; p.directions.push_back(std::move(d));
  }
  p.diagnostic="Synthetic protocol fixture; no native or screen admission"; return p;
}
inline RawJobReceipt Write(const std::filesystem::path& path,bool complete,bool contact_payload=false) {
  auto job=rt::Fixture(); RawJobWriter writer(path,1,0,rt::Provenance(),"synthetic.json",ScreenSetByteCap);
  writer(job,{RawProgressKind::Model});
  if(complete) {
    for(unsigned s=0;s<6;++s) for(unsigned a=0;a<3;++a) { rt::Native(job,s,a,true); writer(job,{RawProgressKind::NativeMatrix,s,a}); }
    for(unsigned s=0;s<6;++s) for(unsigned b=0;b<2;++b) {
      job.steps[s].contact[b]=Contact(job,s,b,true); job.steps[s].contact_attempted[b]=true;
      writer(job,{RawProgressKind::ContactBranch,s,b});
    }
    job.collection_complete=true;
  } else if(contact_payload) {
    rt::Native(job,0,2,true); writer(job,{RawProgressKind::NativeMatrix,0,2});
    job.steps[0].contact[1]=Contact(job,0,1,false); job.steps[0].contact_attempted[1]=true;
    writer(job,{RawProgressKind::ContactBranch,0,1});
  } else {
    rt::Native(job,0,0,false); job.steps[0].native[0].derivative.full(0,1)=std::numeric_limits<double>::quiet_NaN();
    writer(job,{RawProgressKind::NativeMatrix,0,0});
  }
  writer(job,{RawProgressKind::Finished}); return writer.receipt();
}
// Full semantic snapshot of retained owning model, every attempted probe and
// receipt. Avoid comparing padding/pointers in vector-containing value types.
inline std::string Snapshot(const RawReadResult& result) {
  io::Document d; d.SetObject(); const auto& job=result.job;
  io::Integer(d,"cells",job.cells); io::Number(d,"boost",job.normal_velocity);
  io::Boolean(d,"complete",job.collection_complete); io::String(d,"diagnostic",job.diagnostic);
  io::Boolean(d,"prepared",job.model.prepared());
  if(job.model.prepared()) raw_detail::Field(d,"model",raw_detail::DescribeModel(job.model));
  io::Value steps(rapidjson::kArrayType);
  for(const auto& step:job.steps) {
    io::Document value; value.SetObject(); io::Number(value,"h",step.h);
    for(unsigned a=0;a<3;++a) {
      const auto name="native"+std::to_string(a); io::Boolean(value,(name+"_attempted").c_str(),step.native_attempted[a]);
      if(step.native_attempted[a]) raw_detail::Field(value,name.c_str(),raw_detail::DescribeNative(step.native[a],job.cells,step.h,job.normal_velocity,a));
    }
    for(unsigned b=0;b<2;++b) {
      const auto name="contact"+std::to_string(b); io::Boolean(value,(name+"_attempted").c_str(),step.contact_attempted[b]);
      if(step.contact_attempted[b]) raw_detail::Field(value,name.c_str(),raw_detail::DescribeBranch(step.contact[b],job.cells,step.h,job.normal_velocity,b));
    }
    raw_detail::Append(d,steps,value);
  }
  d.AddMember("steps",steps,d.GetAllocator()); const auto& receipt=result.receipt;
  io::Integer(d,"receipt_cells",receipt.cells); io::Number(d,"receipt_boost",receipt.normal_velocity);
  io::Integer(d,"bytes",receipt.total_bytes); io::Boolean(d,"final_index",receipt.final_index_present);
  io::Boolean(d,"receipt_complete",receipt.collection_complete); io::Value files(rapidjson::kArrayType);
  for(const auto& file:receipt.files) {
    io::Document f; f.SetObject(); io::String(f,"name",file.name); io::String(f,"hash",file.sha256); io::Integer(f,"bytes",file.bytes);
    raw_detail::Append(d,files,f);
  }
  d.AddMember("files",files,d.GetAllocator()); return EncodeRawJson(d);
}
using Mutation=std::function<void(const std::string&,io::Document&)>;
// Test-only clone: rebind every dependent hash so structural mutations reach
// the semantic reader instead of merely failing the outer byte hash.
RawReadBinding Clone(const std::filesystem::path& source,const std::filesystem::path& destination,const Mutation&);
} // namespace tl::qualification::qeph::wall_recurrence::read_test
