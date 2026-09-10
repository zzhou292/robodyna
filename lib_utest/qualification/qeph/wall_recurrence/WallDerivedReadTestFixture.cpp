#include "WallDerivedReadTestFixture.h"
#include "WallRawJson.h"
#include <cmath>
#include <limits>
#include <map>

namespace tl::qualification::qeph::wall_recurrence::derived_test {
namespace {
Eigen::MatrixXd Operator(const WallRecurrenceModel& model,double h) {
  const auto& m=model.native(); const auto& d=m.dictionary; const unsigned size=d.size();
  Eigen::MatrixXd a=Eigen::MatrixXd::Identity(size,size);
  for(unsigned row=0;row<size;++row) {
    if(d[row].group==recurrence::Group::ForceCache) a.row(row).setZero();
    if(d[row].group!=recurrence::Group::Velocity&&d[row].group!=recurrence::Group::Spin) continue;
    for(unsigned col=0;col<size;++col) if(d[col].group==recurrence::Group::ForceCache) {
      const bool force=d[col].component<12; const unsigned node=m.connectivity[d[col].entity][(d[col].component%12)/3];
      if(force==(d[row].group==recurrence::Group::Velocity)&&node==d[row].entity&&d[row].component==d[col].component%3)
        a(row,col)=-h*d[col].scale/((force?m.mass[node]:m.inertia[node])*d[row].scale);
    }
  }
  for(unsigned row=0;row<size;++row) {
    const bool position=d[row].group==recurrence::Group::Position;
    if(!position&&d[row].group!=recurrence::Group::OrientationTangent) continue;
    const auto rate=model.coordinate(position?recurrence::Group::Velocity:recurrence::Group::Spin,d[row].entity,d[row].component);
    for(unsigned col=0;col<size;++col) a(row,col)=(row==col?1.:0.)+h*d[rate].scale/d[row].scale*a(rate,col);
  }
  return a;
}
WallSpectrum MeasuredSpectrum(const Eigen::MatrixXd& matrix) {
  WallSpectrum out; out.matrix_norm=matrix.norm(); out.complete=true; out.passed=true;
  out.spectral_radius=1; out.near_one=matrix.rows(); out.eigenvalues.resize(matrix.rows(),{1,0});
  out.diagnostic="Synthetic reviewed-producer spectrum; not a shell eigensolve"; return out;
}
WallGramAnalysis MeasuredGram(unsigned dimension,unsigned steps,double gain) {
  WallGramAnalysis out; out.ordinary_steps=steps; out.state_count=steps+1;
  const double eigenvalue=out.state_count*gain*gain;
  out.eigenvalues=Eigen::VectorXd::Constant(dimension,eigenvalue);
  out.minimum_eigenvalue=eigenvalue; out.maximum_eigenvalue=eigenvalue; out.gram_norm=out.eigenvalues.norm();
  out.controlling_direction=Eigen::VectorXd::Zero(dimension); out.controlling_direction[dimension-1]=1;
  out.controlling_coordinate=dimension-1; out.mean_gain=std::sqrt(eigenvalue/out.state_count);
  out.individual_power_bound=std::sqrt(eigenvalue); out.complete=true;
  out.within_gain_budget=out.mean_gain<=recurrence::MaximumGramGain;
  out.diagnostic="Synthetic measured Gram; no power or eigenvalue solve"; return out;
}
WallSequenceAnalysis MeasuredSequence(unsigned dimension,unsigned steps,double gain) {
  WallSequenceAnalysis out; out.raw=MeasuredGram(dimension,steps,80); out.weighted=MeasuredGram(dimension,steps,gain);
  out.complete=true; out.passed=out.weighted.within_gain_budget; return out;
}
void PrepareAmplitude(const RawJob& raw,WallJobAnalysis& job,unsigned s,unsigned a,bool failed_scalar) {
  auto& result=job.steps[s].amplitudes[a]; const auto& source=raw.steps[s].native[a];
  result.attempted=true; result.input_complete=true; result.baseline=CheckWallMovingBaseline(raw.model,raw.steps[s].h,source);
  auto& b=result.branches; b.h=raw.steps[s].h; std::string error;
  if(!BuildWallStateMetric(raw.model,b.metric,error)) throw std::runtime_error(error);
  b.schedule=AnalyzeWallSwitchingSchedule(raw.model,b.h); const auto n=job.dimension;
  bool pass=true;
  for(unsigned i=0;i<2;++i) {
    auto& c=b.branches[i]; c.branch=i?ContactBranch::Active:ContactBranch::Inactive;
    if(!BuildContactBranch(raw.model,b.h,c.branch,source.derivative.full,c.full,error)||
       !ApplyWallStateMetric(c.full,b.metric.diagonal,c.weighted,error)) throw std::runtime_error(error);
    c.identities=CheckWallStateIdentities(raw.model,b.h,c.full);
    c.spectrum=MeasuredSpectrum(c.full); c.weighted_spectrum=MeasuredSpectrum(c.weighted);
    c.continuous=MeasuredSequence(n,b.schedule.ordinary_steps,s==5?65:1);
    pass=pass&&c.identities.passed&&c.continuous.passed;
  }
  for(unsigned i=0;i<9;++i) {
    b.events[i].window=b.schedule.windows[i];
    b.events[i].sequence=MeasuredSequence(n,b.schedule.ordinary_steps,s==5?65:1);
    pass=pass&&b.events[i].sequence.passed;
  }
  b.completed_branches=2; b.completed_events=9; b.complete=true; b.passed=pass;
  if(failed_scalar) {
    auto& spectrum=b.branches[0].spectrum; spectrum.eigenvalues.clear(); spectrum.near_one=0;
    spectrum.spectral_radius=0; spectrum.schur_residual=std::numeric_limits<double>::quiet_NaN();
    spectrum.complete=false; spectrum.passed=false; spectrum.diagnostic="Synthetic partial nonfinite Schur residual";
    b.completed_branches=1; b.complete=false; b.passed=false;
  }
  result.complete=result.baseline.complete&&b.complete; result.passed=result.complete&&result.baseline.passed&&b.passed;
  result.diagnostic="Synthetic report test, not qualified native dynamics";
  job.completed_amplitudes+=result.complete;
}
}
Fixture::Fixture(bool partial,bool nonfinite) {
  auto source=job_test::Job(); const auto raw_path=directory.path/"raw";
  RawJobWriter writer(raw_path,1,0,rt::Provenance(),"synthetic-raw.json",ScreenSetByteCap);
  writer(source,{RawProgressKind::Model});
  for(unsigned s=0;s<6;++s) for(unsigned a=0;a<3;++a) {
    job_test::Native(source,s,a); source.steps[s].native[a].derivative.full=Operator(source.model,source.steps[s].h);
    writer(source,{RawProgressKind::NativeMatrix,s,a});
  }
  for(unsigned s=0;s<6;++s) for(unsigned b=0;b<2;++b) {
    source.steps[s].contact[b]=job_test::Contact(source,s,b?ContactBranch::Active:ContactBranch::Inactive,source.steps[s].native[2].derivative.full);
    source.steps[s].contact_attempted[b]=true; writer(source,{RawProgressKind::ContactBranch,s,b});
  }
  source.collection_complete=true; writer(source,{RawProgressKind::Finished});
  raw_binding={1,0,io::Sha256(rt::Provenance()),writer.receipt().files.back().sha256,ScreenSetByteCap};
  std::string error; if(!ReadRawJob(raw_path,raw_binding,raw,error)) throw std::runtime_error(error);
  derived=directory.path/"derived"; WallJobReportWriter report(derived,raw,raw_binding,rt::Provenance(),"synthetic-analysis.json",ScreenSetByteCap);
  WallJobAnalysis job; job.cells=1; job.dimension=109; job.input_valid=true;
  for(unsigned s=0;s<6;++s) {
    job.steps[s].h=recurrence::Steps[s]; job.steps[s].contact[1].branch=ContactBranch::Active;
    for(unsigned a=0;a<3;++a) job.steps[s].amplitudes[a].amplitude=recurrence::Amplitudes[a];
  }
  bool all_complete=true,all_passed=true;
  for(unsigned s=0;s<6;++s) {
    auto& step=job.steps[s];
    for(unsigned a=0;a<3;++a) {
      PrepareAmplitude(raw.job,job,s,a,nonfinite&&s==0&&a==0); report(job,{WallJobProgressKind::Amplitude,s,a});
      if(partial) break;
    }
    if(partial) break;
    bool complete=true,pass=true;
    for(const auto& a:step.amplitudes) { complete=complete&&a.complete; pass=pass&&a.passed; }
    for(unsigned p=0;p<2;++p) {
      step.native_amplitude_comparisons[p]=CompareWallMatrices(raw.job.steps[s].native[p].derivative.full,raw.job.steps[s].native[p+1].derivative.full);
      step.derived_amplitude_comparisons[p]=CompareWallAnalyses(step.amplitudes[p].branches,step.amplitudes[p+1].branches);
      complete=complete&&step.native_amplitude_comparisons[p].complete&&step.derived_amplitude_comparisons[p].complete;
      pass=pass&&step.native_amplitude_comparisons[p].passed&&step.derived_amplitude_comparisons[p].passed;
    }
    for(unsigned b=0;b<2;++b) {
      step.contact[b]=RecheckWallContact(raw.job.model,step.h,0,b?ContactBranch::Active:ContactBranch::Inactive,
        raw.job.steps[s].native[2].derivative.full,raw.job.steps[s].contact[b]);
      job.completed_contacts+=step.contact[b].complete;
      complete=complete&&step.contact[b].complete; pass=pass&&step.contact[b].passed;
      report(job,{WallJobProgressKind::Contact,s,b});
    }
    step.complete=complete; step.passed=complete&&pass; job.completed_steps+=step.complete;
    all_complete=all_complete&&step.complete; all_passed=all_passed&&step.passed;
    report(job,{WallJobProgressKind::Step,s});
  }
  job.complete=!partial&&all_complete; job.passed=job.complete&&all_passed;
  job.diagnostic="Synthetic protocol fixture: failed 4H0, raw gain 80 is diagnostic";
  report(job,{WallJobProgressKind::Finished});
  binding={report.receipt().files.back().sha256,io::Sha256(rt::Provenance()),ScreenSetByteCap};
}

WallDerivedReadBinding Clone(const std::filesystem::path& source,const std::filesystem::path& target,const Mutation& mutate) {
  io::Require(std::filesystem::create_directory(target),"New derived clone directory required");
  const auto index=rt::Read(source/"index.json"); std::vector<std::string> names;
  for(const auto& f:index["files"].GetArray()) names.emplace_back(f["name"].GetString()); names.push_back("index.json");
  std::vector<RawFileReceipt> files; std::map<std::string,std::string> hashes; std::size_t total=0;
  for(const auto& name:names) {
    std::string bytes=io::ReadBounded(source/name,RawFileByteCap);
    if(name!="provenance.json") {
      auto d=rt::Read(source/name);
      if(d.HasMember("context_sha256")) d["context_sha256"].SetString(hashes.at(d["context_file"].GetString()).c_str(),d.GetAllocator());
      for(const char* label:{"derived_amplitudes","derived_contact"}) if(d.HasMember(label))
        for(auto& link:d[label].GetArray()) link["sha256"].SetString(hashes.at(link["file"].GetString()).c_str(),d.GetAllocator());
      if(d.HasMember("files")) {
        io::Value entries(rapidjson::kArrayType);
        for(const auto& f:files) {
          io::Document entry; entry.SetObject(); io::String(entry,"name",f.name); io::String(entry,"sha256",f.sha256); io::Integer(entry,"bytes",f.bytes);
          raw_detail::Append(d,entries,entry);
        }
        d["files"]=std::move(entries); d["bytes_before_this_index"].SetUint64(total);
        for(auto& step:d["steps"].GetArray()) if(step["context_available"].GetBool())
          step["context_sha256"].SetString(hashes.at(step["context_file"].GetString()).c_str(),d.GetAllocator());
      }
      mutate(name,d); bytes=EncodeRawJson(d);
    }
    const auto hash=io::Sha256(bytes); io::WriteBytes(target/name,bytes);
    files.push_back({name,hash,bytes.size()}); hashes[name]=hash; total+=bytes.size();
  }
  return {hashes.at("index.json"),hashes.at("provenance.json"),ScreenSetByteCap};
}
std::string Snapshot(const WallDerivedReadResult& result) {
  io::Document d; d.SetObject(); io::Boolean(d,"available",result.summary_available); io::String(d,"diagnostic",result.diagnostic);
  const auto& r=result.receipt; io::Integer(d,"cells",r.cells); io::Number(d,"boost",r.normal_velocity); io::Integer(d,"bytes",r.total_bytes);
  io::String(d,"raw_index",r.raw_index_sha256); io::String(d,"raw_provenance",r.raw_provenance_sha256); io::String(d,"analysis_provenance",r.analysis_provenance_sha256);
  io::Boolean(d,"final",r.final_index_present); io::Boolean(d,"complete",r.analysis_complete); io::Boolean(d,"passed",r.analysis_passed);
  io::Value files(rapidjson::kArrayType),steps(rapidjson::kArrayType);
  for(const auto& f:r.files) {
    io::Document file; file.SetObject(); io::String(file,"name",f.name); io::String(file,"hash",f.sha256); io::Integer(file,"bytes",f.bytes);
    raw_detail::Append(d,files,file);
  }
  const auto& summary=result.summary; io::Integer(d,"summary_cells",summary.cells); io::Integer(d,"dimension",summary.dimension); io::Number(d,"summary_boost",summary.normal_velocity);
  for(const auto& s:summary.steps) {
    io::Document step; step.SetObject(); io::Number(step,"h",s.h); io::Integer(step,"total",s.total_steps); io::Integer(step,"ordinary",s.ordinary_steps);
    io::Boolean(step,"complete",s.complete); io::Boolean(step,"passed",s.passed);
    step.AddMember("diagonal",raw_detail::Vector(step,s.diagonal),step.GetAllocator());
    io::Value windows(rapidjson::kArrayType),amplitudes(rapidjson::kArrayType);
    for(const auto& w:s.windows) {
      io::Value row(rapidjson::kArrayType); row.PushBack(w.entry_shift,step.GetAllocator()); row.PushBack(w.exit_shift,step.GetAllocator());
      for(unsigned x:{w.entry_base_epoch,w.exit_base_epoch,w.inactive_before,w.active,w.inactive_after}) row.PushBack(x,step.GetAllocator());
      windows.PushBack(row,step.GetAllocator());
    }
    for(const auto& a:s.amplitudes) {
      io::Document amplitude; amplitude.SetObject(); io::Boolean(amplitude,"complete",a.complete); io::Boolean(amplitude,"passed",a.passed);
      io::Value gains(rapidjson::kArrayType);
      for(const auto& g:a.gains) { io::Value row(rapidjson::kArrayType); row.PushBack(g.raw,amplitude.GetAllocator()); row.PushBack(g.weighted,amplitude.GetAllocator()); row.PushBack(g.complete,amplitude.GetAllocator()); gains.PushBack(row,amplitude.GetAllocator()); }
      amplitude.AddMember("gains",gains,amplitude.GetAllocator()); raw_detail::Append(step,amplitudes,amplitude);
    }
    step.AddMember("windows",windows,step.GetAllocator()); step.AddMember("amplitudes",amplitudes,step.GetAllocator()); raw_detail::Append(d,steps,step);
  }
  d.AddMember("files",files,d.GetAllocator()); d.AddMember("steps",steps,d.GetAllocator()); return EncodeRawJson(d);
}
} // namespace tl::qualification::qeph::wall_recurrence::derived_test
