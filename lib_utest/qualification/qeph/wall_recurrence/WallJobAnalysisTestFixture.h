#pragma once
#include "WallJobAnalysis.h"
#include "WallRecurrenceTestFixture.h"
#include <stdexcept>
#include <utility>

namespace tl::qualification::qeph::wall_recurrence::job_test {
inline RawJob Job(unsigned cells=1,double velocity=0) {
  RawJob job; job.cells=cells; job.normal_velocity=velocity; std::string error;
  if(!BuildWallRecurrenceModel(cells,job.model,error)) throw std::runtime_error(error);
  for(unsigned s=0;s<6;++s) job.steps[s].h=recurrence::Steps[s]; return job;
}
inline Eigen::VectorXd Baseline(const WallRecurrenceModel& model,double h,double velocity) {
  const auto& d=model.native().dictionary; Eigen::VectorXd result=Eigen::VectorXd::Zero(d.size());
  for(unsigned i=0;i<d.size();++i) {
    if(d[i].group==recurrence::Group::Position&&d[i].component==0) result[i]=h*velocity/d[i].scale;
    if(d[i].group==recurrence::Group::Velocity&&d[i].component==0) result[i]=velocity/d[i].scale;
  }
  return result;
}
inline void Native(RawJob& job,unsigned step,unsigned amplitude) {
  auto& p=job.steps[step].native[amplitude]; job.steps[step].native_attempted[amplitude]=true;
  p.velocity={job.normal_velocity,0,0}; p.baseline_complete=true;
  p.baseline=Baseline(job.model,job.steps[step].h,job.normal_velocity);
  p.derivative.amplitude=recurrence::Amplitudes[amplitude]; p.derivative.complete=true;
  const auto n=job.model.native().dictionary.size(); p.derivative.completed_columns=n;
  // Deliberately not native shell physics. This full identity fails the
  // observer test and exercises retention of a valid failed analysis.
  p.derivative.full=Eigen::MatrixXd::Identity(n,n);
}
// Physical contact records come from the already-qualified owning host law.
// The normalized state is an explicit synthetic linear-map oracle, not a
// native force call or an authenticated scientific matrix.
inline ContactMapSample Sample(const WallRecurrenceModel& model,const Eigen::VectorXd& input,
                               const Eigen::VectorXd& synthetic_state,double velocity) {
  const auto& m=model.native();
  std::array<double,3*recurrence::MaxNodes> x{},v{};
  std::array<double,recurrence::MaxNodes> inverse{};
  std::array<std::uint8_t,recurrence::MaxNodes> fixed{};
  for(unsigned n=0;n<m.nodes;++n) {
    const double base[]{m.position[n].x,m.position[n].y,m.position[n].z};
    for(unsigned a=0;a<3;++a) {
      const auto xi=model.coordinate(recurrence::Group::Position,n,a),vi=model.coordinate(recurrence::Group::Velocity,n,a);
      x[3*n+a]=base[a]+input[xi]*m.dictionary[xi].scale;
      v[3*n+a]=(a==0?velocity:0)+input[vi]*m.dictionary[vi].scale;
    }
    inverse[n]=1/m.mass[n];
  }
  contact::NodalWallResult result;
  const auto report=contact::EvaluateNodalWallContact(model.weights(),{x.data(),m.nodes,3,1},{v.data(),m.nodes,3,1},
    {inverse.data(),fixed.data(),m.nodes,1,contact::TranslationMassModel::kIsotropicLumped},model.law(),1,&result);
  if(report.status!=contact::NodalWallStatus::Ok) throw std::runtime_error("Synthetic-state test's actual point law failed");
  ContactMapSample sample; sample.state=synthetic_state;
  sample.nodes.assign(result.nodes.begin(),result.nodes.begin()+result.node_count);
  sample.parents.assign(result.parents.begin(),result.parents.begin()+result.parent_count);
  sample.resultant=result.resultant; sample.potential=result.potential;
  sample.wall_reaction=result.wall_reaction; sample.wall_moment=result.wall_moment;
  sample.surface_power=result.surface_power; sample.base_epoch=result.base_epoch; sample.attempt=result.attempt;
  return sample;
}
inline ContactBranchProbe Contact(const RawJob& job,unsigned step,ContactBranch branch,const Eigen::MatrixXd& shell) {
  ContactBranchProbe probe; probe.h=job.steps[step].h; probe.normal_velocity=job.normal_velocity; probe.branch=branch;
  std::string error;
  if(!BuildContactBranch(job.model,probe.h,branch,shell,probe.full,error)) throw std::runtime_error(error);
  const auto baseline=Baseline(job.model,probe.h,job.normal_velocity);
  probe.baseline=Sample(job.model,Eigen::VectorXd::Zero(baseline.size()),baseline,job.normal_velocity);
  probe.baseline_complete=true; std::vector<ContactDirection> directions;
  if(!SignConeDirections(job.model,branch,directions,error)) throw std::runtime_error(error);
  for(auto& direction:directions) {
    ContactDirectionProbe d; d.direction=std::move(direction);
    for(unsigned a=0;a<3;++a) {
      const Eigen::VectorXd input=recurrence::Amplitudes[a]*d.direction.value;
      const Eigen::VectorXd state=baseline+probe.full*input;
      d.samples[a]=Sample(job.model,input,state,job.normal_velocity); ++d.completed_samples;
    }
    // Intentionally leave every old numerical check false/empty. The derived
    // checker must reconstruct its own quotient and verdict from the samples.
    probe.directions.push_back(std::move(d));
  }
  return probe;
}
} // namespace tl::qualification::qeph::wall_recurrence::job_test
