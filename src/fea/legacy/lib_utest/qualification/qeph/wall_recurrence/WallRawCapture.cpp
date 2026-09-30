#include "WallRawCapture.h"
#include <stdexcept>

namespace tl::qualification::qeph::wall_recurrence {
namespace r=recurrence;
std::uint64_t PlannedNativeCellIntervals(unsigned cells) {
  if(cells!=1&&cells!=2) throw std::invalid_argument("Raw capture requires one or two cells");
  const unsigned nodes=2*(cells+1),dimension=12*nodes+61*cells;
  return 6u*cells*(3u*(1u+2u*dimension)+2u*(1u+3u*(nodes+7u)));
}
bool CompleteRawStep(const RawStep& step,unsigned cells) {
  if(cells!=1&&cells!=2) return false;
  const unsigned nodes=2*(cells+1),dimension=12*nodes+61*cells;
  for(unsigned a=0;a<3;++a) {
    const auto& p=step.native[a];
    if(!step.native_attempted[a]||!p.baseline_complete||!p.derivative.complete||
       p.derivative.completed_columns!=dimension) return false;
  }
  for(unsigned b=0;b<2;++b) {
    const auto& p=step.contact[b];
    if(!step.contact_attempted[b]||!p.baseline_complete||p.directions.size()!=nodes+7) return false;
    for(const auto& direction:p.directions) if(direction.completed_samples!=3) return false;
  }
  return true;
}
RawJob CollectRawJob(unsigned cells,double velocity,const RawProgressCallback& callback) {
  if((cells!=1&&cells!=2)||!FrozenVelocity(velocity))
    throw std::invalid_argument("Raw capture requires a frozen fixture and physical boost");
  RawJob job; job.cells=cells; job.normal_velocity=velocity;
  for(unsigned s=0;s<6;++s) job.steps[s].h=r::Steps[s];
  const bool prepared=BuildWallRecurrenceModel(cells,job.model,job.diagnostic);
  if(callback) callback(job,{RawProgressKind::Model});
  if(!prepared) {
    if(callback) callback(job,{RawProgressKind::Finished});
    return job;
  }
  for(unsigned s=0;s<6;++s) for(unsigned a=0;a<3;++a) {
    auto& step=job.steps[s]; step.native_attempted[a]=true;
    step.native[a]=DifferentiateMoving(job.model.native(),step.h,r::Amplitudes[a],{velocity,0,0});
    if(callback) callback(job,{RawProgressKind::NativeMatrix,s,a});
  }
  for(unsigned s=0;s<6;++s) {
    auto& step=job.steps[s];
    if(!step.native.back().derivative.complete) continue;
    for(unsigned b=0;b<2;++b) {
      step.contact_attempted[b]=true;
      step.contact[b]=ProbeContactBranch(job.model,step.h,velocity,
        b==0?ContactBranch::Inactive:ContactBranch::Active,step.native.back().derivative.full);
      if(callback) callback(job,{RawProgressKind::ContactBranch,s,b});
    }
  }
  job.collection_complete=true;
  for(const auto& step:job.steps) job.collection_complete=job.collection_complete&&CompleteRawStep(step,cells);
  if(!job.collection_complete) job.diagnostic="One or more native matrices or physical branch samples are incomplete";
  if(callback) callback(job,{RawProgressKind::Finished});
  return job;
}
} // namespace tl::qualification::qeph::wall_recurrence
