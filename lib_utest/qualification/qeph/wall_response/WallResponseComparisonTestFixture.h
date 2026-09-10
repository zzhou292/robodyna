#pragma once
#include "WallResponseComparison.h"
#include "WallResponseTestFixture.h"
#include <array>
#include <cmath>
#include <stdexcept>

namespace tl::qualification::qeph::wall_response::comparison_test {
// Synthetic observation records, not a discrete owner trajectory. The exact
// continuum velocity is kept while depth is perturbed smoothly inside contact.
// This supplies nonzero, independently decreasing U/K0 and x/d differences
// without moving the entry/release zeros or inventing native material history.
inline Run Synthetic(unsigned refinement=1,unsigned cells=1,double depth_error=.008,unsigned last_epoch=MaxSteps) {
  Run run; run.config=test::ConfigFor(cells,refinement); std::string error;
  if(!BuildModel(cells,run.model,error)) throw std::runtime_error(error);
  run.owner_id=71+refinement; run.samples.reserve(SampleCount);
  run.owner_device_bytes=1; run.batch_device_bytes=1; run.wall_device_bytes=1;
  run.owner_allocations=1; run.batch_allocations=1; run.wall_allocations=1;
  const long double omega=std::sqrt(static_cast<long double>(run.model.screened().mass_rates()[0].value));
  const unsigned count=std::min(last_epoch,Steps(run.config));
  for(unsigned epoch=0;epoch<=count;++epoch) {
    const long double tau=static_cast<long double>(epoch)*Step(run.config)-
        static_cast<long double>(wr::InitialGap)/wr::ImpactSpeed;
    const double shape=static_cast<double>(std::max(0.L,std::sin(omega*tau)));
    const auto input=test::EndpointAt(run.model,run.config,epoch,1+depth_error/refinement*shape);
    Sample sample; Summary next;
    if(!ObserveEndpoint(run.model,run.config,input,sample,error)||
       !StageSummary(run.model,run.config,run.summary,sample,next,error))
      throw std::runtime_error("Synthetic comparison observation rejected: "+error);
    run.summary=next; run.last_accepted=sample;
    if(epoch%SampleStride(run.config)==0) run.samples.push_back(sample);
  }
  run.accepted_steps=run.attempted_steps=count;
  run.native_cell_intervals=run.accepted_steps*cells;
  // The counters and allocation fields above are labelled synthetic metadata;
  // no native interval or device allocation occurs in this fixture.
  run.completed=count==Steps(run.config);
  if(!ValidateRun(run,run.completed,error)) throw std::runtime_error("Synthetic comparison record rejected: "+error);
  return run;
}
inline std::array<Run,3> Sequence(unsigned cells=1) {
  return {Synthetic(1,cells),Synthetic(2,cells),Synthetic(4,cells)};
}
} // namespace tl::qualification::qeph::wall_response::comparison_test
