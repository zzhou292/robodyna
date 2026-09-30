#pragma once
#include "Config.h"
#include "Progress.h"
#include <vector>
namespace crash::cases::vehicle_run::detail {
// Private orchestration seam. Production always supplies the real loaded owner,
// authenticated interval factory and bounded archive. Tests inject call faults.
class Operations {
  public:
    virtual ~Operations()=default;
    virtual Endpoint Accepted() const noexcept=0;
    virtual ContactTotals Contact() const noexcept {return {};}
    virtual SelfContactTotals SelfContact() const noexcept {return {};}
    virtual MechanicsTotals Mechanics() const noexcept {return {};}
    virtual SampledShellPlasticityTotals SampledShellPlasticity() const noexcept {return {};}
    virtual vehicle_dynamics::StepTimingSnapshot MechanicsTiming() const noexcept {return {};}
    virtual void Prepare()=0;
    virtual void Commit()=0;
    virtual void Discard() noexcept=0;
    virtual void Append()=0;
    virtual void Capture()=0;
    virtual void SaveSample()=0;
    virtual void Finish(bool complete,const std::string& reason)=0;
};
using Clock=std::function<double()>;
void ValidateLoop(const Horizon&,const std::vector<std::uint64_t>&,const Control&);
LoopResult RunLoop(Operations&,const Horizon&,const std::vector<std::uint64_t>& sample_epochs,
    const Control&,const Clock&);
} // namespace crash::cases::vehicle_run::detail
