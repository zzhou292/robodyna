#include "Loop.h"
#include <algorithm>
#include <cmath>
#include <exception>
#include <stdexcept>
namespace crash::cases::vehicle_run::detail {
namespace {
std::string Message() {
    try {throw;}
    catch(const std::exception& error) {return std::string(error.what()).substr(0,4096);}
    catch(...) {return "Non-standard exception from a run operation or observer";}
}
}
void ValidateLoop(const Horizon& plan,const std::vector<std::uint64_t>& samples,const Control& control) {
    if(!plan.intervals || samples.size()<2 || samples.front()!=0 || samples.back()!=plan.intervals ||
        !std::is_sorted(samples.begin(),samples.end()) ||
        std::adjacent_find(samples.begin(),samples.end())!=samples.end() ||
        !std::isfinite(control.maximum_elapsed_s) || control.maximum_elapsed_s<0 ||
        !std::isfinite(control.progress_period_s) || control.progress_period_s<=0)
        throw std::invalid_argument("Run loop plan, sample schedule or control is invalid");
}
LoopResult RunLoop(Operations& operations,const Horizon& plan,const std::vector<std::uint64_t>& samples,
    const Control& control,const Clock& clock) {
    ValidateLoop(plan,samples,control);
    if(!clock || operations.Accepted().epoch!=0) throw std::invalid_argument("Run loop needs a fresh owner and clock");
    LoopResult result;
    auto& progress=result.progress;
    progress.planned_intervals=plan.intervals;
    const double start=clock();
    double last_progress=start;
    std::uint64_t sampled=0;
    bool have_sample=false;
    StopKind sample_failure=StopKind::CaptureFailure;
    auto refresh=[&]() {
        progress.accepted=operations.Accepted();
        progress.contact=operations.Contact();
        progress.mechanics_timing=operations.MechanicsTiming();
        progress.elapsed_s=clock()-start;
        progress.accepted_intervals_per_second=progress.elapsed_s>0?progress.accepted.epoch/progress.elapsed_s:0;
    };
    auto sample=[&]() {
        double begin=clock();
        sample_failure=StopKind::CaptureFailure;
        operations.Capture();
        progress.timing.capture_s+=clock()-begin;
        begin=clock();
        sample_failure=StopKind::ArchiveFailure;
        operations.SaveSample();
        progress.timing.archive_s+=clock()-begin;
        sampled=operations.Accepted().epoch;
        have_sample=true;
    };
    try {sample();}
    catch(...) {
        result.kind=sample_failure;
        result.reason=Message();
        refresh();
        return result;
    }
    while(operations.Accepted().epoch<plan.intervals) {
        refresh();
        try {
            if(control.maximum_accepted_intervals && progress.accepted.epoch>=control.maximum_accepted_intervals) {
                result.kind=StopKind::IntervalLimit;
                result.reason="Declared diagnostic accepted-interval limit reached";
                break;
            }
            if(control.stop_requested && control.stop_requested()) {
                result.kind=StopKind::Requested;
                result.reason="Cooperative stop requested";
                break;
            }
            if(control.maximum_elapsed_s>0 && progress.elapsed_s>=control.maximum_elapsed_s) {
                result.kind=StopKind::TimeLimit;
                result.reason="Cooperative elapsed-time limit reached";
                break;
            }
            if(control.progress && clock()-last_progress>=control.progress_period_s) {
                control.progress(progress);
                last_progress=clock();
            }
        } catch(...) {
            result.kind=StopKind::ObserverFailure;
            result.reason=Message();
            break;
        }
        try {
            double begin=clock();
            operations.Prepare();
            progress.timing.step_s+=clock()-begin;
            begin=clock();
            operations.Commit();
            progress.timing.commit_s+=clock()-begin;
        } catch(...) {
            operations.Discard();
            result.kind=StopKind::PhysicsRejected;
            result.reason=Message();
            break;
        }
        try {
            const double begin=clock();
            operations.Append();
            progress.timing.archive_s+=clock()-begin;
        } catch(...) {
            result.kind=StopKind::ArchiveFailure;
            result.reason=Message();
            refresh();
            return result; // Actual accepted state is ahead of the recorded ledger.
        }
        if(std::binary_search(samples.begin(),samples.end(),operations.Accepted().epoch)) {
            try {sample();}
            catch(...) {
                result.kind=sample_failure;
                result.reason=Message();
                refresh();
                return result;
            }
        }
    }
    refresh();
    const bool complete=progress.accepted.epoch==plan.intervals;
    if(complete) {
        result.kind=StopKind::Completed;
        result.reason.clear();
    }
    if(!have_sample || sampled!=progress.accepted.epoch) {
        try {sample();}
        catch(...) {
            result.kind=sample_failure;
            result.reason=Message();
            refresh();
            return result;
        }
    }
    try {
        const double begin=clock();
        operations.Finish(complete,result.reason);
        progress.timing.archive_s+=clock()-begin;
        result.valid_manifest=true;
    } catch(...) {
        result.kind=StopKind::ArchiveFailure;
        result.reason=Message();
    }
    refresh();
    return result;
}
} // namespace crash::cases::vehicle_run::detail
