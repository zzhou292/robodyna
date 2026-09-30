#include "ComparisonInput.h"
#include "Metrics.h"

namespace crash::benchmarks::assembly_pilot {
namespace {
void Time(const Run& run,std::size_t i,double reference_step,std::uint64_t tick) {
    const auto& e=run.bundle.entries[i];const double common=reference_step*tick;
    Require(std::isfinite(common),"Pilot physical time overflows");
    // Integer/significand matching establishes the physical grid. Reuse the
    // reader's separate rounding allowance for the recorded accumulated clock.
    rd::CheckReplayTime(e.time,common,run.bundle.fixed_dt,e.epoch);
}
Value Pair(Document& d,const Run& x,const Run& y,std::size_t xi,std::size_t yi,double reference_step) {
    const auto xe=x.Epochs(),ye=y.Epochs();const auto common=CommonFrames(xe,x.multiple,ye,y.multiple);
    auto& a=d.GetAllocator();Value out(rapidjson::kObjectType),series(rapidjson::kArrayType),envelope(rapidjson::kObjectType);
    out.AddMember("reference_run",xi,a);out.AddMember("candidate_run",yi,a);out.AddMember("common_saved_frames",common.size(),a);
    const auto shared=std::min(Tick(xe.back(),x.multiple),Tick(ye.back(),y.multiple));
    const auto last=Tick(xe[common.back().first],x.multiple);
    out.AddMember("shared_accepted_horizon_s",shared*reference_step,a);out.AddMember("last_compared_time_s",last*reference_step,a);
    out.AddMember("shared_terminal_is_saved_in_both",last==shared,a);out.AddMember("has_positive_common_time",last>0,a);
    out.AddMember("reference_terminal_compared",last==Tick(xe.back(),x.multiple),a);
    out.AddMember("candidate_terminal_compared",last==Tick(ye.back(),y.multiple),a);
    std::map<std::string,std::pair<double,double>> maxima;
    for(const auto [i,j]:common) {
        const auto tick=Tick(xe[i],x.multiple);Time(x,i,reference_step,tick);Time(y,j,reference_step,tick);
        auto xf=x.Frame(i),yf=y.Frame(j);const auto metrics=Difference(xf,yf);
        Value sample(rapidjson::kObjectType);sample.AddMember("reference_tick",tick,a);sample.AddMember("time_s",tick*reference_step,a);
        sample.AddMember("reference_epoch",xe[i],a);sample.AddMember("candidate_epoch",ye[j],a);
        sample.AddMember("reference_recorded_time_s",x.bundle.entries[i].time,a);sample.AddMember("candidate_recorded_time_s",y.bundle.entries[j].time,a);
        sample.AddMember("metrics",MetricDocument(d,metrics),a);series.PushBack(sample,a);
        for(const auto& [name,m]:metrics)if(!maxima.count(name)||m.maximum>maxima[name].first)maxima[name]={m.maximum,tick*reference_step};
    }
    for(const auto& [name,p]:maxima) {
        Value v(rapidjson::kObjectType);v.AddMember("max_abs_difference",p.first,a);v.AddMember("first_time_at_max_s",p.second,a);
        envelope.AddMember(Value(name.c_str(),a),v,a);
    }
    out.AddMember("maximum_differences_over_common_samples",envelope,a);out.AddMember("series",series,a);return out;
}
}
Document Compare(const std::vector<std::filesystem::path>& directories) {
    Require(directories.size()>=2&&directories.size()<=3,"Pilot comparison needs a finest reference and one or two candidates");
    std::array<Run,3> runs;
    for(std::size_t i=0;i<directories.size();++i) {
        runs[i].Open(directories[i]);runs[i].multiple=StepMultiple(runs[0].bundle.fixed_dt,runs[i].bundle.fixed_dt);
        MatchRuns(runs[0],runs[i]);
    }
    Document d;d.SetObject();auto& a=d.GetAllocator();
    output::String(d,"schema","robo_dyna.source_assembly_pilot_comparison.v1");output::String(d,"status","reported");
    output::String(d,"scope","Validated original six-part component; internal groups active and external connections released");
    output::String(d,"assessment","Differences and accepted coverage only; convergence, accuracy and longer-horizon admission are not inferred");
    output::String(d,"time_matching","Exact binary64 rational step multiples and integer epoch ticks; no interpolation. Recorded clocks retain the owning reader's rounding checks");
    output::String(d,"stress_frame","Native corotational XX/YY/XY/YZ/ZX in each run's current source-shell axes; not world-tensor stress differences");
    output::String(d,"excluded","Carried midpoint v/omega/K, lagged frame kinetic totals, interval work increments, rate/power and force-stage kinetic are not compared as endpoint quantities");
    output::String(d,"work_policy","Cumulative native/plastic work are separate diagnostic channels; no reaction-as-dissipation or additional total-energy formula");
    output::String(d,"index_policy","Node index is global source-node index; layer flat index is 3*source_parent_index+layer, bottom/middle/top; parent index follows source inventory");
    output::Number(d,"reference_dt_s",runs[0].bundle.fixed_dt);
    Value summaries(rapidjson::kArrayType),pairs(rapidjson::kArrayType);
    for(std::size_t i=0;i<directories.size();++i)summaries.PushBack(RunSummary(d,runs[i]),a);
    for(std::size_t i=0;i<directories.size();++i)for(std::size_t j=i+1;j<directories.size();++j)
        pairs.PushBack(Pair(d,runs[i],runs[j],i,j,runs[0].bundle.fixed_dt),a);
    d.AddMember("runs",summaries,a);d.AddMember("comparisons",pairs,a);return d;
}
} // namespace crash::benchmarks::assembly_pilot
