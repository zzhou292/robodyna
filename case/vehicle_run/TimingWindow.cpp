#include "TimingWindow.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace crash::cases::vehicle_run {
TimingWindow::TimingWindow(TimingWindowRequest request,std::uint64_t planned,
    const std::vector<std::uint64_t>& samples,std::uint64_t rows_per_chunk) {
    if(!request.first_epoch||request.first_epoch>=request.last_epoch||request.last_epoch>=planned||
       !rows_per_chunk||samples.size()<2||samples.front()!=0||samples.back()!=planned||
       !std::is_sorted(samples.begin(),samples.end())||std::adjacent_find(samples.begin(),samples.end())!=samples.end())
        throw std::invalid_argument("Timing window requires an interior epoch range and complete archive schedule");
    // Timestamp is after the first endpoint's Append/sample; output at that
    // endpoint is excluded. Output at the last endpoint would be included.
    const auto frame=std::upper_bound(samples.begin(),samples.end(),request.first_epoch);
    if((frame!=samples.end()&&*frame<=request.last_epoch)||
       request.last_epoch/rows_per_chunk!=request.first_epoch/rows_per_chunk)
        throw std::invalid_argument("Timing window contains a saved frame or interval-chunk write");
    result_.requested=request;result_.planned_intervals=planned;
}
void TimingWindow::Observe(const Progress& value) {
    const auto endpoint=value.accepted;
    if(value.planned_intervals!=result_.planned_intervals||endpoint.epoch!=next_epoch_||
       endpoint.epoch>=result_.planned_intervals||!std::isfinite(endpoint.time_s)||endpoint.time_s<0||
       !std::isfinite(value.elapsed_s)||value.elapsed_s<0||
       (next_epoch_==0?endpoint.time_s!=0:(endpoint.time_s<=previous_.time_s||value.elapsed_s<previous_elapsed_)))
        throw std::invalid_argument("Timing boundary skipped, repeated, changed horizon or moved backwards");
    auto next=result_;
    if(endpoint.epoch==next.requested.first_epoch){
        next.started=true;next.first=endpoint;next.first_elapsed_s=value.elapsed_s;
    }
    if(endpoint.epoch==next.requested.last_epoch){
        if(!next.started||value.elapsed_s<=next.first_elapsed_s)
            throw std::invalid_argument("Timing window has no positive finite measured duration");
        next.complete=true;next.last=endpoint;next.last_elapsed_s=value.elapsed_s;
        next.wall_s=next.last_elapsed_s-next.first_elapsed_s;
        next.measured_intervals=next.last.epoch-next.first.epoch;
    }
    result_=next;previous_=endpoint;previous_elapsed_=value.elapsed_s;++next_epoch_;
}
} // namespace crash::cases::vehicle_run
