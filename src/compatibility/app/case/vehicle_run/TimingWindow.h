#pragma once
#include "Progress.h"
#include <vector>
namespace crash::cases::vehicle_run {
struct TimingWindowRequest {std::uint64_t first_epoch=0,last_epoch=0;};
struct TimingWindowResult {
    TimingWindowRequest requested;
    std::uint64_t planned_intervals=0,measured_intervals=0;
    bool started=false,complete=false;
    Endpoint first,last;
    double first_elapsed_s=0,last_elapsed_s=0,wall_s=0;
};
// Interior accepted-boundary wall window. The run clock supplies elapsed values;
// this observer does not time CUDA stages, execute physics or write output.
class TimingWindow {
  public:
    TimingWindow(TimingWindowRequest,std::uint64_t planned_intervals,
        const std::vector<std::uint64_t>& saved_frame_epochs,std::uint64_t interval_rows_per_chunk);
    // Bind to Control.accepted_boundary before Execute; every boundary is needed.
    // Invalid input throws before changing this observer's retained values.
    void Observe(const Progress&);
    const TimingWindowResult& result()const noexcept{return result_;}
  private:
    TimingWindowResult result_;
    std::uint64_t next_epoch_=0;
    Endpoint previous_;
    double previous_elapsed_=0;
};
} // namespace crash::cases::vehicle_run
