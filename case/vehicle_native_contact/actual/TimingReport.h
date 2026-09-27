#pragma once
#include "case/vehicle_run/TimingWindow.h"
#include "output/BoundedArrayJson.h"
#include <cstdlib>
#include <optional>
namespace crash::cases::vehicle_native_contact::test {
inline std::optional<vehicle_run::TimingWindowRequest> RequestedTimingWindow() {
    const auto* first=std::getenv("ROBO_NATIVE_TIMING_FIRST_EPOCH");
    const auto* last=std::getenv("ROBO_NATIVE_TIMING_LAST_EPOCH");
    if(!first&&!last)return std::nullopt;
    auto read=[](const char* raw){
        output::Require(raw&&*raw,"Both timing endpoint epochs are required");
        const std::string text(raw);std::size_t used=0;
        output::Require(text.find_first_not_of("0123456789")==std::string::npos,"Timing epoch must be unsigned decimal");
        const auto value=std::stoull(text,&used);
        output::Require(used==text.size(),"Invalid timing epoch");return value;
    };
    return vehicle_run::TimingWindowRequest{read(first),read(last)};
}
inline void TimingReport(output::Document& doc,const vehicle_run::TimingWindowResult& result) {
    output::Document one;one.SetObject();
    output::String(one,"scope","steady-clock accepted-boundary wall interval; no scheduled frames or chunk flushes inside; includes controller and diagnostics");
    output::Boolean(one,"started",result.started);output::Boolean(one,"complete",result.complete);
    output::Integer(one,"requested_first_epoch",result.requested.first_epoch);
    output::Integer(one,"requested_last_epoch",result.requested.last_epoch);
    if(result.started){output::Integer(one,"first_epoch",result.first.epoch);
        output::Number(one,"first_time_s",result.first.time_s);output::Number(one,"first_elapsed_s",result.first_elapsed_s);}
    if(result.complete){output::Integer(one,"last_epoch",result.last.epoch);
        output::Number(one,"last_time_s",result.last.time_s);output::Number(one,"last_elapsed_s",result.last_elapsed_s);
        output::Integer(one,"intervals",result.measured_intervals);output::Number(one,"wall_s",result.wall_s);}
    output::array_json::Child(doc,"interior_timing",std::move(one));
}
} // namespace crash::cases::vehicle_native_contact::test
