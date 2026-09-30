#pragma once
#include "RunArchive.h"
#include "IntervalIO.h"
namespace crash::output::physical_run {
struct RunArchive::Data {
    Data(std::filesystem::path r,const records::Context& c,physical_frames::Archive a,Configuration config,Forecast f)
        :root(std::move(r)),context(c),frames(std::move(a)),configuration(std::move(config)),forecast(std::move(f)) {}
    std::filesystem::path root;
    records::Context context;
    physical_frames::Archive frames;
    Configuration configuration;
    Forecast forecast;
    std::unique_ptr<IntervalWriter> intervals;
    Index index;
    Manifest manifest;
    cases::vehicle_wall::SetupIdentity wall;
    bool failed=false,closed=false,prefix_sample=false;
};
namespace detail {
void ValidateRequest(const records::PlanRequest&,Profile,bool wall,bool environment=false);
Forecast ForecastRun(const records::Context&,const physical_frames::Archive&,Profile,Limits,bool wall);
}
} // namespace crash::output::physical_run
