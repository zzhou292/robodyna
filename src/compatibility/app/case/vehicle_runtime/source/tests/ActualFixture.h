#pragma once
#include "../../VehiclePhysicalStartup.h"
#include "case/vehicle_dynamics/VehiclePhysicalDynamics.h"
#include "case/vehicle_wall/native/execution/tests/ActualFixture.h"
#include "output/ArtifactIO.h"
#include <algorithm>
#include <cstdlib>
#include <filesystem>
namespace crash::cases::vehicle_runtime::source_test {
namespace e=vehicle_wall::native::execution_test;
constexpr std::size_t GuardBytes=std::size_t{10}<<30;
constexpr std::size_t ExportBytes=2u<<20;
inline std::size_t Add(std::size_t a,std::size_t b) {
    output::Require(b<=SIZE_MAX-a,"Runtime source qualification forecast overflow");return a+b;
}
inline const vehicle_wall::native::EnvelopeOwnerSource& OwnerSource() {
    static const auto value=[] {
        const auto& execution=e::Executed();const auto& post=e::Post();const auto& joints=e::JointSource();
        const auto f=vehicle_wall::native::EnvelopeOwnerSource::Preflight(execution,post,joints);
        const auto coexist=Add(Add(execution.forecast().total_bytes,post.forecast().total_host_bytes),joints.forecast().total_bytes);
        output::Require(Add(std::max(f.peak_bytes,coexist),ExportBytes)<=GuardBytes,
            "Source construction exceeds unchanged qualification guard");
        return vehicle_wall::native::EnvelopeOwnerSource::Prepare(execution,post,joints);
    }();
    return value;
}
inline Config RuntimeConfig() {
    Config c;c.reserved_step_s=2e-7;return c;
}
inline vehicle_dynamics::Config DynamicsConfig() {
    vehicle_dynamics::Config c;c.startup=RuntimeConfig();
    c.structural={tl::fea::NodalCinStructuralProfile::NativeOrdinaryRigidTrace,.8,true};return c;
}
inline std::size_t Qualification(const Forecast& f) {
    auto peak=f.peak_host_upper_bound;
    for(const auto old:f.prior_construction_bytes)peak=std::max(peak,old);
    return Add(peak,ExportBytes);
}
inline std::filesystem::path Destination() {
    const char* name=std::getenv("ROBO_ENVELOPE_RUNTIME_OUTPUT");
    output::Require(name&&*name,"Missing create-only runtime qualification output");
    std::filesystem::path path(name);
    output::Require(std::filesystem::create_directory(path),"Runtime qualification output already exists");
    return path;
}
inline void ForecastFields(output::Document& doc,const Forecast& f) {
    output::Integer(doc,"retained_source_bytes",f.retained_source_upper_bound);
    output::Integer(doc,"retained_runtime_bytes",f.retained_host_upper_bound);
    output::Integer(doc,"runtime_peak_bytes",f.peak_host_upper_bound);
    output::Integer(doc,"qualification_peak_bytes",Qualification(f));
    output::Integer(doc,"exact_device_bytes",f.device_bytes);
    output::Boolean(doc,"fits_unchanged_host_guard",Qualification(f)<=GuardBytes);
}
}
