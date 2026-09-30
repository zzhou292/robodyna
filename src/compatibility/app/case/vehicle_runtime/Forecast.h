#pragma once
#include "Config.h"
#include "SourceIdentity.h"
#include "Source.h"
#include <array>
namespace crash::cases::vehicle_startup::joints { class VehicleJointModel; }
namespace crash::cases::vehicle_runtime {
struct Forecast {
    // Previous source construction bounds are reported separately. Their retired
    // scratch and module caps are not new runtime allocations or additive RSS.
    std::array<std::size_t,3> prior_construction_bytes{}; // model, execution, attachments
    std::size_t retained_source_upper_bound = 0, app_fixed_bytes = 0, packing_bytes = 0;
    std::size_t retained_host_upper_bound = 0, peak_temporary_bytes = 0, readback_temporary_bytes = 0;
    std::size_t peak_host_upper_bound = 0, device_bytes = 0;
    tl::fea::NodalAssemblyCinForecast owner;
    std::array<tl::fea::ShellMappedFootprint,6> participants{}; // Q,T,B,TYPE25,TYPE13,solids
    // Optional seventh constraint contributor; complete immutable source is
    // conservatively reserved separately from the existing shared shell source.
    bool has_type45 = false;
    std::size_t joint_source_bytes = 0;
    tl::fea::type45::BatchForecast joints;
    tl::fea::ShellPhysicalPublicationForecast publisher;
    bool has_beam18 = false;
    tl::fea::beam18::BatchForecast structural_beams;
    std::size_t structural_beam_incremental_host_bytes = 0;
    unsigned solid_packet_blocks = 0;
    std::size_t solid_worker_slots = 0;
};
namespace detail {
std::size_t SourceBytes(const Execution&,const Attachments&,std::size_t cap);
Forecast ForecastStartup(const Config&,const Source&,std::size_t fixed_bytes);
Forecast ForecastStartup(const Config&,const Execution&,const Attachments&,std::size_t fixed_bytes,
    const vehicle_startup::joints::VehicleJointModel* = nullptr);
} // namespace detail
} // namespace crash::cases::vehicle_runtime
