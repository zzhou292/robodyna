#include "Storage.h"
#include "output/ArtifactIO.h"
#include <cuda_runtime_api.h>
namespace crash::cases::vehicle_native_contact {
namespace {
struct Stream {
    cudaStream_t value = nullptr;
    Stream() { output::Require(cudaStreamCreate(&value) == cudaSuccess, "Source census CUDA stream creation failed"); }
    ~Stream() { if (value) { cudaStreamSynchronize(value); cudaStreamDestroy(value); } }
    Stream(const Stream&) = delete;
    Stream& operator=(const Stream&) = delete;
};
}
std::array<InitialCensus, 2> VehicleContactStartup::CensusInitialStates() const {
    const auto& data = *data_;
    output::Require(data.forecast.census_peak_host_bytes <= data.config.dynamics.startup.limits.host_bytes &&
                        data.forecast.census_peak_device_bytes <= data.config.peak_device_bytes,
                    "Source-only initial census exceeds the explicit case limits");
    Stream stream;
    std::array<InitialCensus, 2> result;
    std::size_t ordinal = 0;
    for (const auto& entry : data.forecast.sources.interfaces) {
        const auto i = detail::RoleIndex(entry.role);
        n::initial_source::DeviceSeed seed;
        const auto report = n::initial_source::Prepare(data.prepared[i], stream.value, seed);
        if (report.status != n::initial_source::Status::Ok)
            throw PreparationError(entry.role == Role::Self ? "Self source census" : "Wall source census", report);
        output::Require(seed.prepared(), "Successful source census has no genuine seed");
        result[ordinal++] = {entry.role, seed.identity(), report};
        // Prepare drained. This seed retires before the next interface starts;
        // GeneralInitialize will independently compute on the real owner stream.
    }
    return result;
}
} // namespace crash::cases::vehicle_native_contact
