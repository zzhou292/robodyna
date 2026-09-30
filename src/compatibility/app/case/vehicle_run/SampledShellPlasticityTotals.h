#pragma once
#include <cstdint>

namespace crash::cases::vehicle_run {

// Observations of successfully saved frames only. These scalars do not locate
// first yield between samples or supply missing solid/beam material histories.
struct SampledShellPlasticityTotals {
    bool available = false;
    bool native_fields_available = false;
    bool positive_sample_observed = false;
    std::uint64_t saved_samples = 0;
    std::uint64_t last_epoch = 0;
    std::uint64_t last_attempt = 0;
    double last_time_s = 0;
    std::uint64_t native_points = 0;
    std::uint64_t last_positive_points = 0;
    double last_max_native_equivalent_plastic_strain = 0;
    double peak_saved_native_equivalent_plastic_strain = 0;
    std::uint64_t first_positive_saved_epoch = 0;
    double first_positive_saved_time_s = 0;
};

static_assert(sizeof(SampledShellPlasticityTotals) <= 256,
    "Sampled scalars must fit the existing bounded controller reserve");

} // namespace crash::cases::vehicle_run
