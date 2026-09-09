#pragma once

// Internal bounded telemetry records and JSON presentation. No mechanics.
#include "CouponProfile.h"
#include "case/ElasticCouponCase.h"
#include <string>
#include <vector>

namespace crash::benchmarks::profile_detail {
struct Phase {
    std::string name;
    double seconds = 0;
};
struct Memory {
    std::string phase;
    bool owner_initialized = false;
    std::uint64_t free_bytes = 0, total_bytes = 0, owned_state_bytes = 0, owned_element_bytes = 0;
    std::uint64_t state_allocations = 0, element_allocations = 0, epoch = 0;
    double query_seconds = 0;
};
struct Step {
    bool warmup = false, audited = false;
    unsigned repetition = 0;
    std::uint64_t epoch = 0;
    double time = 0, seconds = 0;
};
struct Capture {
    std::uint64_t epoch = 0;
    double time = 0, seconds = 0;
};
struct Data {
    CouponProfileConfig config;
    std::string device_name;
    int device = 0, runtime_version = 0, driver_version = 0;
    unsigned audit_intervals = 64;
    std::vector<Phase> phases;
    std::vector<Memory> memory;
    std::vector<Step> steps;
    std::vector<Capture> captures;
    case_data::ElasticCouponMetrics final;
    reference::ElasticCouponModalReport modal;
};
output::Document Report(const Data&);
}  // namespace crash::benchmarks::profile_detail
