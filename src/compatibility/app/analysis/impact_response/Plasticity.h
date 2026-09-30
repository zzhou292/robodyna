#pragma once

#include "ParentCatalog.h"
#include "output/physical_run/Replay.h"
#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace crash::analysis::impact_response {

struct AnalysisLimits {
    std::size_t samples = 10000;
    std::size_t parts = 65536;
    std::size_t host_bytes = 256u << 20;
    std::size_t report_bytes = 16u << 20;
    std::size_t incident_relations = 1u << 20;
};

struct SampleStamp {
    std::uint64_t epoch = 0;
    std::uint64_t attempt = 0;
    double time_s = 0;
};

struct Occurrence {
    bool available = false;
    SampleStamp stamp;
    bool parent_active = false;
    std::array<double, 3> centroid_m{};
};

struct Witness {
    bool available = false;
    std::size_t parent_index = SIZE_MAX;
    double value = 0;
    Occurrence occurrence;
};

struct SpatialBounds {
    bool available = false;
    std::array<double, 3> low_m{};
    std::array<double, 3> high_m{};
};

struct ParentPlasticity {
    bool field_available = false;
    bool ever_positive = false;
    bool ever_inactive = false;
    bool active_at_final_sample = false;
    std::size_t ever_positive_points = 0;
    std::size_t peak_positive_points = 0;
    std::size_t final_positive_points = 0;
    double first_positive_strain = 0;
    double peak_strain = 0;
    double final_max_strain = 0;
    Occurrence first_positive;
    Occurrence peak;
    std::array<double, 3> reference_centroid_m{};
    std::array<double, 3> final_centroid_m{};
};

struct SamplePlasticity {
    SampleStamp stamp;
    std::size_t active_parents = 0;
    std::size_t positive_points = 0;
    std::size_t active_positive_points = 0;
    std::size_t inactive_positive_points = 0;
    std::size_t positive_parents = 0;
    std::size_t positive_parts = 0;
    Witness maximum;
};

struct PartPlasticity {
    std::uint64_t source_part = 0;
    std::uint64_t source_material = 0;
    std::uint64_t source_section = 0;
    std::uint32_t source_elform = 0;
    std::vector<std::uint32_t> native_families;
    std::size_t parents = 0;
    std::size_t field_parents = 0;
    std::size_t native_points = 0;
    std::size_t ever_positive_parents = 0;
    std::size_t ever_positive_points = 0;
    std::size_t final_positive_parents = 0;
    std::size_t final_positive_points = 0;
    std::size_t ever_inactive_parents = 0;
    double peak_strain = 0;
    double final_max_strain = 0;
    Witness first_positive;
    Witness peak;
    SpatialBounds yielded_reference_centroid_bounds;
    SpatialBounds yielded_final_centroid_bounds;
};

struct PlasticityResult {
    std::vector<ParentCatalogEntry> catalog;
    std::vector<ParentPlasticity> parents;
    std::vector<SamplePlasticity> samples;
    std::vector<PartPlasticity> parts;
    std::array<std::size_t, 5> native_point_layouts{};  // 0, 1, 3, 4, other.
    std::size_t available_parents = 0;
    std::size_t unavailable_parents = 0;
    std::size_t not_applicable_parents = 0;
    std::size_t native_points = 0;
    std::size_t ever_positive_points = 0;
    std::size_t ever_positive_parents = 0;
    std::size_t ever_positive_parts = 0;
    bool inactive_positive_history_observed = false;
    Witness first_positive;
    Witness peak;
};

// Incremental saved-state analysis. Observe consumes one authenticated sample
// at a time and retains only bounded aggregate state; it never stores the full
// archive or infers values between samples.
class PlasticityAccumulator {
  public:
    PlasticityAccumulator(const output::full_shell::Context&,
        std::vector<ParentCatalogEntry>, AnalysisLimits = {});
    void Observe(const output::physical_run::Sample&);
    PlasticityResult Finish();

  private:
    output::full_shell::Context context_;
    AnalysisLimits limits_;
    PlasticityResult result_;
    std::vector<std::uint8_t> point_ever_positive_;
    bool finished_ = false;
};

}  // namespace crash::analysis::impact_response
