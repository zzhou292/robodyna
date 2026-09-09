#include "CouponProfileData.h"

#include <algorithm>
#include <numeric>

namespace crash::benchmarks::profile_detail {
namespace {
using namespace output;

Document Object() { Document doc; doc.SetObject(); return doc; }
void Put(Document& parent, const char* name, const Document& child) {
    Value value(child, parent.GetAllocator());
    parent.AddMember(Value(name, parent.GetAllocator()), value, parent.GetAllocator());
}
void Append(Value& array, Document& owner, const Document& child) {
    Value value(child, owner.GetAllocator());
    array.PushBack(value, owner.GetAllocator());
}
Document Statistics(std::vector<double> values) {
    auto doc = Object();
    Integer(doc, "count", values.size());
    if (values.empty()) {
        String(doc, "status", "not_observed");
        return doc;
    }
    for (double value : values) Require(value >= 0, "Negative profile duration");
    std::sort(values.begin(), values.end());
    const auto count = values.size();
    const double median = count % 2 ? values[count / 2] :
                          .5 * (values[count / 2 - 1] + values[count / 2]);
    const double sum = std::accumulate(values.begin(), values.end(), 0.0);
    String(doc, "status", "observed");
    Number(doc, "minimum_s", values.front());
    Number(doc, "maximum_s", values.back());
    Number(doc, "median_s", median);
    Number(doc, "mean_s", sum / count);
    Number(doc, "sum_s", sum);
    return doc;
}
template <typename Predicate>
std::vector<double> SelectSteps(const Data& data, Predicate select) {
    std::vector<double> values;
    for (const auto& step : data.steps) if (select(step)) values.push_back(step.seconds);
    return values;
}
Document Configuration(const Data& data) {
    auto doc = Object();
    Integer(doc, "warmup_steps", data.config.warmup_steps);
    Integer(doc, "steps_per_repetition", data.config.steps_per_repeat);
    Integer(doc, "measured_repetitions", data.config.repetitions);
    Integer(doc, "total_requested_steps", data.steps.size());
    Integer(doc, "step_hard_cap", 1000);
    Integer(doc, "fixed_refinement", 1);
    Integer(doc, "diagnostic_intervals", data.audit_intervals);
    Integer(doc, "accepted_b2_diagnostic_intervals", 20);
    Integer(doc, "derived_audit_stride_steps",
            (data.modal.step_count + data.audit_intervals - 1) / data.audit_intervals);
    Integer(doc, "node_count", data.final.stamp.node_count);
    Integer(doc, "element_count", reference::kCouponElements);
    Number(doc, "fixed_dt_s", data.final.stamp.fixed_dt);
    Integer(doc, "full_horizon_steps", data.final.required_steps);
    Number(doc, "full_horizon_s", data.modal.horizon);
    Boolean(doc, "full_horizon_executed", false);
    String(doc, "audit_classification", "observed increment of full_state_audit_reads across each successful Step");
    String(doc, "capture_cadence", "initial accepted state and after each measured repetition");
    return doc;
}
Document Admission(const Data& data) {
    using Limits = case_data::ElasticCouponLimits;
    auto doc = Object();
    String(doc, "kind", "RestrictedElasticTrajectory");
    Integer(doc, "qualification_id", data.final.diagnostics.configuration_id);
    Boolean(doc, "case_thresholds_modified", false);
    Number(doc, "maximum_step_s", data.modal.proposed_step_limit);
    Number(doc, "spectral_step_limit_s", data.modal.spectral_step_limit);
    Number(doc, "wave_step_limit_s", data.modal.wave_step_limit);
    Number(doc, "rotary_step_limit_s", data.modal.rotary_step_limit);
    Number(doc, "monitored_operator_norm_limit_per_s2", data.modal.monitored_norm_limit);
    Number(doc, "maximum_displacement_m", Limits::displacement);
    Number(doc, "maximum_director_departure_rad", Limits::director_departure);
    Number(doc, "maximum_pair_angle_rad", Limits::pair_angle);
    Number(doc, "maximum_rotation_increment_rad", Limits::rotation_increment);
    Number(doc, "maximum_membrane_strain", Limits::strain);
    Number(doc, "maximum_thickness_curvature", Limits::thickness_curvature);
    Number(doc, "minimum_area_ratio", Limits::minimum_area_ratio);
    Number(doc, "maximum_area_ratio", Limits::maximum_area_ratio);
    Number(doc, "maximum_relative_total_energy_error", Limits::energy_fraction);
    return doc;
}
Document FinalMetrics(const Data& data) {
    const auto& metrics = data.final;
    const auto& stamp = metrics.stamp;
    const auto& d = metrics.diagnostics;
    auto doc = Object();
    Integer(doc, "owner_id", stamp.owner_id);
    Integer(doc, "epoch", stamp.epoch);
    Number(doc, "accepted_time_s", stamp.time);
    Number(doc, "fixed_dt_s", stamp.fixed_dt);
    Integer(doc, "diagnostics_base_epoch", d.base_epoch);
    Integer(doc, "diagnostics_attempt", d.attempt);
    Integer(doc, "configuration_id", d.configuration_id);
    Boolean(doc, "diagnostics_valid", d.valid);
    String(doc, "diagnostics_phase", "prepared_candidate_committed_by_case");
    Integer(doc, "full_state_audit_reads", metrics.full_state_audit_reads);
    Integer(doc, "last_operator_epoch", metrics.last_operator_epoch);
    Number(doc, "last_operator_norm_per_s2", metrics.last_operator_norm);
    Number(doc, "initial_total_energy_j", metrics.initial_energy);
    Number(doc, "elastic_energy_j", d.elastic_energy);
    Number(doc, "bending_energy_j", d.bending_energy);
    Number(doc, "kinetic_translation_j", d.kinetic_translation);
    Number(doc, "kinetic_physical_rotation_j", d.kinetic_physical_rotation);
    Number(doc, "kinetic_artificial_drilling_j", d.kinetic_artificial_drilling);
    Number(doc, "total_energy_j", d.elastic_energy + case_data::CouponKineticEnergy(d));
    Number(doc, "maximum_relative_total_energy_error", metrics.maximum_relative_energy_error);
    Number(doc, "maximum_displacement_m", d.maximum_displacement);
    Number(doc, "maximum_director_departure_rad", d.maximum_director_departure);
    Number(doc, "maximum_pair_angle_rad", d.maximum_pair_angle);
    Number(doc, "maximum_membrane_strain", d.maximum_membrane_strain);
    Number(doc, "maximum_thickness_curvature", d.maximum_thickness_curvature);
    Number(doc, "minimum_signed_area_ratio", d.minimum_signed_area_ratio);
    Number(doc, "minimum_area_norm_ratio", d.minimum_area_norm_ratio);
    Number(doc, "maximum_area_norm_ratio", d.maximum_area_norm_ratio);
    Number(doc, "minimum_display_triangle_area_ratio", d.minimum_display_triangle_area_ratio);
    Number(doc, "base_elastic_energy_j", d.base_elastic_energy);
    Number(doc, "base_kinetic_energy_j", d.base_kinetic_energy);
    Number(doc, "elastic_energy_increment_j", d.elastic_energy_increment);
    Number(doc, "kinetic_energy_increment_j", d.kinetic_energy_increment);
    Number(doc, "kinetic_midpoint_work_j", d.kinetic_midpoint_work);
    Number(doc, "kinetic_work_residual_j", d.kinetic_work_residual);
    Number(doc, "force_coordinate_work_j", d.force_coordinate_work);
    Number(doc, "conservative_force_coordinate_defect_j", d.conservative_force_coordinate_defect);
    Number(doc, "mass_weighted_increment_squared_kg_m2", d.mass_weighted_increment_squared);
    return doc;
}
Document Timings(const Data& data) {
    auto doc = Object();
    Put(doc, "warmup_steps", Statistics(SelectSteps(data, [](const Step& s) { return s.warmup; })));
    Put(doc, "measured_steps", Statistics(SelectSteps(data, [](const Step& s) { return !s.warmup; })));
    const auto audited = SelectSteps(data, [](const Step& s) { return !s.warmup && s.audited; });
    Put(doc, "measured_audit_bearing_steps", Statistics(audited));
    Put(doc, "measured_ordinary_steps", Statistics(SelectSteps(data, [](const Step& s) { return !s.warmup && !s.audited; })));
    Boolean(doc, "measured_audit_bearing_step_observed", !audited.empty());
    std::vector<double> captures;
    for (const auto& capture : data.captures) captures.push_back(capture.seconds);
    Put(doc, "explicit_capture_calls", Statistics(captures));
    Value repetitions(rapidjson::kArrayType);
    for (unsigned i = 1; i <= data.config.repetitions; ++i) {
        auto item = Object();
        Integer(item, "repetition", i);
        Put(item, "all_steps", Statistics(SelectSteps(data, [i](const Step& s) { return !s.warmup && s.repetition == i; })));
        Put(item, "audit_bearing_steps", Statistics(SelectSteps(data, [i](const Step& s) { return !s.warmup && s.repetition == i && s.audited; })));
        Append(repetitions, doc, item);
    }
    doc.AddMember("repetitions", repetitions, doc.GetAllocator());
    return doc;
}
}  // namespace

Document Report(const Data& data) {
    auto doc = Object();
    String(doc, "schema", "robo-dyna.elastic-coupon-profile.v1");
    String(doc, "status", "completed_profile");
    String(doc, "scope", "P0 public-API prefix of the synthetic two-Q4 elastic coupon; profile evidence only");
    String(doc, "clock", "std::chrono::steady_clock wall seconds; no extra synchronization around Step");
    Boolean(doc, "accepted_trajectory_bundle", false);
    Put(doc, "configuration", Configuration(data));
    Put(doc, "unchanged_admission", Admission(data));
    Put(doc, "final_accepted_metrics", FinalMetrics(data));
    Put(doc, "timings", Timings(data));
    auto cuda = Object();
    Integer(cuda, "visible_device_ordinal", data.device);
    String(cuda, "device_name", data.device_name);
    Integer(cuda, "runtime_version", data.runtime_version);
    Integer(cuda, "driver_version", data.driver_version);
    Put(doc, "cuda", cuda);

    Value phases(rapidjson::kArrayType), memory(rapidjson::kArrayType);
    Value steps(rapidjson::kArrayType), captures(rapidjson::kArrayType);
    for (const auto& phase : data.phases) {
        auto item = Object();
        String(item, "phase", phase.name);
        Number(item, "wall_s", phase.seconds);
        Append(phases, doc, item);
    }
    for (const auto& sample : data.memory) {
        auto item = Object();
        String(item, "phase", sample.phase);
        Boolean(item, "owner_initialized", sample.owner_initialized);
        Integer(item, "epoch", sample.epoch);
        Integer(item, "device_wide_free_bytes", sample.free_bytes);
        Integer(item, "device_wide_total_bytes", sample.total_bytes);
        Integer(item, "device_wide_used_bytes", sample.total_bytes - sample.free_bytes);
        Integer(item, "owned_state_bytes", sample.owned_state_bytes);
        Integer(item, "owned_element_bytes", sample.owned_element_bytes);
        Integer(item, "owned_state_allocations", sample.state_allocations);
        Integer(item, "owned_element_allocations", sample.element_allocations);
        Number(item, "cuda_mem_get_info_wall_s", sample.query_seconds);
        Append(memory, doc, item);
    }
    for (const auto& step : data.steps) {
        auto item = Object();
        Integer(item, "epoch", step.epoch);
        Number(item, "accepted_time_s", step.time);
        Integer(item, "repetition", step.repetition);
        Boolean(item, "warmup", step.warmup);
        Boolean(item, "audit_bearing", step.audited);
        Number(item, "step_wall_s", step.seconds);
        Append(steps, doc, item);
    }
    for (const auto& capture : data.captures) {
        auto item = Object();
        Integer(item, "epoch", capture.epoch);
        Number(item, "accepted_time_s", capture.time);
        Number(item, "capture_wall_s", capture.seconds);
        Append(captures, doc, item);
    }
    doc.AddMember("cold_phases", phases, doc.GetAllocator());
    doc.AddMember("memory_samples", memory, doc.GetAllocator());
    doc.AddMember("step_samples", steps, doc.GetAllocator());
    doc.AddMember("capture_samples", captures, doc.GetAllocator());
    Value limitations(rapidjson::kArrayType);
    for (const char* text : {
             "One owner and one cold lifecycle; measured repetitions are consecutive windows of one changing trajectory.",
             "Step includes accepted assembly, integration, candidate checks, synchronization and commit; isolated kernel/candidate/sync timings are unavailable through this API.",
             "Audit-bearing Step includes CPU spectral audit and full-state copies; its time is not an isolated audit measurement or a baseline-subtracted estimate.",
             "case_initialize combines CPU modal audit, owner and batch initialization, first assembly, and initial mesh publication.",
             "cudaFree(nullptr) is the first CUDA API here; a fresh guarded CLI process is required for cold interpretation. Metadata queries follow before the first memory sample.",
             "cudaMemGetInfo reports device-wide observations, including runtime and other users; sparse samples are neither owned allocations nor a memory peak. The external guard supplies the pre-process baseline.",
             "Capture includes device readback and accepted mesh publication, not artifact file serialization or rendering. It changes no accepted epoch/time.",
             "Profiling uses diagnostic_intervals=64; accepted B2 uses 20. A short custom request may observe no audit-bearing measured step.",
             "Warmup is excluded from measured groups. No timing assertions, performance forecasts or new physics qualification are made."})
        limitations.PushBack(Value(text, doc.GetAllocator()), doc.GetAllocator());
    doc.AddMember("limitations", limitations, doc.GetAllocator());
    return doc;
}
}  // namespace crash::benchmarks::profile_detail
