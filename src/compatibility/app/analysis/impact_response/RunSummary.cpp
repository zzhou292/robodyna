#include "Report.h"

#include "output/BoundedArrayJson.h"
#include <string_view>

namespace crash::analysis::impact_response {
namespace {

using output::Require;
using output::Value;

const Value& Member(const Value& object, const char* name) {
    Require(object.IsObject(), "Expected run-summary object");
    const Value* found = nullptr;
    for (auto it = object.MemberBegin(); it != object.MemberEnd(); ++it) {
        if (std::string_view(it->name.GetString(), it->name.GetStringLength()) == name) {
            Require(!found, "Duplicate run-summary field");
            found = &it->value;
        }
    }
    Require(found, "Missing run-summary field");
    return *found;
}

std::string Text(const Value& object, const char* name) {
    return output::array_json::Text(Member(object, name));
}

std::uint64_t Unsigned(const Value& object, const char* name) {
    return output::array_json::UInt(Member(object, name));
}

double Real(const Value& object, const char* name) {
    return output::array_json::Real(Member(object, name));
}

bool Flag(const Value& object, const char* name) {
    const auto& value = Member(object, name);
    Require(value.IsBool(), "Expected run-summary boolean");
    return value.GetBool();
}

void Same(double actual, double expected, const char* message) {
    Require(output::Bits(actual) == output::Bits(expected), message);
}

}  // namespace

SummaryEvidence VerifyRunSummary(const std::string& bytes,
    const RunMetadata& metadata, const PlasticityResult& result) {
    Require(bytes.size() == metadata.run_summary.bytes &&
            output::Sha256(bytes) == metadata.run_summary.sha256 &&
            !result.samples.empty(),
        "Impact analysis run-summary bytes or sampled result differ");
    const auto document =
        output::array_json::Parse(bytes, output::physical_run::MetadataCap);
    SummaryEvidence evidence;
    evidence.schema = Text(document, "schema");
    evidence.status = Text(document, "status");
    evidence.reason = Text(document, "reason");
    evidence.physical_profile = Text(document, "physical_profile");
    evidence.actual_completed_time_s = Real(document, "actual_completed_time_s");
    Require(evidence.schema == "robo_dyna.vehicle_run_summary.v1" &&
            Flag(document, "valid_archive_manifest") &&
            Unsigned(document, "accepted_intervals") == metadata.accepted_intervals &&
            evidence.reason == metadata.stop_reason &&
            (metadata.horizon_complete
                ? metadata.stop_reason.empty() : !metadata.stop_reason.empty()) &&
            Text(document, "archive_manifest_sha256") ==
                metadata.archive_manifest.sha256 &&
            Text(document, "viewer_input_sha256") ==
                metadata.viewer_input.sha256,
        "Impact analysis run summary differs from archive authority");
    Same(Real(document, "fixed_dt_s"), metadata.fixed_dt_s,
        "Impact analysis run-summary timestep differs");
    Same(evidence.actual_completed_time_s, result.samples.back().stamp.time_s,
        "Impact analysis run-summary endpoint differs");

    const auto& sampled = Member(document, "sampled_shell_plasticity");
    const auto& final = result.samples.back();
    Require(Text(sampled, "schema") == "robo_dyna.sampled_shell_plasticity.v1" &&
            Flag(sampled, "available") &&
            Unsigned(sampled, "saved_samples") == result.samples.size() &&
            Unsigned(sampled, "last_saved_epoch") == final.stamp.epoch &&
            Unsigned(sampled, "last_saved_attempt") == final.stamp.attempt &&
            Flag(sampled, "native_fields_available") ==
                (result.native_points != 0) &&
            Unsigned(sampled, "stored_native_points") == result.native_points &&
            Unsigned(sampled, "last_saved_positive_points") ==
                final.positive_points &&
            Flag(sampled, "positive_saved_sample_observed") ==
                result.first_positive.available,
        "Impact analysis sampled shell counts differ from run summary");
    Same(Real(sampled, "last_saved_time_s"), final.stamp.time_s,
        "Impact analysis last saved time differs");
    Same(Real(sampled, "last_saved_max_native_equivalent_plastic_strain"),
        final.maximum.available ? final.maximum.value : 0,
        "Impact analysis final shell maximum differs");
    Same(Real(sampled, "peak_saved_native_equivalent_plastic_strain"),
        result.peak.available ? result.peak.value : 0,
        "Impact analysis peak shell maximum differs");
    if (result.first_positive.available) {
        Require(Unsigned(sampled, "first_positive_saved_epoch") ==
                result.first_positive.occurrence.stamp.epoch,
            "Impact analysis first positive saved epoch differs");
        Same(Real(sampled, "first_positive_saved_time_s"),
            result.first_positive.occurrence.stamp.time_s,
            "Impact analysis first positive saved time differs");
    }
    Require(Unsigned(document, "last_accepted_active_parents") ==
            final.active_parents &&
            Flag(document, "contact_observations_available"),
        "Impact analysis final activity/contact summary differs");

    evidence.peak_wall_force_n = Real(document, "peak_observed_force_n");
    evidence.peak_wall_penetration_m =
        Real(document, "peak_observed_penetration_m");
    evidence.peak_same_mask_potential_j =
        Real(document, "peak_observed_same_mask_potential_j");
    evidence.last_same_mask_potential_j =
        Real(document, "last_same_mask_potential_j");
    evidence.last_removed_potential_j =
        Real(document, "last_removed_potential_j");
    Require(evidence.peak_wall_force_n >= 0 &&
            evidence.peak_wall_penetration_m >= 0 &&
            evidence.peak_same_mask_potential_j >= 0 &&
            evidence.last_same_mask_potential_j >= 0 &&
            evidence.last_removed_potential_j >= 0,
        "Impact analysis wall companion contains a negative magnitude");

    const auto& mechanics = Member(document, "accepted_mechanics");
    Require(Flag(mechanics, "available"),
        "Impact analysis accepted mechanics companion is unavailable");
    const auto& solids = Member(mechanics, "solids");
    const auto& solid_plastic =
        Member(solids, "included_law36_law44_plastic_work");
    evidence.solid_reported_plastic_work_sum_j =
        Real(solid_plastic, "accepted_increment_sum_j");
    const auto& beam = Member(mechanics, "beam18");
    const auto& beam_plastic = Member(beam, "included_plastic_work");
    evidence.beam_reported_plastic_work_sum_j =
        Real(beam_plastic, "accepted_increment_sum_j");
    Require(evidence.solid_reported_plastic_work_sum_j >= 0 &&
            evidence.beam_reported_plastic_work_sum_j >= 0,
        "Impact analysis reported plastic-work companion is negative");

    const auto& motion = Member(mechanics, "motion");
    const auto& translation = Member(motion, "translation_departure_m");
    evidence.translation_departure_last_m = Real(translation, "last");
    evidence.translation_departure_peak_m = Real(translation, "peak");
    const auto& velocity = Member(motion, "velocity_departure_m_s");
    evidence.velocity_departure_last_m_s = Real(velocity, "last");
    evidence.velocity_departure_peak_m_s = Real(velocity, "peak");
    const auto& orientation =
        Member(motion, "orientation_component_departure");
    evidence.orientation_component_departure_last =
        Real(orientation, "last");
    evidence.orientation_component_departure_peak =
        Real(orientation, "peak");
    const auto& spin = Member(motion, "spin_component_rad_s");
    evidence.spin_component_last_rad_s = Real(spin, "last");
    evidence.spin_component_peak_rad_s = Real(spin, "peak");
    Require(evidence.translation_departure_last_m >= 0 &&
            evidence.translation_departure_peak_m >= 0 &&
            evidence.velocity_departure_last_m_s >= 0 &&
            evidence.velocity_departure_peak_m_s >= 0 &&
            evidence.orientation_component_departure_last >= 0 &&
            evidence.orientation_component_departure_peak >= 0 &&
            evidence.spin_component_last_rad_s >= 0 &&
            evidence.spin_component_peak_rad_s >= 0,
        "Impact analysis motion companion contains a negative magnitude");
    return evidence;
}

}  // namespace crash::analysis::impact_response
