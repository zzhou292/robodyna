#include "../SelfContactDocument.h"
#include "case/vehicle_self_contact/SelfContactStageError.h"
#include "chrono_thirdparty/rapidjson/stringbuffer.h"
#include "chrono_thirdparty/rapidjson/writer.h"

#include <gtest/gtest.h>
#include <limits>

namespace crash::cases::vehicle_run::test {
namespace {

namespace contact = tlfea::contact;
namespace runtime = vehicle_self_contact;

output::Document Encode(const contact::SelfContactTransactionReport& report) {
    const auto document = detail::SelfContactErrorDocument(
        runtime::SelfContactStageError(report,
            runtime::SelfContactRuntimeStage::CandidateSeal, 65536));
    rapidjson::StringBuffer buffer;
    rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
    EXPECT_TRUE(document.Accept(writer));
    EXPECT_LT(buffer.GetSize(), 16u << 10);
    output::Document decoded;
    decoded.Parse<rapidjson::kParseFullPrecisionFlag>(buffer.GetString());
    EXPECT_FALSE(decoded.HasParseError());
    return decoded;
}

}  // namespace

TEST(VehicleRunSelfContactError, OptionalFacetFilterStatusSurvivesWithoutInventingDefaultFailure) {
    contact::SelfContactTransactionReport report;
    EXPECT_FALSE(Encode(report).HasMember("filter_status_code"));
    report.status=contact::SelfContactTransactionStatus::FacetFilterFailure;
    report.pair=13;
    report.message="Actual filter CUDA failure";
    report.filter_status=contact::self_contact_filters::Status::DeviceFailure;
    const auto document=Encode(report);
    EXPECT_EQ(document["status_code"].GetUint(),static_cast<unsigned>(report.status));
    EXPECT_EQ(document["filter_status_code"].GetUint(),static_cast<unsigned>(report.filter_status));
    EXPECT_EQ(document["pair_ordinal"].GetUint64(),13u);
    EXPECT_STREQ(document["message"].GetString(),report.message);
    EXPECT_FALSE(document.HasMember("native_crossing_failure"));
    EXPECT_FALSE(document.HasMember("required_force_events"));
}

TEST(VehicleRunSelfContactError, CandidateChunkDeviceFailureDoesNotInventAnOffendingPair) {
    contact::SelfContactTransactionReport report;
    EXPECT_FALSE(Encode(report).HasMember("filter_failure_scope"));
    EXPECT_FALSE(Encode(report).HasMember("filter_chunk_begin"));
    EXPECT_FALSE(Encode(report).HasMember("filter_chunk_pairs"));
    report.status=contact::SelfContactTransactionStatus::FacetFilterFailure;
    report.filter_status=contact::self_contact_filters::Status::DeviceFailure;
    report.filter_scope=contact::SelfContactFacetFilterFailureScope::CandidateChunkBeforeFold;
    report.filter_chunk_begin=(std::size_t{1}<<54)+7;
    report.filter_chunk_pairs=4096;
    report.message="Actual CUDA transfer failure";
    const auto document=Encode(report);
    EXPECT_STREQ(document["filter_failure_scope"].GetString(),"candidate_chunk_before_serial_fold");
    EXPECT_EQ(document["filter_chunk_begin"].GetUint64(),report.filter_chunk_begin);
    EXPECT_EQ(document["filter_chunk_pairs"].GetUint64(),4096u);
    EXPECT_EQ(document["filter_status_code"].GetUint(),static_cast<unsigned>(report.filter_status));
    EXPECT_STREQ(document["message"].GetString(),report.message);
    EXPECT_FALSE(document.HasMember("pair_ordinal"));
    EXPECT_FALSE(document.HasMember("native_crossing_failure"));
    EXPECT_FALSE(document.HasMember("required_force_events"));
}

TEST(VehicleRunSelfContactError, NativeWorkFailureRetainsPrefixAndBatchScopesExactly) {
    contact::SelfContactTransactionReport report;
    report.status = contact::SelfContactTransactionStatus::CrossingFailure;
    report.crossing_status = contact::RepresentedIntervalStatus::ResourceLimit;
    report.pair = 9001;
    auto& native = report.crossing_diagnostics;
    native.available = true;
    native.batch_pair_offset = 1024;
    native.prior_batch_work = (std::size_t{1} << 54) + 17;
    native.input_pair = 1;
    native.input_paths = 3;
    native.input_pairs = native.unique_pairs = 2;
    native.unresolved = 1;
    native.admitted_work = 3;
    native.total_work_limit = 5;
    native.rejected_pair_work = 3;
    const auto json = Encode(report);
    EXPECT_EQ(json["pair_ordinal"].GetUint64(), 9001u);
    const auto& value = json["native_crossing_failure"];
    EXPECT_STREQ(value["schema"].GetString(), "robo_dyna.native_crossing_failure.v1");
    EXPECT_EQ(value["batch_pair_offset"].GetUint64(), 1024u);
    EXPECT_EQ(value["prior_batch_work"].GetUint64(), native.prior_batch_work);
    EXPECT_FALSE(value.HasMember("input_path"));
    EXPECT_EQ(value["input_pair"].GetUint64(), 1u);
    EXPECT_EQ(value["input_paths"].GetUint64(), 3u);
    EXPECT_EQ(value["input_pairs"].GetUint64(), 2u);
    EXPECT_EQ(value["unique_pairs"].GetUint64(), 2u);
    EXPECT_EQ(value["unresolved"].GetUint64(), 1u);
    EXPECT_EQ(value["admitted_work"].GetUint64(), 3u);
    EXPECT_EQ(value["total_work_limit"].GetUint64(), 5u);
    EXPECT_EQ(value["rejected_pair_work"].GetUint64(), 3u);
    EXPECT_FALSE(value.HasMember("complete_chunk_work"));
    EXPECT_FALSE(json.HasMember("required_force_events"));
}

TEST(VehicleRunSelfContactError, UnavailableNativeDiagnosticsAreNotInvented) {
    contact::SelfContactTransactionReport report;
    EXPECT_FALSE(Encode(report).HasMember("native_crossing_failure"));
    report.crossing_diagnostics.available = true;
    const auto json = Encode(report);
    const auto& value = json["native_crossing_failure"];
    EXPECT_FALSE(value.HasMember("input_pair"));
    EXPECT_FALSE(value.HasMember("total_work_limit"));
    EXPECT_FALSE(value.HasMember("rejected_pair_work"));
}

TEST(VehicleRunSelfContactError, SourceFacetIdentityAndBoundsSurviveJsonRoundTrip) {
    contact::SelfContactTransactionReport report;
    report.status = contact::SelfContactTransactionStatus::UnresolvedCandidate;
    report.crossing_reason = contact::RepresentedIntervalReason::WorkExhausted;
    report.pair = 2186;
    auto& first = report.offending_motion[0];
    first.facet = {19, 2142381, 0, 1};
    first.active_parent = 71;
    first.motion = contact::SelfContactFacetMotion::PartialOrMixedRigid;
    first.rigid_group_count = 1;
    first.rigid_groups[0].binding_group = 7;
    first.rigid_groups[0].source_group_id = (std::uint64_t{1} << 60) + 31;
    first.rigid_groups[0].source_node_set_id = 91;
    report.offending_motion[1].facet = {19, 2230072, 0, 1};
    report.offending_half_thickness_m[0] = .001;
    report.offending_swept_bounds[0] = {{-1, -2, -3}, {4, 5, 6}};
    report.offending_quadratic_lower[0][2] = {-.25, -.5, -.75};
    report.offending_quadratic_upper[0][2] = {.25, .5, .75};
    const auto json = Encode(report);
    EXPECT_STREQ(json["schema"].GetString(), "robo_dyna.self_contact_stage_error.v2");
    EXPECT_TRUE(json.HasMember("pair_ordinal_scope"));
    EXPECT_TRUE(json.HasMember("crossing_reason_detail"));
    const auto& facet = json["offending_facets"][0];
    EXPECT_TRUE(facet["available"].GetBool());
    EXPECT_EQ(facet["parent_eid"].GetUint64(), 2142381u);
    EXPECT_EQ(facet["local_facet"].GetUint64(), 1u);
    EXPECT_EQ(facet["active_parent_ordinal"].GetUint64(), 71u);
    EXPECT_EQ(facet["rigid_groups"][0]["source_group_id"].GetUint64(),
              (std::uint64_t{1} << 60) + 31);
    EXPECT_EQ(facet["reported_half_thickness_m"]["binary64_bits"][0].GetUint64(),
              output::Bits(.001));
    EXPECT_EQ(facet["reported_swept_lower_m"]["values"][2].GetDouble(), -3);
    EXPECT_EQ(facet["reported_quadratic_coefficients"][2]["upper"]["binary64_bits"][1].GetUint64(),
              output::Bits(.5));
}

TEST(VehicleRunSelfContactError, NonfiniteDiagnosticsPreserveBitsWithoutInvalidJson) {
    contact::SelfContactTransactionReport report;
    report.offending_motion[0].facet = {1, 2, 0, 0};
    const double nan = std::numeric_limits<double>::quiet_NaN();
    report.offending_swept_bounds[0].lower = {-0.0, nan,
        std::numeric_limits<double>::infinity()};
    const auto json = Encode(report);
    const auto& coordinates = json["offending_facets"][0]["reported_swept_lower_m"];
    EXPECT_FALSE(coordinates["all_finite"].GetBool());
    EXPECT_EQ(coordinates["binary64_bits"][0].GetUint64(), output::Bits(-0.0));
    EXPECT_EQ(coordinates["binary64_bits"][1].GetUint64(), output::Bits(nan));
    EXPECT_EQ(coordinates["binary64_bits"][2].GetUint64(),
              output::Bits(std::numeric_limits<double>::infinity()));
    EXPECT_TRUE(coordinates["values"][1].IsNull());
    EXPECT_TRUE(coordinates["values"][2].IsNull());
}

TEST(VehicleRunSelfContactError, MissingMotionAndInvalidGroupCountRemainExplicitAndBounded) {
    contact::SelfContactTransactionReport report;
    const auto empty = Encode(report);
    EXPECT_FALSE(empty["offending_facets"][0]["available"].GetBool());
    EXPECT_FALSE(empty["offending_facets"][0].HasMember("parent_eid"));
    EXPECT_FALSE(empty.HasMember("pair_ordinal_scope"));
    report.offending_motion[0].facet = {1, 2, 0, 0};
    report.offending_motion[0].rigid_group_count = SIZE_MAX;
    const auto corrupt = Encode(report);
    const auto& facet = corrupt["offending_facets"][0];
    EXPECT_EQ(facet["rigid_group_count"].GetUint64(), SIZE_MAX);
    EXPECT_FALSE(facet["rigid_groups_complete"].GetBool());
    EXPECT_EQ(facet["rigid_groups"].Size(), 4u);
}

TEST(VehicleRunSelfContactError, NativeDeviceWindowDoesNotFabricatePhysicalPair) {
    contact::SelfContactTransactionReport report;
    EXPECT_FALSE(Encode(report).HasMember("crossing_device_status_code"));
    report.crossing_device_status=contact::RepresentedIntervalDeviceStatus::Ok;
    EXPECT_FALSE(Encode(report).HasMember("crossing_device_status_code"));
    report.status=contact::SelfContactTransactionStatus::CrossingFailure;
    report.crossing_status=contact::RepresentedIntervalStatus::ResourceLimit;
    report.crossing_device_status=contact::RepresentedIntervalDeviceStatus::DeviceFailure;
    report.crossing_fault_cohort_begin=(std::size_t{1}<<54)+11;
    report.crossing_fault_cohort_count=4096;
    const auto document=Encode(report);
    EXPECT_EQ(document["crossing_fault_cohort_begin"].GetUint64(),report.crossing_fault_cohort_begin);
    EXPECT_EQ(document["crossing_fault_cohort_count"].GetUint64(),4096u);
    EXPECT_FALSE(document.HasMember("crossing_fault_pair_ordinal"));
    EXPECT_FALSE(document.HasMember("pair_ordinal"));
    report.crossing_fault_pair_ordinal=report.crossing_fault_cohort_begin+2;
    const auto known=Encode(report);
    EXPECT_EQ(known["crossing_fault_pair_ordinal"].GetUint64(),report.crossing_fault_pair_ordinal);
    EXPECT_FALSE(known.HasMember("pair_ordinal"));
}

}  // namespace crash::cases::vehicle_run::test
