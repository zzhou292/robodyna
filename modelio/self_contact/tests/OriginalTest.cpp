#include "../OriginalSelection.h"
#include "modelio/vehicle_source/tests/TestSupport.h"
#include <gtest/gtest.h>
#include <iostream>
#include <map>

namespace crash::modelio::self_contact::test {
namespace {

const std::string& Member(const char* variable, std::size_t bytes) {
    const auto* path = std::getenv(variable);
    output::Require(path && *path,
        "Missing explicit original self-contact source member");
    static std::map<std::string, std::string> values;
    auto found = values.find(variable);
    if (found == values.end()) {
        auto value = output::ReadBounded(path, bytes);
        output::Require(value.size() == bytes,
            "Original self-contact source member extent differs");
        found = values.emplace(variable, std::move(value)).first;
    }
    return found->second;
}

const std::string& Auxiliary() {
    return Member("ROBO_SELF_CONTACT_AUX_MEMBER", 44991);
}

const std::string& Combine() {
    return Member("ROBO_SELF_CONTACT_COMBINE_MEMBER", 10577);
}

const OriginalSelection& Actual() {
    static const auto value = OriginalSelection::Prepare(
        vehicle::test::Canonical(), Auxiliary(), Combine());
    return value;
}

}  // namespace

TEST(OriginalSelfContactSelectionActual,
    ExactOriginalCardSetAndCompleteTypedCensusAreRetained) {
    const auto& selection = Actual();
    const auto& data = selection.data();
    EXPECT_EQ(data.profile,
        OriginalSelectionProfile::AutomaticSingleSurfacePartSetV1);
    EXPECT_EQ(data.auxiliary_sha256,
        "b93d5370a899f6f70299ea61cd55142c1f8b765b8ab7f9ac979d078486028929");
    EXPECT_EQ(data.combine_sha256,
        "3e0137cd8c569a71a4281cc307549dc2f20772bc67eac0658ae73682dbe242a2");
    EXPECT_EQ(data.source_fields.slave_set_id, 1000002u);
    EXPECT_EQ(data.source_fields.master_set_id, 0u);
    EXPECT_EQ(data.source_fields.slave_set_type, 2u);
    ASSERT_TRUE(data.source_fields.static_friction);
    ASSERT_TRUE(data.source_fields.dynamic_friction);
    ASSERT_TRUE(data.source_fields.decay_coefficient);
    ASSERT_TRUE(data.source_fields.soft);
    ASSERT_TRUE(data.source_fields.ignore_initial_penetration);
    EXPECT_EQ(*data.source_fields.static_friction, .2);
    EXPECT_EQ(*data.source_fields.dynamic_friction, .1);
    EXPECT_EQ(*data.source_fields.decay_coefficient, .001);
    EXPECT_EQ(*data.source_fields.soft, 1);
    EXPECT_EQ(*data.source_fields.ignore_initial_penetration, 1);
    EXPECT_EQ(data.sources.size(), 3u);
    EXPECT_EQ(data.selected_part_ids.size(), 861u);
    EXPECT_EQ(data.parts.size(), data.selected_part_ids.size());
    // Independently reproduced from the pre-existing contact-evidence set and
    // authenticated canonical arrays in self-contact-source-independent-census-1.
    EXPECT_EQ(data.counts.selected_parts, 861u);
    EXPECT_EQ(data.counts.shell_parts, 850u);
    EXPECT_EQ(data.counts.retained_shell_parts, 842u);
    EXPECT_EQ(data.counts.excluded_shell_parts, 8u);
    EXPECT_EQ(data.counts.non_shell_parts, 11u);
    EXPECT_EQ(data.counts.shells, 345904u);
    EXPECT_EQ(data.counts.retained_shells, 337092u);
    EXPECT_EQ(data.counts.excluded_shells, 8812u);
    EXPECT_EQ(data.counts.solids, 2952u);
    EXPECT_EQ(data.counts.beams, 0u);
    EXPECT_GT(data.startup_budget_bytes, data.owned_payload_bytes);
    std::cout << "Original self-contact selection parts="
              << data.counts.selected_parts
              << " shell_parts=" << data.counts.shell_parts
              << " retained_shell_parts=" << data.counts.retained_shell_parts
              << " excluded_shell_parts=" << data.counts.excluded_shell_parts
              << " non_shell_parts=" << data.counts.non_shell_parts
              << " shells=" << data.counts.shells
              << " retained_shells=" << data.counts.retained_shells
              << " excluded_shells=" << data.counts.excluded_shells
              << " solids=" << data.counts.solids
              << " beams=" << data.counts.beams
              << " startup_budget_bytes=" << data.startup_budget_bytes
              << " owned_payload_bytes=" << data.owned_payload_bytes << '\n';
    RecordProperty("selected_parts", data.counts.selected_parts);
    RecordProperty("shell_parts", data.counts.shell_parts);
    RecordProperty("retained_shell_parts", data.counts.retained_shell_parts);
    RecordProperty("excluded_shell_parts", data.counts.excluded_shell_parts);
    RecordProperty("non_shell_parts", data.counts.non_shell_parts);
    RecordProperty("shells", data.counts.shells);
    RecordProperty("retained_shells", data.counts.retained_shells);
    RecordProperty("excluded_shells", data.counts.excluded_shells);
    RecordProperty("solids", data.counts.solids);
    RecordProperty("beams", data.counts.beams);
    RecordProperty("startup_budget_bytes",
        std::to_string(data.startup_budget_bytes));
    RecordProperty("owned_payload_bytes",
        std::to_string(data.owned_payload_bytes));
}

TEST(OriginalSelfContactSelectionActual,
    ExactHostCapAuthenticationRetentionAndRetry) {
    const auto& canonical = vehicle::test::Canonical();
    const auto forecast = OriginalSelection::Forecast(
        canonical, Auxiliary().size(), Combine().size());
    auto limits = Limits{};
    limits.host_bytes = forecast - 1;
    EXPECT_THROW(OriginalSelection::Forecast(canonical,
        Auxiliary().size(), Combine().size(), limits), std::exception);
    ++limits.host_bytes;
    EXPECT_EQ(OriginalSelection::Forecast(canonical,
        Auxiliary().size(), Combine().size(), limits), forecast);
    EXPECT_THROW(OriginalSelection::Prepare(canonical,
        Auxiliary(), Combine() + "x"), std::exception);
    const auto copy = [] {
        auto value = OriginalSelection::Prepare(
            vehicle::test::Canonical(), Auxiliary(), Combine());
        return OriginalSelection(value);
    }();
    EXPECT_EQ(copy.data().selected_part_ids,
        Actual().data().selected_part_ids);
    EXPECT_EQ(&copy.canonical().data(),
        &vehicle::test::Canonical().data());
}

}  // namespace crash::modelio::self_contact::test
