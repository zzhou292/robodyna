#include "chrono_fmi/fmi2/FmuArchiveValues.h"
#include "chrono/core/ChMatrix.h"
#include "chrono/serialization/ChArchiveJSON.h"

#include <gtest/gtest.h>
#include <map>
#include <sstream>
#include <string>

namespace {
using chrono::ChCausalityType;
using chrono::ChNameValue;
using chrono::ChVariabilityType;
using robodyna::fmi::ArchiveValueBindings;

ChNameValue<int> CaptureStackVersion(ArchiveValueBindings& bindings) {
    int value = 17;
    auto captured = bindings.CaptureConstant(
        ChNameValue<int>("version", value, 0, ChCausalityType::local, ChVariabilityType::constant));
    EXPECT_NE(&captured.value(), &value);
    value = 99;
    EXPECT_EQ(captured.value(), 17);
    return captured;
}

TEST(FmiArchiveValues, ConstantMetadataSurvivesStackOverwriteAndStorageGrowth) {
    ArchiveValueBindings bindings;
    const auto version = CaptureStackVersion(bindings);
    volatile int overwrite[256];
    for (int i = 0; i < 256; ++i) overwrite[i] = i;
    for (int i = 0; i < 1024; ++i) {
        bindings.CaptureConstant(ChNameValue<int>("other", i, 0,
                                                ChCausalityType::local, ChVariabilityType::constant));
    }
    EXPECT_EQ(version.value(), 17);
    EXPECT_EQ(version.GetVariability(), ChVariabilityType::constant);
    EXPECT_EQ(overwrite[255], 255);
}

TEST(FmiArchiveValues, ConstantStringsOwnTheirCharactersAndLiveMembersStayLive) {
    ArchiveValueBindings bindings;
    std::string text = "original metadata";
    auto constant = bindings.CaptureConstant(ChNameValue<std::string>(
        "label", text, 0, ChCausalityType::local, ChVariabilityType::constant));
    text.assign(4096, 'x');
    EXPECT_EQ(constant.value(), "original metadata");
    double position = 2;
    int step = 1;
    auto live_position = bindings.CaptureConstant(ChNameValue<double>("position", position));
    auto live_step = bindings.CaptureConstant(ChNameValue<int>("step", step));
    EXPECT_EQ(&live_position.value(), &position);
    EXPECT_EQ(&live_step.value(), &step);
    position = 7;
    step = 12;
    EXPECT_EQ(live_position.value(), 7);
    EXPECT_EQ(live_step.value(), 12);
}

TEST(FmiArchiveValues, OnlyNonRealContinuousDefaultsBecomeDiscrete) {
    for (auto value : {ChVariabilityType::constant, ChVariabilityType::fixed,
                       ChVariabilityType::tunable, ChVariabilityType::discrete,
                       ChVariabilityType::continuous}) {
        EXPECT_EQ(robodyna::fmi::ScalarArchiveVariability(true, value), value);
        EXPECT_EQ(robodyna::fmi::ScalarArchiveVariability(false, value),
                  value == ChVariabilityType::continuous ? ChVariabilityType::discrete : value);
    }
}

class MetadataArchive : public chrono::ChArchiveOutJSON {
  public:
    explicit MetadataArchive(std::ostream& stream) : ChArchiveOutJSON(stream) {}
    void out(ChNameValue<int> value) override {
        metadata[value.name()] = value.GetVariability();
        ChArchiveOutJSON::out(value);
    }
    void out(ChNameValue<unsigned long> value) override {
        metadata[value.name()] = value.GetVariability();
        ChArchiveOutJSON::out(value);
    }
    std::map<std::string, ChVariabilityType> metadata;
};

TEST(FmiArchiveValues, ActualMatrixArchiveTagsOnlyTransientSchemaMetadataConstant) {
    std::ostringstream stream;
    MetadataArchive archive(stream);
    chrono::ChMatrixDynamic<> matrix(2, 3);
    matrix.setConstant(4.0);
    archive << chrono::make_ChNameValue("matrix", matrix);
    ASSERT_EQ(archive.metadata.at("rows"), ChVariabilityType::constant);
    ASSERT_EQ(archive.metadata.at("columns"), ChVariabilityType::constant);
    unsigned versions = 0;
    for (const auto& [name, variability] : archive.metadata) {
        if (name.rfind("_version_", 0) == 0) {
            ++versions;
            EXPECT_EQ(variability, ChVariabilityType::constant);
        }
    }
    EXPECT_GT(versions, 0u);
}
}  // namespace
