#include <sstream>
#include <stdexcept>
#include <string>
#include <typeindex>

#include <gtest/gtest.h>

#include "chrono/serialization/ChArchiveBinary.h"
#include "chrono/serialization/ChArchiveJSON.h"

namespace {
struct VersionMarker {};
struct NestedMarker {};

template <class Base>
class ObservedOutput : public Base {
  public:
    explicit ObservedOutput(std::ostream& stream) : Base(stream) {}
    int calls = 0;
    bool throw_next = false;
    bool nested_next = false;

  protected:
    void out_version(int version, std::type_index type) override {
        ++calls;
        if (throw_next) {
            throw_next = false;
            throw std::runtime_error("output hook failure");
        }
        if (nested_next) {
            nested_next = false;
            // A normal nested request must not inherit the outer explicit name.
            this->template VersionWrite<NestedMarker>();
        }
        Base::out_version(version, type);
    }
};

template <class Base>
class ObservedInput : public Base {
  public:
    explicit ObservedInput(std::istream& stream) : Base(stream) {}
    int calls = 0;
    bool throw_next = false;

  protected:
    int in_version(std::type_index type) override {
        ++calls;
        if (throw_next) {
            throw_next = false;
            throw std::runtime_error("input hook failure");
        }
        return Base::in_version(type);
    }
};

TEST(NamedArchiveVersion, RetainsClusteringAndExistingVirtualHooks) {
    for (const bool cluster : {true, false}) {
        std::stringstream stream;
        {
            ObservedOutput<chrono::ChArchiveOutJSON> output(stream);
            output.SetClusterClassVersions(cluster);
            output.VersionWrite<VersionMarker>("LegacyMarker");
            output.VersionWrite<VersionMarker>("LegacyMarker");
            EXPECT_EQ(output.calls, cluster ? 1 : 2);
        }
        ObservedInput<chrono::ChArchiveInJSON> input(stream);
        input.SetClusterClassVersions(cluster);
        EXPECT_EQ(input.VersionRead<VersionMarker>("LegacyMarker"), 0);
        // Preserve the inherited repeated clustered-read sentinel.
        EXPECT_EQ(input.VersionRead<VersionMarker>("LegacyMarker"), cluster ? 99999 : 0);
        EXPECT_EQ(input.calls, cluster ? 1 : 2);
    }
}

TEST(NamedArchiveVersion, DisabledVersionsDoNotInvokeHooksOrReadTokens) {
    std::stringstream stream;
    {
        ObservedOutput<chrono::ChArchiveOutJSON> output(stream);
        output.SetUseVersions(false);
        output.VersionWrite<VersionMarker>("LegacyMarker");
        EXPECT_EQ(output.calls, 0);
    }
    EXPECT_EQ(stream.str().find("LegacyMarker"), std::string::npos);
    ObservedInput<chrono::ChArchiveInJSON> input(stream);
    input.SetUseVersions(false);
    EXPECT_EQ(input.VersionRead<VersionMarker>("LegacyMarker"), 99999);
    EXPECT_EQ(input.calls, 0);
}

TEST(NamedArchiveVersion, OutputScopeRestoresOnExceptionAndFiltersNestedType) {
    std::stringstream stream;
    {
        ObservedOutput<chrono::ChArchiveOutJSON> output(stream);
        output.SetClusterClassVersions(false);
        output.throw_next = true;
        EXPECT_THROW(output.VersionWrite<VersionMarker>("FailedLegacyName"), std::runtime_error);
        output.VersionWrite<VersionMarker>();
        output.nested_next = true;
        output.VersionWrite<VersionMarker>("OuterLegacyName");
        EXPECT_EQ(output.calls, 4);
    }
    const auto bytes = stream.str();
    EXPECT_EQ(bytes.find("FailedLegacyName"), std::string::npos);
    EXPECT_NE(bytes.find("_version_" + std::string(typeid(VersionMarker).name())), std::string::npos);
    EXPECT_NE(bytes.find("_version_" + std::string(typeid(NestedMarker).name())), std::string::npos);
    EXPECT_NE(bytes.find("_version_OuterLegacyName"), std::string::npos);
}

TEST(NamedArchiveVersion, InputScopeRestoresOnException) {
    std::stringstream stream;
    {
        chrono::ChArchiveOutJSON output(stream);
        output.SetClusterClassVersions(false);
        output.VersionWrite<VersionMarker>();
        output.VersionWrite<VersionMarker>("LegacyMarker");
    }
    ObservedInput<chrono::ChArchiveInJSON> input(stream);
    input.SetClusterClassVersions(false);
    input.throw_next = true;
    EXPECT_THROW(input.VersionRead<VersionMarker>("FailedLegacyName"), std::runtime_error);
    EXPECT_EQ(input.VersionRead<VersionMarker>(), 0);
    EXPECT_EQ(input.VersionRead<VersionMarker>("LegacyMarker"), 0);
    EXPECT_EQ(input.calls, 3);
}

TEST(NamedArchiveVersion, BinaryUsesTheSameVirtualPolicyAndBytes) {
    std::stringstream named(std::ios::in | std::ios::out | std::ios::binary);
    std::stringstream ordinary(std::ios::in | std::ios::out | std::ios::binary);
    {
        ObservedOutput<chrono::ChArchiveOutBinary> output(named);
        output.VersionWrite<VersionMarker>("LegacyMarker");
        EXPECT_EQ(output.calls, 1);
        chrono::ChArchiveOutBinary unchanged(ordinary);
        unchanged.VersionWrite<VersionMarker>();
    }
    EXPECT_EQ(named.str(), ordinary.str());
    ObservedInput<chrono::ChArchiveInBinary> input(named);
    EXPECT_EQ(input.VersionRead<VersionMarker>("LegacyMarker"), 0);
    EXPECT_EQ(input.calls, 1);
}

TEST(NamedArchiveVersion, RejectsEmptyNamesAndRetainsExistingKeySanitization) {
    std::stringstream stream;
    {
        ObservedOutput<chrono::ChArchiveOutJSON> output(stream);
        EXPECT_THROW(output.VersionWrite<VersionMarker>(nullptr), std::invalid_argument);
        EXPECT_THROW(output.VersionWrite<VersionMarker>(""), std::invalid_argument);
        EXPECT_EQ(output.calls, 0);
        output.VersionWrite<VersionMarker>("old::type<double value>");
    }
    EXPECT_NE(stream.str().find("_version_old__type_double_value_"), std::string::npos);
    ObservedInput<chrono::ChArchiveInJSON> input(stream);
    EXPECT_THROW(input.VersionRead<VersionMarker>(nullptr), std::invalid_argument);
    EXPECT_THROW(input.VersionRead<VersionMarker>(""), std::invalid_argument);
    EXPECT_EQ(input.calls, 0);
    EXPECT_EQ(input.VersionRead<VersionMarker>("old::type<double value>"), 0);
}
}  // namespace
