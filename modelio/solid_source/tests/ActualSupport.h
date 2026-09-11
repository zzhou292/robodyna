#pragma once
#include "TestSupport.h"
#include "modelio/vehicle_source/tests/TestSupport.h"
#include <iostream>

namespace crash::modelio::solid_source::test {
inline const std::string& MemberBytes() {
    static const auto bytes = [] {
        const auto* path = std::getenv("ROBO_STATIC_MEMBER");
        output::Require(path && *path, "Missing original solid source member");
        return output::ReadBounded(path, 42846753);
    }();
    return bytes;
}
inline const VehicleSolidSource& Original() {
    static const auto result = [] {
        const auto& canonical = vehicle::test::Canonical();
        const auto policy = Policy::OriginalAdhesive18RubberHephS6zV1;
        const auto forecast = VehicleSolidSource::Preflight(canonical, policy);
        std::cout << "Original solid complete startup forecast: " << forecast.total_bytes << " bytes\n";
        return VehicleSolidSource::Prepare(canonical, MemberBytes(), policy);
    }();
    return result;
}
inline const VehicleSolidSource& Extended() {
    static const auto result = [] {
        const auto& canonical = vehicle::test::Canonical();
        const auto policy = Policy::OriginalAdhesive18ExtendedRubberHephS6zV2;
        const auto forecast = VehicleSolidSource::Preflight(canonical, policy);
        std::cout << "Extended rubber complete startup forecast: " << forecast.total_bytes << " bytes\n";
        return VehicleSolidSource::Prepare(canonical, MemberBytes(), policy);
    }();
    return result;
}
template<class Input> void InputBits(const Input& actual, const Input& expected, unsigned nodes) {
    ASSERT_EQ(actual.source_element_id, expected.source_element_id);
    EXPECT_EQ(actual.source_part_id, expected.source_part_id);
    EXPECT_EQ(actual.source_material_id, expected.source_material_id);
    EXPECT_EQ(actual.source_section_id, expected.source_section_id);
    EXPECT_EQ(output::Bits(actual.density_kg_m3), output::Bits(expected.density_kg_m3));
    for (unsigned n = 0; n < nodes; ++n) {
        EXPECT_EQ(actual.source_node_id[n], expected.source_node_id[n]);
        EXPECT_EQ(output::Bits(actual.position_m[n].x), output::Bits(expected.position_m[n].x));
        EXPECT_EQ(output::Bits(actual.position_m[n].y), output::Bits(expected.position_m[n].y));
        EXPECT_EQ(output::Bits(actual.position_m[n].z), output::Bits(expected.position_m[n].z));
    }
}
template<class Values> void EqualBits(const Values& actual, const Values& expected) {
    ASSERT_EQ(actual.size(), expected.size());
    for (std::size_t i = 0; i < actual.size(); ++i) {
        SCOPED_TRACE(i);
        EXPECT_EQ(output::Bits(actual[i]), output::Bits(expected[i]));
    }
}
} // namespace crash::modelio::solid_source::test
