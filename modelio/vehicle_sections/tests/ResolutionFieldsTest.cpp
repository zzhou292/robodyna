#include "../Internal.h"
#include "modelio/source_assembly/NativeMaterialInput.h"
#include <gtest/gtest.h>

namespace crash::modelio::vehicle::test {
namespace {
output::Document Fixture() {
    const auto* path = std::getenv("ROBO_VEHICLE_SECTION_FIELDS");
    output::Require(path && *path, "Missing explicit tiny field fixture");
    const auto bytes = output::ReadBounded(path, 128 * 1024);
    output::Document doc;
    doc.Parse<rapidjson::kParseFullPrecisionFlag>(bytes.data(), bytes.size());
    output::Require(!doc.HasParseError(), "Invalid tiny field fixture");
    return doc;
}
assembly::Data ReadFields(const output::Value& row) {
    assembly::Data data;
    data.schema = assembly::SectionInventorySchema;
    const auto& typed = row["constant_failure_declarations"];
    assembly::reader::ReadMaterialPolicy(typed, data);
    assembly::reader::ReadConstantFailureDeclarations(typed, {}, data);
    for (const auto& material : data.materials) detail::CheckTypedCards(material.source, material.cards);
    return data;
}
}

TEST(VehicleSectionFields, TypedFailureKeepsSourceUnitsAndLegacyRejection) {
    auto doc = Fixture();
    for (const char* kind : {"table", "linear"}) {
        SCOPED_TRACE(kind);
        const auto data = ReadFields(doc[kind]);
        ASSERT_EQ(data.materials.size(), 1);
        const auto& material = data.materials[0];
        const bool linear = std::string(kind) == "linear";
        EXPECT_EQ(material.law, assembly::MaterialLaw::LayeredLaw44);
        EXPECT_DOUBLE_EQ(material.young_pa, 2e11);
        EXPECT_DOUBLE_EQ(material.density_kg_m3, 7890);
        EXPECT_DOUBLE_EQ(*material.cards[0].values[6], linear ? 3.5 : 1);
        EXPECT_EQ(material.curve_id, linear ? 0 : 48);
        ASSERT_TRUE(material.supplied_etan_pa);
        EXPECT_DOUBLE_EQ(*material.supplied_etan_pa, linear ? 8e8 : 0);
        const auto native = assembly::detail::NativeMaterial(material);
        EXPECT_DOUBLE_EQ(native.rate.cowper_symonds_c_per_s, 8000);
        EXPECT_DOUBLE_EQ(native.rate.cowper_symonds_p, 8);
        const auto& value = doc[kind]["constant_failure_declarations"]["declarations"]["materials"][0];
        EXPECT_THROW(assembly::reader::ReadLaw44Material(value, data), std::runtime_error);
    }
}

TEST(VehicleSectionFields, LiteralFailureAndRawSourceBitsRejectIndependentAlterations) {
    for (unsigned fault = 0; fault < 9; ++fault) {
        SCOPED_TRACE(fault);
        auto doc = Fixture();
        auto& material = doc["table"]["constant_failure_declarations"]["declarations"]["materials"][0];
        switch (fault) {
        case 0: material["failure_strain"].SetDouble(0); break;
        case 1: material["failure_strain"].SetDouble(-0.0); break;
        case 2: material["failure_strain"].SetDouble(3.5); break;
        case 3: material["cards"][0]["values"][6].SetDouble(3.5); break;
        case 4:
            material["failure_strain"].SetDouble(3.5);
            material["cards"][0]["values"][6].SetDouble(3.5);
            break;
        case 5:
            material["young_pa"].SetDouble(1e11);
            material["cards"][0]["values"][2].SetDouble(1e5);
            break;
        case 6: material["material_law"].SetString("layered_law1", doc.GetAllocator()); break;
        case 7:
            material["supplied_etan_pa"].SetDouble(-0.0);
            material["cards"][0]["values"][5].SetDouble(-0.0);
            break;
        case 8:
            material["supplied_etan_pa"].SetDouble(1e6);
            material["cards"][0]["values"][5].SetDouble(1);
            break;
        }
        EXPECT_THROW(ReadFields(doc["table"]), std::runtime_error);
    }
    auto retry = Fixture();
    EXPECT_DOUBLE_EQ(*ReadFields(retry["table"]).materials[0].cards[0].values[6], 1);
}
} // namespace crash::modelio::vehicle::test
