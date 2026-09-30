#include "ActualSupport.h"
#include "ExtendedReceipt.h"
#if defined(ROBO_DYNA_EXTENDED_SOLID24)
#include "lib_utest/qualification/solid24_reference/JacobianNative.h"
#else
#include "lib_utest/qualification/solid6z_reference/NativeOracle.h"
#endif

namespace crash::modelio::solid_source::test {
#if defined(ROBO_DYNA_EXTENDED_SOLID24)
TEST(VehicleSolidExtendedNative, All682AddedBricksMatchCompleteNativeReferenceAndMass) {
    std::size_t count = 0;
    const auto& data = Extended().data();
    for (const auto& row : data.rows) {
        if (!Added(row.part_id) || row.family != Family::Solid24) continue;
        SCOPED_TRACE(row.element_id);
        const auto& reference = data.solid24.at(row.reference_index);
        // SI packet independently exercises the native geometry/mass and total
        // Jacobian leaves; production retains its explicit millimetre floor.
        const auto native = solid24_test::GlobalNativeRaw(reference.input());
        ASSERT_EQ(native.status, 0);
        EXPECT_TRUE(solid24_test::Agree(solid24_test::Values(reference), native.legacy()));
        EXPECT_TRUE(solid24_test::JacobianAgree(solid24_test::JacobianValues(reference), native.jacobian()));
        for (unsigned slot = 0; slot < 8; ++slot)
            EXPECT_EQ(reference.source_slot(slot), static_cast<unsigned>(native.permutation[slot]));
        ++count;
    }
    EXPECT_EQ(count, 682u);
    RecordProperty("added_native_bricks", count);
}
#else
TEST(VehicleSolidExtendedNative, All155AddedWedgesMatchCompleteNativeReferenceAndMass) {
    std::size_t count = 0;
    const auto& data = Extended().data();
    for (const auto& row : data.rows) {
        if (!Added(row.part_id) || row.family != Family::Solid6z) continue;
        SCOPED_TRACE(row.element_id);
        const auto& reference = data.solid6z.at(row.reference_index);
        const auto native = solid6z_test::Native(reference.input());
        ASSERT_EQ(native.status, 0);
        EXPECT_TRUE(solid6z_test::Agree(solid6z_test::Values(reference), native.values));
        for (unsigned slot = 0; slot < 6; ++slot)
            EXPECT_EQ(reference.source_slot(slot), static_cast<unsigned>(native.permutation[slot]));
        ++count;
    }
    EXPECT_EQ(count, 155u);
    RecordProperty("added_native_wedges", count);
}
#endif
} // namespace crash::modelio::solid_source::test
