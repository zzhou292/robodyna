#include "../MaterialSlots.h"
#include "../../nodal_seed/tests/native/ContactSlotsReference.h"
#include "lib_src/materials/law42/Prepare.h"
#include "lib_utest/qualification/law90_preparation/TestSupport.h"
#include "lib_utest/qualification/law90_preparation/native/NativeOracle.h"
#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <limits>

namespace crash::cases::vehicle_self_contact::native::nodal_correction::test {
namespace d = detail;
namespace {
void Same(double actual, double expected) {
    EXPECT_TRUE(std::isfinite(actual) && std::isfinite(expected));
    EXPECT_LE(std::abs(actual-expected), 64*std::numeric_limits<double>::epsilon()*std::max(std::abs(actual),std::abs(expected)));
}
}
TEST(CorrectedMaterialSlots, Law42ReaderUpdateAndHighRefreshMatchNativeInThreeUnitContexts) {
    for (const n::UnitScale units : {n::UnitScale{1,1,1}, n::UnitScale{.001,1000,1}, n::UnitScale{.01,1,1}}) {
        modelio::solid_source::Part part;
        part.material_law = modelio::solid_source::MaterialLaw::Law42;
        ASSERT_EQ(tl::material::law42::Prepare(12345.67,.463,1000.,1e20,part.law42), tl::material::law42::Status::Ok);
        const auto pressure = seed::detail::Factors(units).pressure;
        const double input[]{part.law42.mu_pa/pressure,.463};
        double expected[4]{};
        law42_contact_slots_native(input,expected);
        const auto actual = d::Slots(part,units);
        Same(actual.bulk,expected[1]);
        Same(actual.retained_bulk,expected[2]);
        Same(actual.controlled_bulk,expected[3]);
        EXPECT_NE(actual.bulk,actual.retained_bulk);
    }
}
TEST(CorrectedMaterialSlots, Law90RetainsReaderBulkAfterTheFullNativeCurveUpdater) {
    modelio::solid_source::Part part;
    part.material_law = modelio::solid_source::MaterialLaw::Law90;
    const auto input = law90_test::OriginalBlankHuInput();
    const auto curve = law90_test::OriginalBlankHuCurve();
    ASSERT_EQ(tl::material::law90::PrepareSI(input,curve,part.law90),tl::material::law90::Status::Ok);
    const auto inputs = law90_test::InputValues(input);
    const auto flags = law90_test::InputFlags(input);
    const int count = int(curve.count);
    double native[33]{};
    law90_native_prepare(inputs.data(),flags.data(),curve.compression_strain,curve.stress_pa,&count,native);
    ASSERT_NE(native[29],native[6]);
    for (const n::UnitScale units : {n::UnitScale{1,1,1},n::UnitScale{.001,1000,1}}) {
        const auto pressure = seed::detail::Factors(units).pressure;
        const double slots[]{native[29]/pressure,native[6]/pressure};
        double high = 0;
        law90_contact_high_native(slots,&high);
        const auto actual = d::Slots(part,units);
        Same(actual.bulk,slots[0]);
        Same(actual.retained_bulk,slots[1]);
        Same(actual.controlled_bulk,high);
        // Refreshing PM100 from the new PM32 would change the actual law90
        // correction; this is a real phase control, not a synthetic equality.
        EXPECT_NE(actual.controlled_bulk,2*actual.bulk);
    }
}
TEST(CorrectedMaterialSlots, ControlledBulkRefreshPreservesNativeLastEqualSignedZero) {
    for (double first : {0., -0.}) {
        for (double second : {0., -0.}) {
            const double inputs[]{first, second};
            double expected = 19.;
            law90_contact_high_native(inputs, &expected);
            const auto actual = d::RefreshControlledBulk(first, second);
            EXPECT_EQ(output::Bits(actual), output::Bits(expected));
            EXPECT_EQ(output::Bits(actual), output::Bits(second));
        }
    }
}
} // namespace crash::cases::vehicle_self_contact::native::nodal_correction::test
