#include "../MaterialSlots.h"
#include "native/ContactSlotsReference.h"
#include "lib_src/materials/law42/Prepare.h"
#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <limits>

namespace crash::cases::vehicle_self_contact::native::nodal_seed::test {
namespace {
void NearRoundoff(double actual, double native) {
    const auto scale = std::max(std::abs(actual), std::abs(native));
    EXPECT_LE(std::abs(actual - native), 64 * std::numeric_limits<double>::epsilon() * scale);
}
}
TEST(ContactSeedMaterialSlots, NoPronyLaw42UsesReaderGsForPm32AcrossWorkingUnits) {
    const n::UnitScale units[]{{1, 1, 1}, {.001, 1000, 1}, {.01, 1, 1}};
    for (const auto unit : units) {
        const auto factors = detail::Factors(unit);
        SCOPED_TRACE(unit.length_m);
        for (const double mu : {1e-12, .123, 12345.67, 5e8, 1e100}) {
            SCOPED_TRACE(mu);
            tl::material::law42::Parameters material;
            ASSERT_EQ(tl::material::law42::Prepare(mu, .463, 1000., 1e20, material),
                tl::material::law42::Status::Ok);
            const double parameters[]{mu / factors.pressure, .463};
            double observed[4]{};
            law42_contact_slots_native(parameters, observed);
            NearRoundoff(detail::Law42ContactPm32Pa(material) / factors.pressure, observed[1]);
            NearRoundoff(material.bulk_pa / factors.pressure, observed[2]);
            EXPECT_GT(observed[0], observed[1]);
            EXPECT_GT(observed[2], 6 * observed[1]);
            EXPECT_EQ(observed[3], 2 * observed[2]);
            // A physical-bulk substitute is a large, observable wrong-slot
            // control, not merely a unit-rounding discrepancy.
            EXPECT_GT(std::abs(material.bulk_pa / factors.pressure - observed[1]), std::abs(observed[1]));
        }
    }
    RecordProperty("native_profile", "MAT007-to-LAW42 alpha2/no-Prony GammaInf1; slots observed after UPDMAT");
    RecordProperty("production_scope", "PM32 source binding only; PM107 correction is a later product");
}
TEST(ContactSeedMaterialSlots, PhysicalBulkCannotStandInForPm32AndInvalidMuRejects) {
    tl::material::law42::Parameters material;
    ASSERT_EQ(tl::material::law42::Prepare(4., .463, 1000., 1e20, material), tl::material::law42::Status::Ok);
    const auto original = detail::Law42ContactPm32Pa(material);
    // Value-level non-consumption check only; this mutated copy is not
    // represented as an authenticated physical model or source declaration.
    material.bulk_pa *= 3;
    EXPECT_EQ(detail::Law42ContactPm32Pa(material), original);
    material.mu_pa = std::numeric_limits<double>::infinity();
    EXPECT_THROW(detail::Law42ContactPm32Pa(material), std::exception);
}
} // namespace crash::cases::vehicle_self_contact::native::nodal_seed::test
