#include "case/vehicle_run/ContactComposition.h"
#include "case/vehicle_run/source/OriginalYaris.h"

#include <gtest/gtest.h>
#include <stdexcept>
#include <string>

namespace crash::cases::vehicle_run {
namespace {

TEST(VehicleContactComposition, ProfileNamesAreExplicitAndUnknownValuesReject) {
    EXPECT_STREQ(ContactProfileName(ContactProfile::WallOnly), "wall-only");
    EXPECT_STREQ(ContactProfileName(ContactProfile::WallSelfContactV1),
                 "wall-self-contact-v1");
    EXPECT_THROW(ContactProfileName(static_cast<ContactProfile>(-1)),
                 std::invalid_argument);
    EXPECT_THROW(ContactComposition::Prepare(static_cast<ContactProfile>(-1)),
                 std::invalid_argument);
}

TEST(VehicleContactComposition, SelfProfileCannotProceedWithoutItsSource) {
    EXPECT_THROW(ContactComposition::Prepare(ContactProfile::WallSelfContactV1),
                 std::exception);
    const auto wall = ContactComposition::Prepare(ContactProfile::WallOnly);
    EXPECT_EQ(wall.profile(), ContactProfile::WallOnly);
    EXPECT_EQ(wall.self_contact_setup(), nullptr);
    EXPECT_EQ(wall.runtime_config().source_id, 0u);
    EXPECT_EQ(wall.runtime_config().event_capacity, 0u);
}

TEST(VehicleContactComposition, MissingContactMemberRejectsBeforeModelConstruction) {
    // None of these paths exists. The explicit contact source requirement must
    // fail before reading a deck or allocating the CUDA startup search owner.
    const OriginalPaths missing;
    try {
        PrepareOriginalYaris(missing, vehicle_wall::LoadedWallSettings(),
            PhysicalProfile::VehicleSupportsV5, ContactProfile::WallSelfContactV1);
        FAIL() << "Missing contact member was accepted";
    } catch (const std::exception& error) {
        EXPECT_NE(std::string(error.what()).find(
            "explicit original contact member path"), std::string::npos);
    }
}

}  // namespace
}  // namespace crash::cases::vehicle_run
