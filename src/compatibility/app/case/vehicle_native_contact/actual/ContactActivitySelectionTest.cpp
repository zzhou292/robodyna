#include "ContactActivitySelection.h"
#include <gtest/gtest.h>
namespace crash::cases::vehicle_native_contact::test {
TEST(NativeContactActivitySelection, ExplicitCapabilityDoesNotChangeTheLegacyDefault) {
    using P = tlfea::contact::radioss_type25::ContactActivityPolicy;
    EXPECT_EQ(ParseContactActivity(nullptr), P::AllActivePrefix);
    EXPECT_EQ(ParseContactActivity("all_active_prefix"), P::AllActivePrefix);
    EXPECT_EQ(ParseContactActivity("shell_removal"), P::ShellRemoval);
    for (const auto* text : {"", "1", "ShellRemoval", "shell_removal ", "disabled"})
        EXPECT_THROW(ParseContactActivity(text), std::invalid_argument);
}
}
