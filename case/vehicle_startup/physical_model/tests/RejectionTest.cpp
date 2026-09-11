#include "Support.h"

namespace crash::cases::vehicle_startup::physical_model::test {
TEST(VehiclePhysicalModelOriginal,ExactCapsAndLateRigidFailurePreservePublishedModelAndRetry) {
    auto model = Actual();
    const auto* before = model.coefficients().nodes().data();
    Limits cap; cap.host_bytes = model.forecast().total_bytes - 1;
    EXPECT_THROW(model = VehiclePhysicalModel::Prepare(Source(), Shells(), cap), std::runtime_error);
    EXPECT_EQ(model.coefficients().nodes().data(), before);
    ++cap.host_bytes;
    EXPECT_EQ(VehiclePhysicalModel::Preflight(Source(), Shells(), cap).total_bytes, cap.host_bytes);
    cap = {}; cap.plain_bytes = model.plain_groups().startup_payload_bytes() - 1;
    EXPECT_THROW(model = VehiclePhysicalModel::Prepare(Source(), Shells(), cap), std::runtime_error);
    EXPECT_EQ(model.coefficients().nodes().data(), before);
    ++cap.plain_bytes;
    EXPECT_NO_THROW(model = VehiclePhysicalModel::Prepare(Source(), Shells(), cap));
    EXPECT_TRUE(model.coefficients().Matches(Actual().coefficients()));
    EXPECT_EQ(model.rigid_assembly().groups().size(), 773);
    const auto copy = [=] { return VehiclePhysicalModel(model); }();
    EXPECT_EQ(copy.beams().nodes(), model.beams().nodes());
    EXPECT_TRUE(copy.solids().SharesStorage(model.solids()));
}
} // namespace crash::cases::vehicle_startup::physical_model::test
