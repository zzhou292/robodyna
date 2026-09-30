// SPDX-License-Identifier: MIT
#include "../rigid_assembly_owner/Fixture.h"
namespace physical_owner_forecast_test {
using namespace rigid_assembly_owner_test;
TEST(PhysicalOwnerForecastCuda, ExactAllocatedBytesAndRejectedOwnerRetry) {
  Fixture source;
  source.DependentInverses(true);
  auto config = source.Config();
  auto cin = source.Cin();
  const auto forecast = fe::FENodalState::ForecastAssemblyCin(config,source.binding,cin);
  ASSERT_EQ(forecast.report.status,Code::Ok);
  config.max_device_bytes = forecast.device_bytes - 1;
  fe::FENodalState owner;
  EXPECT_EQ(owner.Initialize(config,source.Kinematics(),source.im.data(),source.Dofs(),
      source.binding,&cin).status,Code::ResourceLimit);
  EXPECT_EQ(owner.allocations().device_bytes,0u);
  ++config.max_device_bytes;
  const auto initialized = owner.Initialize(config,source.Kinematics(),source.im.data(),source.Dofs(),
      source.binding,&cin);
  ASSERT_EQ(initialized.status,Code::Ok) << initialized.message;
  EXPECT_EQ(owner.allocations().device_bytes,forecast.device_bytes);
  EXPECT_EQ(owner.accepted().epoch,0u);
  EXPECT_EQ(owner.ValidateRigidAssemblyBinding(source.binding).status,Code::Ok);
}
} // namespace physical_owner_forecast_test
