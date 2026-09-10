#include "VehicleWallFixture.h"
namespace vehicle_wall_device_test {
TEST(VehicleWallSource, CompleteNativeCountIncidenceAndLateMassFailurePreservePreparedModel) {
  auto f=std::make_unique<Fixture>();ASSERT_TRUE(f->Prepare());detail::PreparedModel prepared;
  const auto initialize=[&]{return detail::PrepareModel(f->Config(),f->wall.view(),f->weights,f->Positions(),
    f->inverse.data(),f->fixed.data(),f->motion,&prepared);};
  ASSERT_EQ(initialize().status,Code::Ok);ASSERT_NO_FATAL_FAILURE(CheckIncidence(*f,prepared));
  ASSERT_EQ(prepared.layout().bytes,1078530400u);
  const auto* held=prepared.data();const auto rate=nodal_wall_owner_test::Bytes(prepared.model().rate);
  const auto old=f->inverse.back();f->inverse.back()=0;
  const auto failure=initialize();EXPECT_EQ(failure.status,Code::InvalidMass);EXPECT_EQ(failure.node,f->n-1);
  EXPECT_EQ(prepared.data(),held);EXPECT_EQ(nodal_wall_owner_test::Bytes(prepared.model().rate),rate);
  f->inverse.back()=old;ASSERT_EQ(initialize().status,Code::Ok);ASSERT_NO_FATAL_FAILURE(CheckIncidence(*f,prepared));
  EXPECT_EQ(nodal_wall_owner_test::Bytes(prepared.model().rate),rate);
  RecordProperty("scope","full-count synthetic Q/T geometry, exact native mass/J; no original vehicle runtime claim");
  RecordProperty("contact_device_payload_bytes",std::to_string(prepared.layout().bytes));
  RecordProperty("contact_host_payload_bytes",std::to_string(detail::HostPreparationBytes(prepared.layout())));
}
} // namespace vehicle_wall_device_test
