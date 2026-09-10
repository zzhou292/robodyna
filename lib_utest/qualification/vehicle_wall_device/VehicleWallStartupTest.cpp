#include "../surface_contact/NodalWallCapacityFixture.h"
namespace vehicle_wall_device_test {
using namespace nodal_wall_capacity_test;
using nodal_wall_owner_test::Bytes;
namespace {
sc::NodalWallDeviceConfig Vehicle(sc::NodalWallDeviceConfig c) {
  c.limits=sc::NodalWallDeviceLimits::Vehicle();c.max_device_bytes=sc::MaxVehicleNodalWallDeviceBytes;
  c.max_host_bytes=sc::MaxVehicleNodalWallHostBytes;return c;
}
void SameModel(const detail::PreparedModel& a,const detail::PreparedModel& b) {
  const auto& x=a.model();const auto& y=b.model();
  ASSERT_EQ(x.node_count,y.node_count);ASSERT_EQ(x.parent_count,y.parent_count);
  EXPECT_EQ(Bytes(x.rate),Bytes(y.rate));
  EXPECT_EQ(std::memcmp(x.nodes,y.nodes,x.node_count*sizeof(*x.nodes)),0);
  EXPECT_EQ(std::memcmp(x.parents,y.parents,x.parent_count*sizeof(*x.parents)),0);
  EXPECT_EQ(std::memcmp(x.initial_position,y.initial_position,Nodes*sizeof(*x.initial_position)),0);
  EXPECT_EQ(std::memcmp(x.inverse_mass,y.inverse_mass,Nodes*sizeof(*x.inverse_mass)),0);
  EXPECT_EQ(std::memcmp(x.fixed,y.fixed,Nodes*sizeof(*x.fixed)),0);
  EXPECT_EQ(std::memcmp(x.incident_offsets,y.incident_offsets,(Nodes+1)*sizeof(*x.incident_offsets)),0);
  EXPECT_EQ(std::memcmp(x.incident_slots,y.incident_slots,(4*Quads+3*Triangles)*sizeof(*x.incident_slots)),0);
}
}
TEST(VehicleWallStartup, IndexedMixedNativeIncidenceAndRatesMatchLegacyBits) {
  auto f=std::make_unique<Fixture>();ASSERT_TRUE(f->Prepare());
  detail::PreparedModel legacy,vehicle;
  const auto initialize=[&](const auto& c,auto& out) {return detail::PrepareModel(c,f->wall.view(),f->weights,
    f->Positions(),f->inverse.data(),f->fixed.data(),f->motion,&out);};
  ASSERT_EQ(initialize(f->Config(),legacy).status,Code::Ok);
  ASSERT_EQ(initialize(Vehicle(f->Config()),vehicle).status,Code::Ok);
  SameModel(legacy,vehicle);
  // Both malformed physical inputs are reported in the old ascending-node
  // order, after geometry validation, with the original failing parent index.
  for(unsigned fault=0;fault<5;++fault) {
    const auto x=f->x;const auto inverse=f->inverse;const auto masks=f->fixed;
    auto c=f->Config();
    if(fault==0){f->inverse.back()=0;f->inverse[Nodes/2]=0;}
    if(fault==1){f->fixed.back()=1;}
    if(fault==2){f->x[3*(Nodes-1)+1]=2;}
    if(fault==3){f->x[3*(Nodes-1)]=1;}
    if(fault==4){c.law.stiffness_per_area=std::numeric_limits<double>::max();
      f->inverse.back()=std::numeric_limits<double>::max();}
    const auto a=initialize(c,legacy),b=initialize(Vehicle(c),vehicle);
    EXPECT_NE(a.status,Code::Ok)<<fault;EXPECT_EQ(a.status,b.status);EXPECT_EQ(a.node,b.node);EXPECT_EQ(a.parent,b.parent);
    EXPECT_STREQ(a.message,b.message);EXPECT_EQ(a.point.status,b.point.status);SameModel(legacy,vehicle);
    f->x=x;f->inverse=inverse;f->fixed=masks;
  }
  ASSERT_EQ(initialize(Vehicle(f->Config()),vehicle).status,Code::Ok);SameModel(legacy,vehicle);
}
TEST(VehicleWallStartup, ByteAndProfileFailuresRejectBeforeBorrowedPhysicalReadsAndPreservePreparedState) {
  auto f=std::make_unique<Fixture>();ASSERT_TRUE(f->Prepare());detail::PreparedModel out;
  auto c=Vehicle(f->Config());
  ASSERT_EQ(detail::PrepareModel(c,f->wall.view(),f->weights,f->Positions(),f->inverse.data(),f->fixed.data(),f->motion,&out).status,Code::Ok);
  const auto bytes=Bytes(out);const auto* data=out.data();const auto rate=Bytes(out.model().rate);
  const sc::VectorView bad{reinterpret_cast<const double*>(8),Nodes,3,1};
  for(unsigned fault=0;fault<7;++fault) {
    auto invalid=c;
    if(fault==0)invalid.limits.profile=sc::NodalWallDeviceProfile::Legacy;
    if(fault==1)invalid.limits.parents=524289;if(fault==2)invalid.limits.global_nodes=524289;
    if(fault==3)invalid.max_device_bytes=out.layout().bytes-1;
    if(fault==4)invalid.max_host_bytes=detail::HostPreparationBytes(out.layout())-1;
    if(fault==5)invalid.max_device_bytes=sc::MaxVehicleNodalWallDeviceBytes+1;
    if(fault==6)invalid.limits.profile=static_cast<sc::NodalWallDeviceProfile>(-1);
    EXPECT_EQ(detail::PrepareModel(invalid,f->wall.view(),f->weights,bad,bad.data,
      reinterpret_cast<const std::uint8_t*>(8),f->motion,&out).status,Code::ResourceLimit)<<fault;
    EXPECT_EQ(Bytes(out),bytes);EXPECT_EQ(out.data(),data);EXPECT_EQ(Bytes(out.model().rate),rate);
  }
  c.max_device_bytes=out.layout().bytes;c.max_host_bytes=detail::HostPreparationBytes(out.layout());
  ASSERT_EQ(detail::PrepareModel(c,f->wall.view(),f->weights,f->Positions(),f->inverse.data(),f->fixed.data(),f->motion,&out).status,Code::Ok);
}
} // namespace vehicle_wall_device_test
