// SPDX-License-Identifier: AGPL-3.0-or-later
#include "GeometryFixture.h"
#include "../qbat_binding/Fixture.h"
#include "lib_src/collision/NodalWallContactArena.h"
namespace physical_wall_test {
TEST(PhysicalWallHostPreparation, CompleteSharedFixtureWallReferenceAndProjectedEnvelopePrepareBeforeRuntime) {
  qbat_binding_test::Fixture source;
  fe::ShellBatchBinding binding;
  ASSERT_EQ(binding.InitializeFormulations(source.Input()).status,fe::ShellBindingStatus::Success);
  std::vector<fe::NodalDomainNode> nodes;
  std::vector<double> coordinates;
  for(std::size_t n=0;n<binding.node_count();++n) {
    const auto& node=binding.nodes()[n];
    nodes.push_back({node.source_id,node.position});
    coordinates.insert(coordinates.end(),{node.position.x,node.position.y,node.position.z});
  }
  fe::NodalNodeDomain domain;
  ASSERT_TRUE(domain.Initialize({1,nodes.data(),nodes.size()}));
  fe::ShellNodeMap mapping;
  ASSERT_TRUE(mapping.Initialize(binding,domain));
  Geometry geometry(binding,mapping,coordinates),wrong(binding,mapping,coordinates,true);
  ASSERT_GT(wrong.weights.parent(2).area.value,geometry.weights.parent(2).area.value);
  c::PlanarWallGeometry wall;
  const auto wall_report=wall.Initialize(geometry.Wall());
  ASSERT_EQ(wall_report.status,c::PlanarContactStatus::Ok)<<wall_report.message;
  fe::NodalStamp stamp;
  stamp.owner_id=1;
  stamp.node_count=domain.node_count();
  stamp.fixed_dt=0x1p-28;
  stamp.has_rotations=true;
  stamp.temporal_scheme=fe::NodalTemporalScheme::StaggeredHalfKickStart;
  stamp.velocity_phase=fe::NodalVelocityPhase::Collocated;
  const auto config=Settings(stamp,718,719);
  namespace d=c::nodal_wall_device_detail;
  d::PreparedModel result;
  const c::VectorView x{coordinates.data(),static_cast<std::uint32_t>(stamp.node_count),3,1};
  auto bad_box=geometry.Motion();
  bad_box.minimum.x=-1; bad_box.maximum.x=1;
  EXPECT_EQ(d::PreparePhysicalModel(config,geometry.Wall(),geometry.weights,x,bad_box,&result).status,
      c::NodalWallDeviceStatus::GeometryFailure);
  EXPECT_FALSE(result.prepared());
  bad_box=geometry.Motion(); bad_box.maximum.y=1.1;
  EXPECT_EQ(d::PreparePhysicalModel(config,geometry.Wall(),geometry.weights,x,bad_box,&result).status,
      c::NodalWallDeviceStatus::GeometryFailure);
  auto excess=config; excess.law.maximum_penetration=.001;
  EXPECT_EQ(d::PreparePhysicalModel(excess,geometry.Wall(),geometry.weights,x,geometry.Motion(),&result).status,
      c::NodalWallDeviceStatus::PointFailure);
  auto report=d::PreparePhysicalModel(config,geometry.Wall(),geometry.weights,x,geometry.Motion(),&result);
  ASSERT_EQ(report.status,c::NodalWallDeviceStatus::Ok)<<report.message;
  EXPECT_TRUE(result.model().prepared);
  EXPECT_EQ(result.model().parent_count,4u);
  EXPECT_EQ(result.model().node_count,5u);
  EXPECT_TRUE(result.model().coverage.covered);
  EXPECT_EQ(result.model().coverage.physical.minimum.x,.039);
  EXPECT_EQ(result.model().coverage.physical.maximum.x,.039);
  for(unsigned p=0;p<4;++p)
    EXPECT_EQ(result.model().parents[p].parent_element_id,geometry.weights.parent(p).parent_element_id);
}
} // namespace physical_wall_test
