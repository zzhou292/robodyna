// SPDX-License-Identifier: MIT
#include "NativeOracle.h"
#include "../qbat/source_fixture/YarisQbatSourceFixture.h"
namespace qbat_force_test {
namespace source=yaris_qbat_source_fixture;
TEST(QbatForceNative, OriginalFirstFinalMaximumWarpAndMinimumTimestepGeometry) {
  unsigned max_warp=0,min_dt=0;
  double warp=-1,dt=std::numeric_limits<double>::infinity();
  // The independently authenticated full source geometry sweep already has
  // its own owning gate. Select the two extremes without changing source data.
  for(unsigned i=0;i<std::size(source::quads);++i) {
    Fixture f;
    const auto& quad=source::quads[i];
    for(unsigned n=0;n<4;++n) {
      const auto& node=source::nodes[quad.node[n]];
      f.input.quadrilateral.node_ids[n]=node.id;
      f.input.quadrilateral.position[n]={node.position_m[0],node.position_m[1],node.position_m[2]};
    }
    ASSERT_EQ(qb::InitializeReference(f.input,f.reference),qb::Status::kSuccess);
    qb::Geometry geometry;
    ASSERT_EQ(qb::EvaluateGeometry(f.reference,qbat_test::Current(f.input),geometry),qb::Status::kSuccess);
    if(std::abs(geometry.actual_warpage_m)>warp) { warp=std::abs(geometry.actual_warpage_m);max_warp=i; }
    if(f.reference.coefficients().unscaled_element_dt_s<dt) {
      dt=f.reference.coefficients().unscaled_element_dt_s;min_dt=i;
    }
  }
  for(unsigned index:{0u,static_cast<unsigned>(std::size(source::quads)-1),max_warp,min_dt}) {
    SCOPED_TRACE(source::quads[index].id);
    Fixture f;
    const auto& quad=source::quads[index];
    for(unsigned n=0;n<4;++n) {
      const auto& node=source::nodes[quad.node[n]];
      f.input.quadrilateral.node_ids[n]=node.id;
      f.input.quadrilateral.position[n]={node.position_m[0],node.position_m[1],node.position_m[2]};
    }
    ASSERT_EQ(qb::InitializeReference(f.input,f.reference),qb::Status::kSuccess);
    auto accepted=f.Virgin();
    NativeState native(accepted.data());
    for(unsigned step=0;step<48;++step) {
      SCOPED_TRACE(step);
      const auto in=Path(f,step);
      qb::ForceTrial actual;
      ASSERT_EQ(qb::EvaluateForce(f.reference,f.material,f.failure,accepted,in,actual),qb::Status::kSuccess);
      native.Step(f,in);
      CompareNative(actual,native);
      accepted=actual.proposed_history;
    }
  }
}
} // namespace qbat_force_test
