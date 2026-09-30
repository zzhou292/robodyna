#include "ScreenedWallFixture.h"
#include <algorithm>

namespace qeph_wall_test {
bool InitializeScreenedWall(WallRig& w,const screened_wall::WallRecurrenceModel& model,
                            ScreenedVelocity& velocity,LedgerScales& scales) {
  auto& r=w.shell; const auto& native=model.native();
  r.mass.fill(0); r.inertia.fill(0); r.physical.fill(0); r.added.fill(0); r.x.fill(0);
  for(unsigned e=0;e<r.count;++e) {
    const auto& saved=native.reference[e].data(); q::ReferenceInput input;
    for(unsigned i=0;i<4;++i) {
      input.node_ids[i]=saved.input.node_ids[i];
      input.position[i]={saved.input.position[i].x,saved.input.position[i].y,saved.input.position[i].z};
    }
    input.density=saved.input.density; input.young_modulus=saved.input.young_modulus;
    input.poisson_ratio=saved.input.poisson_ratio; input.thickness=saved.input.thickness;
    const auto status=q::InitializeReference(input,r.element[e].reference);
    EXPECT_EQ(status,q::Status::kSuccess); if(status!=q::Status::kSuccess) return false;
    const auto& parent=model.weights().parent(e);
    w.parents[e].parent_element_id=parent.parent_element_id; w.parents[e].parent_face_id=parent.parent_face_id;
    w.parents[e].feature_id=parent.feature_id;
    for(unsigned i=0;i<4;++i) {
      const auto n=native.connectivity[e][i]; r.element[e].nodes[i]=n; w.parents[e].nodes[i]=n;
      EXPECT_EQ(parent.nodes[i],n); EXPECT_EQ(input.position[i].x,-screened_wall::InitialGap);
      r.x[3*n]=input.position[i].x; r.x[3*n+1]=input.position[i].y; r.x[3*n+2]=input.position[i].z;
      r.physical[n]+=saved.physical_inertia[i]; r.added[n]+=saved.added_inertia[i];
    }
  }
  // Use the actual screened native total mass/J bits, independently of contact
  // area and of the separately retained physical/added rotary partitions.
  for(unsigned n=0;n<r.n;++n) {
    r.mass[n]=native.mass[n]; r.inertia[n]=native.inertia[n];
    r.inverse[n]=1/r.mass[n]; r.inverse_j[n]=1/r.inertia[n]; velocity[3*n]=screened_wall::ImpactSpeed;
    scales.energy+=.5L*r.mass[n]*screened_wall::ImpactSpeed*screened_wall::ImpactSpeed;
    scales.linear+=static_cast<long double>(r.mass[n])*screened_wall::ImpactSpeed;
  }
  scales.angular=Side*scales.linear;
  if(::testing::Test::HasFailure()) return false;
  fe::NodalStateConfig config; config.node_count=r.n; config.fixed_dt=r.h;
  config.temporal_scheme=fe::NodalTemporalScheme::StaggeredHalfKickStart;
  const auto owner=r.owner.Initialize(config,{r.x.data(),velocity.data(),r.zero.data(),r.n,r.orientation.data()},
                                     r.inverse.data(),{r.fixed.data(),r.fixed.data(),r.inverse_j.data()});
  EXPECT_EQ(owner.status,fe::NodalStatus::Ok)<<owner.message;
  if(owner.status!=fe::NodalStatus::Ok||!w.InitializeParticipants()) return false;
  for(unsigned e=0;e<r.count;++e) {
    const auto& actual=w.weights.parent(e); const auto& expected=model.weights().parent(e);
    EXPECT_EQ(actual.parent_element_id,expected.parent_element_id);
    EXPECT_EQ(actual.area.value,expected.area.value); EXPECT_EQ(actual.area.lower,expected.area.lower);
    EXPECT_EQ(actual.area.upper,expected.area.upper); EXPECT_EQ(actual.area.error,expected.area.error);
    EXPECT_EQ(actual.share.value,expected.share.value);
    EXPECT_EQ(actual.share.lower,expected.share.lower); EXPECT_EQ(actual.share.upper,expected.share.upper);
    EXPECT_EQ(actual.share.error,expected.share.error);
  }
  return !::testing::Test::HasFailure();
}
bool InitializeScreenedNative(const WallRig& w,const ScreenedVelocity& velocity,NativeSequence& sequence) {
  if(!sequence.Initialize(w.shell)) return false;
  // Same explicit startup assignment as the retained uniform-translation
  // qualification; NativeSequence keeps its existing clock/history equations.
  std::copy_n(velocity.begin(),3*w.shell.n,sequence.state.v.begin());
  return true;
}
} // namespace qeph_wall_test
