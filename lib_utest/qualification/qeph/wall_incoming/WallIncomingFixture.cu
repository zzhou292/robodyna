#include "WallIncomingFixture.h"
#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <stdexcept>
#include <utility>

namespace qeph_wall_incoming_test {
namespace {
wr::WallRecurrenceModel Model(unsigned cells) {
  wr::WallRecurrenceModel model; std::string error;
  if(!wr::BuildWallRecurrenceModel(cells,model,error)) throw std::runtime_error(error);
  return model;
}
WallExperiment Experiment(const wr::WallRecurrenceModel& model) {
  WallExperiment e; e.qualification=IncomingQualification; e.shell_configuration=IncomingShellConfiguration;
  e.wall_configuration=IncomingWallConfiguration; e.wall_binding=IncomingWallBinding; e.law=model.law();
  e.startup={q::BatchStartupKind::ReferenceUniformTranslation,{wr::ImpactSpeed,0,0}};
  return e;
}
}
bool ValidScreenBinding(const ScreenBinding& b) {
  if((b.selected_h!=H0&&b.selected_h!=H0/2)||b.decision_sha256.size()!=64) return false;
  for(char c:b.decision_sha256) if(!((c>='0'&&c<='9')||(c>='a'&&c<='f'))) return false;
  return true;
}
bool ReadScreenBinding(ScreenBinding& out,std::string& error) {
  const char* step=std::getenv("TL_CW2_SELECTED_H"); const char* hash=std::getenv("TL_CW2_SCREEN_SHA256");
  if(!step||!hash) { error="CW2 requires root-authenticated TL_CW2_SELECTED_H and TL_CW2_SCREEN_SHA256"; return false; }
  ScreenBinding result;
  if(std::strcmp(step,"0x1p-24")==0) result.selected_h=H0;
  else if(std::strcmp(step,"0x1p-25")==0) result.selected_h=H0/2;
  else { error="CW2 step must be exactly 0x1p-24 or 0x1p-25 from the final screen"; return false; }
  result.decision_sha256=hash;
  if(!ValidScreenBinding(result)) { error="CW2 requires an exact lowercase SHA256 decision binding"; return false; }
  out=std::move(result); error.clear(); return true;
}
IncomingRig::IncomingRig(unsigned cells,const ScreenBinding& selected)
  :binding(selected),model(Model(cells)),coupled(cells,Experiment(model)) {}
bool IncomingRig::Initialize(unsigned refinement) {
  if(initialization_attempted||!ValidScreenBinding(binding)||(refinement!=1&&refinement!=2)) return false;
  initialization_attempted=true;
  auto& w=coupled; auto& r=w.shell; const auto& native=model.native();
  r.h=binding.selected_h/refinement;
  intervals=static_cast<unsigned>(PrefixHorizon/r.h);
  if(intervals<1||intervals>MaximumPrefixSteps||intervals*r.h!=PrefixHorizon) return false;
  schedule=wr::AnalyzeWallSwitchingSchedule(model,r.h);
  if(!schedule.passed||schedule.entry_base_epoch>=intervals) return false;
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
      EXPECT_EQ(parent.nodes[i],n); EXPECT_EQ(input.position[i].x,-wr::InitialGap);
      r.x[3*n]=input.position[i].x; r.x[3*n+1]=input.position[i].y; r.x[3*n+2]=input.position[i].z;
      r.physical[n]+=saved.physical_inertia[i]; r.added[n]+=saved.added_inertia[i];
    }
  }
  // Use the actual screened native total mass/J bits, independently of contact
  // area and of the separately retained physical/added rotary partitions.
  for(unsigned n=0;n<r.n;++n) {
    r.mass[n]=native.mass[n]; r.inertia[n]=native.inertia[n];
    r.inverse[n]=1/r.mass[n]; r.inverse_j[n]=1/r.inertia[n]; velocity[3*n]=wr::ImpactSpeed;
    scales.energy+=.5L*r.mass[n]*wr::ImpactSpeed*wr::ImpactSpeed;
    scales.linear+=static_cast<long double>(r.mass[n])*wr::ImpactSpeed;
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
bool IncomingRig::InitializeNative(NativeSequence& sequence) const {
  if(!sequence.Initialize(coupled.shell)) return false;
  // Same explicit startup assignment as the retained uniform-translation
  // qualification; NativeSequence keeps its existing clock/history equations.
  std::copy_n(velocity.begin(),3*coupled.shell.n,sequence.state.v.begin());
  return true;
}
} // namespace qeph_wall_incoming_test
