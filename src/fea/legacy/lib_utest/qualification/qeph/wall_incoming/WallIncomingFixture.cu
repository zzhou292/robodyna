#include "WallIncomingFixture.h"
#include "../wall_coupled/ScreenedWallFixture.h"
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
  auto& w=coupled; auto& r=w.shell;
  r.h=binding.selected_h/refinement;
  intervals=static_cast<unsigned>(PrefixHorizon/r.h);
  if(intervals<1||intervals>MaximumPrefixSteps||intervals*r.h!=PrefixHorizon) return false;
  schedule=wr::AnalyzeWallSwitchingSchedule(model,r.h);
  if(!schedule.passed||schedule.entry_base_epoch>=intervals) return false;
  return InitializeScreenedWall(w,model,velocity,scales);
}

bool IncomingRig::InitializeNative(NativeSequence& sequence) const {
  return InitializeScreenedNative(coupled,velocity,sequence);
}

} // namespace qeph_wall_incoming_test
