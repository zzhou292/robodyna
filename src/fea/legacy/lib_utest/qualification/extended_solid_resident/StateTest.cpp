// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
#include "lib_src/elements/solids/resident/Storage.h"
namespace extended_resident_test {
template<class Traits> void Trajectory(const typename Traits::Parent& parent,
    const typename Traits::Material& material) {
  d::ExtendedScratch<Traits> scratch;
  d::State<Traits> state;
  ASSERT_EQ(d::InitializeExtendedState<Traits>(parent,material,{11.123,-.37,.129},scratch,state),0);
  ASSERT_TRUE(d::ValidResult(parent,material,state,0,0));
  EXPECT_EQ(state.cache.stiffness.rotation_nm,0);
  EXPECT_EQ(state.cache.stiffness.translation_n_m,.25*state.cache.diagnostics.raw_stiffness_n_m);
  for (unsigned step = 0; step < 3; ++step) {
    auto interval = Traits::Phase(state.history.stamp().time_s,1e-8,step);
    Fill<Traits>(parent,interval,1+.0001*double(step+1));
    typename Traits::ForceScratch direct;
    ASSERT_EQ(Traits::EvaluateScratch(parent,material,state.history,interval,direct),Traits::success);
    d::State<Traits> next;
    ASSERT_EQ(d::UpdateExtendedState<Traits>(parent,material,state,interval,scratch,next),0);
    const auto& trial = Traits::TrialValue(direct);
    ASSERT_TRUE(d::ValidResult(parent,material,next,interval.base_time_s+interval.dt_s,step+1));
    for (unsigned n = 0; n < 8; ++n)
      EXPECT_TRUE(fe::shell_startup_detail::SameVector(next.cache.rhs_force_n[n],trial.rhs_force_n[n]));
    const auto saved = next;
    auto bad = interval;
    Traits::Node(bad,7,{0,0,std::numeric_limits<double>::quiet_NaN()},{});
    EXPECT_NE(d::UpdateExtendedState<Traits>(parent,material,state,bad,scratch,next),0);
    EXPECT_EQ(std::memcmp(&saved,&next,sizeof(next)),0);
    ASSERT_EQ(d::UpdateExtendedState<Traits>(parent,material,state,interval,scratch,next),0);
    EXPECT_EQ(std::memcmp(&saved,&next,sizeof(next)),0);
    state = next;
  }
}
TEST(ExtendedResidentState, BothTypedExternalScratchPathsRetainPhaseAndLateFailureRetry) {
  for(int flag : {1,2}) {
    Fixture fixture;
    fixture.foam_input.hysteresis=flag==1?0:1;fixture.PrepareFoam();
    const auto domain = fixture.Domain();
    s::Model model;
    ASSERT_TRUE(model.Initialize(domain,fixture.Input()));
    Trajectory<d::Traits18Law44>(model.solid18_law44()[0],model.materials44()[0].value);
    Trajectory<d::Traits18Law90>(model.solid18_law90()[0],model.materials90()[0].value);
  }
}
TEST(ExtendedResidentState, FinalFamilyCompleteFieldsAndDiagnosticsIdentityAreChecked) {
  Fixture fixture;
  const auto domain = fixture.Domain();
  s::Model model;
  ASSERT_TRUE(model.Initialize(domain,fixture.Input()));
  d::State<d::Traits18Law90> state;
  d::ExtendedScratch<d::Traits18Law90> scratch;
  const auto& parent = model.solid18_law90()[0];
  const auto& material = model.materials90()[0].value;
  ASSERT_EQ(d::InitializeExtendedState<d::Traits18Law90>(parent,material,{},scratch,state),0);
  ASSERT_TRUE(d::ValidResult(parent,material,state,0,0));
  state.cache.rhs_force_n[7].z = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(d::ValidResult(parent,material,state,0,0));
  s::BatchDiagnostics a,b;
  a.parent_count[4]=1; b=a;
  EXPECT_TRUE(d::SameDiagnostics(a,b));
  b.parent_count[4]=2;
  EXPECT_FALSE(d::SameDiagnostics(a,b));
  b=a; b.native_internal_work_increment_j[3]=1;
  EXPECT_FALSE(d::SameDiagnostics(a,b));
}
} // namespace extended_resident_test
