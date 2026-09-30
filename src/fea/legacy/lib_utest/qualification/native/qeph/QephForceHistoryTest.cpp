#include "QephForceTestFixture.h"

namespace {
using namespace qeph_force_test;

TEST(QephHistory, PreparedValuesBindEveryReferenceFieldAndRejectLateInvalidInputs) {
  const auto input=Rectangle(); Reference reference;
  ASSERT_EQ(Initialize(input,reference),Status::kSuccess);
  HistoryValues values; values.thickness=input.thickness;
  values.stress={{1,2,3,4,5}}; values.material_stress={{6,7,8,9,10}};
  values.bending_stress={{11,12,13}};
  for (unsigned i=0;i<12;++i) values.stabilization[i]=14+i;
  for (unsigned i=0;i<8;++i) values.strain_curvature[i]=.01*(i+1);
  values.internal_work={{-2,3}}; values.hourglass_viscous_work=4;
  History output; ASSERT_EQ(PreparePrescribedHistory(reference,values,{.125,72},output),Status::kSuccess);
  SameHistory(output.data(),values); EXPECT_TRUE(output.matches_reference(reference));
  const auto before=Bytes(output);
  for (unsigned variant=0;variant<5;++variant) {
    SCOPED_TRACE(variant); auto bad=values;
    if (variant==0) bad.stabilization[11]=std::numeric_limits<double>::quiet_NaN();
    if (variant==1) bad.hourglass_viscous_work=std::numeric_limits<double>::infinity();
    if (variant==2) bad.thickness=0;
    if (variant==3) bad.active=.5;
    const HistoryStamp stamp{variant==4?-1.:.125,72};
    EXPECT_EQ(PreparePrescribedHistory(reference,bad,stamp,output),Status::kInvalidInput);
    EXPECT_EQ(Bytes(output),before);
  }
  for (unsigned field=0;field<6;++field) {
    auto different=input;
    if (field==0) different.node_ids[3]+=1;
    if (field==1) different.position[3].x=std::nextafter(different.position[3].x,0.);
    if (field==2) different.density=std::nextafter(different.density,20.);
    if (field==3) different.young_modulus=std::nextafter(different.young_modulus,3e6);
    if (field==4) different.poisson_ratio=std::nextafter(different.poisson_ratio,.4);
    if (field==5) different.thickness=std::nextafter(different.thickness,.2);
    Reference other; ASSERT_EQ(Initialize(different,other),Status::kSuccess);
    EXPECT_FALSE(output.matches_reference(other));
  }
  Reference identical; ASSERT_EQ(Initialize(input,identical),Status::kSuccess);
  EXPECT_TRUE(output.matches_reference(identical));
}

TEST(QephHistory, StressFreeSequencePreservesAllHistoryAndMatchesQ1Kinematics) {
  for (double warp:{0.,.05}) {
    const auto input=Rectangle(warp); Reference reference; History history;
    ASSERT_EQ(Initialize(input,reference),Status::kSuccess);
    ASSERT_EQ(InitializeHistory(reference,{.125,72},history),Status::kSuccess);
    const auto zero=history.data();
    for (unsigned step=0;step<4;++step) {
      const auto interval=Interval(input,history,.001); const auto saved=Bytes(history);
      ForceTrial output; ASSERT_EQ(EvaluateForce(reference,history,interval,output),Status::kSuccess);
      EXPECT_EQ(Bytes(history),saved); SameHistory(output.proposed_history.data(),zero);
      for (unsigned n=0;n<4;++n) { Near(output.internal_force[n],{}); Near(output.internal_couple[n],{}); }
      Kinematics q1; ASSERT_EQ(EvaluatePrescribed(reference,interval,q1),Status::kSuccess);
      SameGeometry(output.kinematics,q1);
      EXPECT_DOUBLE_EQ(output.proposed_history.stamp().time,interval.base_time+interval.dt);
      EXPECT_EQ(output.proposed_history.stamp().sample_index,interval.sample_index);
      history=output.proposed_history;
    }
  }
}

TEST(QephHistory, LoadingStationaryAndReversalKeepMaterialStressSeparateFromInstantaneousDamping) {
  const auto input=Rectangle(); Reference reference; History history;
  ASSERT_EQ(Initialize(input,reference),Status::kSuccess);
  ASSERT_EQ(InitializeHistory(reference,{},history),Status::kSuccess);
  constexpr double dt=1e-3,rate=.02;
  const double a=input.young_modulus/(1-input.poisson_ratio*input.poisson_ratio);
  const double b=input.poisson_ratio*a;
  const double viscosity=1.414*.015*input.density*std::sqrt(input.young_modulus/input.density)*std::sqrt(2.);
  std::array<double,5> material{},previous_total{};
  double work=0,thickness=input.thickness,strain=0;
  // Two load steps detect recursive damping; stationary drops only DM; reverse
  // checks the old total FOR contribution to source trapezoidal internal work.
  for (double signed_rate:{rate,rate,0.,-rate,-rate}) {
    auto interval=Interval(input,history,dt); Mode(interval,0,signed_rate);
    ForceTrial output; ASSERT_EQ(EvaluateForce(reference,history,interval,output),Status::kSuccess);
    const double increment=signed_rate*dt;
    material[0]+=a*increment; material[1]+=b*increment;
    auto total=material; total[0]+=viscosity*signed_rate; total[1]+=.5*viscosity*signed_rate;
    work+=.5*(input.thickness*2)*(previous_total[0]+total[0])*increment;
    thickness*=1-input.poisson_ratio*increment/(1-input.poisson_ratio); strain+=increment;
    const auto& proposed=output.proposed_history.data();
    Near(proposed.material_stress,material); Near(proposed.stress,total);
    Near(proposed.internal_work[0],work,kEnergy); Near(proposed.internal_work[1],0.,kEnergy);
    Near(proposed.thickness,thickness); Near(proposed.strain_curvature[0],strain,2e-12);
    for (double hg:proposed.stabilization) Near(hg,0.);
    previous_total=total; history=output.proposed_history;
  }
  Near(history.data().material_stress,std::array<double,5>{});
  EXPECT_GT(std::abs(history.data().stress[0]),1.); // Current reversal DM persists only in FOR.
}

TEST(QephHistory, AllTwelveStabilizationValuesPersistAndUpdateWithoutStateAliasing) {
  const auto input=Rectangle(.05); Reference reference; History zero,seeded;
  ASSERT_EQ(Initialize(input,reference),Status::kSuccess);
  ASSERT_EQ(InitializeHistory(reference,{},zero),Status::kSuccess);
  auto values=zero.data();
  for (unsigned i=0;i<12;++i) values.stabilization[i]=.03*(i+1)*(i%2?-1:1);
  values.internal_work={{.012,.023}}; values.hourglass_viscous_work=.034;
  ASSERT_EQ(PreparePrescribedHistory(reference,values,{},seeded),Status::kSuccess);
  auto interval=Interval(input,zero,.001);
  for (unsigned n=0;n<4;++n) {
    const double s=n%2?-1.:1.;
    interval.velocity_midpoint[n]={s*.003,s*.004,s*.002};
    interval.omega_midpoint[n]={s*.005,s*.006,0};
  }
  const auto original=Bytes(seeded); ForceTrial from_zero,from_seed;
  ASSERT_EQ(EvaluateForce(reference,zero,interval,from_zero),Status::kSuccess);
  ASSERT_EQ(EvaluateForce(reference,seeded,interval,from_seed),Status::kSuccess);
  EXPECT_EQ(Bytes(seeded),original);
  double change=0;
  for (unsigned i=0;i<12;++i) {
    const double delta=from_zero.proposed_history.data().stabilization[i]; change+=std::abs(delta);
    Near(from_seed.proposed_history.data().stabilization[i],values.stabilization[i]+delta);
  }
  EXPECT_GT(change,0.);
  EXPECT_GT(from_zero.proposed_history.data().hourglass_viscous_work,0.);
  Near(from_seed.proposed_history.data().hourglass_viscous_work-values.hourglass_viscous_work,
       from_zero.proposed_history.data().hourglass_viscous_work,1e-17);
  const auto stationary=Interval(input,from_seed.proposed_history,.001); ForceTrial held;
  ASSERT_EQ(EvaluateForce(reference,from_seed.proposed_history,stationary,held),Status::kSuccess);
  auto expected_hold=from_seed.proposed_history.data(); expected_hold.stress=expected_hold.material_stress;
  SameHistory(held.proposed_history.data(),expected_hold);
}

TEST(QephHistory, AlternatingMembraneAndDirectorModesHaveLinearForcesAndQuadraticWork) {
  const auto input=Rectangle(); Reference reference; History zero;
  ASSERT_EQ(Initialize(input,reference),Status::kSuccess);
  ASSERT_EQ(InitializeHistory(reference,{},zero),Status::kSuccess);
  for (unsigned mode=0;mode<5;++mode) {
    SCOPED_TRACE(mode); auto first=Interval(input,zero,.001),twice=first;
    for (unsigned n=0;n<4;++n) {
      const double s=(n%2?-1.:1.)*.001;
      if (mode==0) first.velocity_midpoint[n].x=s;
      if (mode==1) first.velocity_midpoint[n].y=s;
      if (mode==2) first.omega_midpoint[n].y=s;
      if (mode==3) first.omega_midpoint[n].x=-s;
      if (mode==4) first.velocity_midpoint[n].z=s;
      twice.velocity_midpoint[n]=Scale(first.velocity_midpoint[n],2);
      twice.omega_midpoint[n]=Scale(first.omega_midpoint[n],2);
    }
    ForceTrial a,b;
    ASSERT_EQ(EvaluateForce(reference,zero,first,a),Status::kSuccess);
    ASSERT_EQ(EvaluateForce(reference,zero,twice,b),Status::kSuccess);
    for (unsigned n=0;n<4;++n) {
      Near(b.internal_force[n],Scale(a.internal_force[n],2));
      Near(b.internal_couple[n],Scale(a.internal_couple[n],2));
    }
    double stored=0;
    for (unsigned i=0;i<12;++i) {
      Near(b.proposed_history.data().stabilization[i],2*a.proposed_history.data().stabilization[i]);
      stored+=std::abs(a.proposed_history.data().stabilization[i]);
    }
    EXPECT_GT(stored,0.);
    for (unsigned i=0;i<2;++i)
      Near(b.proposed_history.data().internal_work[i],4*a.proposed_history.data().internal_work[i],kEnergy);
    Near(b.proposed_history.data().hourglass_viscous_work,4*a.proposed_history.data().hourglass_viscous_work,kEnergy);
    EXPECT_GT(a.proposed_history.data().hourglass_viscous_work,0.);
  }
}

TEST(QephHistory, FixedPlanarIdrilZeroLeavesNodalNormalSpinInactive) {
  const auto input=Rectangle(); Reference reference; History zero;
  ASSERT_EQ(Initialize(input,reference),Status::kSuccess);
  ASSERT_EQ(InitializeHistory(reference,{},zero),Status::kSuccess);
  const auto stationary=Interval(input,zero,.001); auto drilling=stationary;
  for (unsigned n=0;n<4;++n) drilling.omega_midpoint[n].z=(n%2?-1.:1.)*(n+1);
  ForceTrial a,b;
  ASSERT_EQ(EvaluateForce(reference,zero,stationary,a),Status::kSuccess);
  ASSERT_EQ(EvaluateForce(reference,zero,drilling,b),Status::kSuccess);
  SameTrial(a,b);
  for (unsigned n=0;n<4;++n) { Near(b.internal_force[n],{}); Near(b.internal_couple[n],{}); }
  RecordProperty("drilling_response_qualified","false");
  RecordProperty("fixed_native_idril","0");
}

TEST(QephHistory, InvalidStampForeignHistoryAndLateThicknessFailurePreserveEntireOutputThenRetry) {
  const auto input=Rectangle(); Reference reference; History initial;
  ASSERT_EQ(Initialize(input,reference),Status::kSuccess);
  ASSERT_EQ(InitializeHistory(reference,{.125,72},initial),Status::kSuccess);
  const auto valid=Interval(input,initial,.001); ForceTrial output;
  ASSERT_EQ(EvaluateForce(reference,initial,valid,output),Status::kSuccess);
  const auto saved_output=Bytes(output);
  const auto saved_base=Bytes(initial);
  for (unsigned failure=0;failure<5;++failure) {
    auto invalid=valid;
    if (failure==0) invalid.base_time=std::nextafter(valid.base_time,1.);
    if (failure==1) invalid.sample_index+=1;
    if (failure==2) invalid.dt=std::numeric_limits<double>::denorm_min();
    if (failure==3) invalid.position_endpoint[3]=invalid.position_endpoint[0];
    if (failure==4) invalid.omega_midpoint[3].z=std::numeric_limits<double>::infinity();
    EXPECT_NE(EvaluateForce(reference,initial,invalid,output),Status::kSuccess);
    EXPECT_EQ(Bytes(output),saved_output); EXPECT_EQ(Bytes(initial),saved_base);
  }
  History overflow;
  ASSERT_EQ(InitializeHistory(reference,{.125,std::numeric_limits<std::uint64_t>::max()},overflow),Status::kSuccess);
  auto overflowing=valid; overflowing.sample_index=0;
  EXPECT_EQ(EvaluateForce(reference,overflow,overflowing,output),Status::kInvalidInput);
  EXPECT_EQ(Bytes(output),saved_output);
  auto foreign_input=input; foreign_input.node_ids[0]+=1; Reference foreign; History foreign_history;
  ASSERT_EQ(Initialize(foreign_input,foreign),Status::kSuccess);
  ASSERT_EQ(InitializeHistory(foreign,{.125,72},foreign_history),Status::kSuccess);
  EXPECT_EQ(EvaluateForce(reference,foreign_history,valid,output),Status::kInvalidReference);
  EXPECT_EQ(Bytes(output),saved_output);
  // Finite geometry/history; native LAW1 predicts negative reported THKN.
  // The supported-domain guard fires after geometry/material evaluation, before
  // the original MAX would hide the invalid state. ITHK0 force thickness stays t.
  auto late=valid; Mode(late,0,4./late.dt);
  EXPECT_EQ(EvaluateForce(reference,initial,late,output),Status::kNativeFailure);
  EXPECT_EQ(Bytes(output),saved_output); EXPECT_EQ(Bytes(initial),saved_base);
  ForceTrial clean;
  ASSERT_EQ(EvaluateForce(reference,initial,valid,clean),Status::kSuccess);
  ASSERT_EQ(EvaluateForce(reference,initial,valid,output),Status::kSuccess);
  SameTrial(clean,output); EXPECT_EQ(Bytes(initial),saved_base);
}
}  // namespace
