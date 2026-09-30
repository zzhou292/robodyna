#include "ResponseTestFixture.h"
#include "ResponseBounds.h"
#include <limits>

namespace response_test {
TEST(QephResponseObserver, FrozenModelDictionaryScalesAndPulseAreIndependentOfRefinement) {
  const auto m=Model();
  const long double area=r::Side*r::Side,total=2*area*r::Density*r::Thickness;
  long double mass=0,inertia=0;
  for(unsigned n=0;n<m.nodes;++n) { mass+=m.mass[n]; inertia+=m.inertia[n]; }
  EXPECT_NEAR(static_cast<double>(mass),static_cast<double>(total),2e-12*static_cast<double>(total));
  const long double expected=total*(r::Thickness*r::Thickness+area)/12;
  EXPECT_NEAR(static_cast<double>(inertia),static_cast<double>(expected),2e-12*static_cast<double>(expected));
  const auto fields=r::Dictionary(m); EXPECT_EQ(fields.size(),438u);
  EXPECT_EQ(fields[Field(fields,"element.hourglass[0][2]")].unit,"Pa/m");
  EXPECT_EQ(fields[Field(fields,"element.hourglass[0][2]")].scale,r::Young*r::Theta/r::Side);
  const double moment=r::Young*r::Thickness*r::Thickness*r::Thickness*r::Theta/r::Side;
  EXPECT_EQ(fields[Field(fields,"element.bending_stress[0][1]")].scale,moment/(r::Thickness*r::Thickness));
  EXPECT_FALSE(fields[Field(fields,"element.internal_work[0][0]")].compare);
  EXPECT_EQ(r::PulseFactor(0),0); EXPECT_EQ(r::PulseFactor(r::Pulse),0); EXPECT_EQ(r::PulseFactor(r::Horizon),0);
  EXPECT_DOUBLE_EQ(r::PulseFactor(r::Pulse/2),1); EXPECT_NEAR(r::PulseFactor(r::Pulse/4),.5,4e-16);
  EXPECT_TRUE(std::isnan(r::PulseFactor(-1)));
  auto output=m; const auto before=Bytes(output); std::string error;
  EXPECT_FALSE(r::BuildModel(3,output,error)); EXPECT_EQ(Bytes(output),before);
}
TEST(QephResponseObserver, InitialRestAndSharedEndpointReconstructionKeepTimeAndInertiaPartitions) {
  const auto m=Model(); const auto fields=r::Dictionary(m); auto state=Rest(m);
  auto cache=Histories(m,0,r::H0); r::Sample initial; r::Limits limits; std::string error;
  ASSERT_TRUE(r::Observe(m,r::H0,0,state,cache,0,initial,limits,error));
  EXPECT_FALSE(initial.interval_available); EXPECT_EQ(initial.carried_velocity_time,0);
  EXPECT_EQ(initial.kick_dt,0); EXPECT_EQ(initial.residual,0);
  for(double energy:initial.synchronous_kinetic) EXPECT_EQ(energy,0);
  constexpr unsigned epoch=512; cache=Histories(m,epoch,r::H0);
  // The shared node2 receives a positive internal cache from both cells.
  cache[0].internal_force[1].x=2; cache[1].internal_force[0].x=5;
  cache[0].internal_couple[1].y=3; cache[1].internal_couple[0].y=7;
  state.v[6]=.125; state.omega[7]=.25;
  r::Sample output;
  ASSERT_TRUE(r::Observe(m,r::H0,epoch,state,cache,0,output,limits,error))<<error;
  EXPECT_EQ(output.carried_velocity_time,(epoch-.5)*r::H0);
  const double vx=static_cast<double>(.125L-.5L*r::H0*7/m.mass[2]);
  const double wy=static_cast<double>(.25L-.5L*r::H0*10/m.inertia[2]);
  EXPECT_EQ(output.values[Field(fields,"node.synchronous_velocity[2][0]")],vx);
  EXPECT_EQ(output.values[Field(fields,"node.synchronous_omega[2][1]")],wy);
  EXPECT_EQ(output.values[Field(fields,"node.carried_velocity[2][0]")],.125);
  // Pulse peak force for middle-station node2 is -D*delta/L².
  const double vz=static_cast<double>(-.5L*r::H0*r::BendingScale()*r::Delta/(r::Side*r::Side)/m.mass[2]);
  EXPECT_NEAR(output.values[Field(fields,"node.synchronous_velocity[2][2]")],vz,2e-15*std::abs(vz));
  EXPECT_NEAR(output.synchronous_kinetic[2],static_cast<double>(.5L*m.physical[2]*wy*wy),2e-15*output.synchronous_kinetic[2]);
  EXPECT_NEAR(output.synchronous_kinetic[3],static_cast<double>(.5L*m.added[2]*wy*wy),2e-15*output.synchronous_kinetic[3]);
  EXPECT_EQ(output.carried_kinetic[0],.5*m.mass[2]*.125*.125);
}
TEST(QephResponseObserver, LateNonfiniteHistoryTimeAndQuaternionFailuresPreserveBothOutputs) {
  const auto m=Model(); auto state=Rest(m); const auto cache=Histories(m,1,r::H0);
  r::Sample output; r::Limits limits; std::string error;
  ASSERT_TRUE(r::Observe(m,r::H0,1,state,cache,0,output,limits,error));
  const auto saved=Bytes(output);
  const auto saved_limits=Bytes(limits);
  auto invalid=cache; invalid[1].internal_force[3].z=std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(r::Observe(m,r::H0,1,state,invalid,0,output,limits,error));
  EXPECT_EQ(Bytes(output),saved); EXPECT_EQ(Bytes(limits),saved_limits);
  EXPECT_FALSE(r::Observe(m,r::H0,2,state,cache,0,output,limits,error));
  EXPECT_EQ(Bytes(output),saved); EXPECT_EQ(Bytes(limits),saved_limits);
  state.q[4*(m.nodes-1)]=2;
  EXPECT_FALSE(r::Observe(m,r::H0,1,state,cache,0,output,limits,error));
  EXPECT_EQ(Bytes(output),saved); EXPECT_EQ(Bytes(limits),saved_limits);
  ASSERT_TRUE(r::Observe(m,r::H0,1,Rest(m),cache,0,output,limits,error)); EXPECT_EQ(Bytes(output),saved);
}
TEST(QephResponseObserver, AnalyticRefinementDifferencesAndEnergyUseFixedScales) {
  const std::array<r::Run,3> runs{{Synthetic(1),Synthetic(2),Synthetic(4)}};
  const auto result=r::Compare(runs); ASSERT_TRUE(result.passed)<<result.diagnostic;
  EXPECT_NEAR(result.coarse_medium.maximum,.004,2e-15); EXPECT_NEAR(result.medium_fine.maximum,.002,2e-15);
  EXPECT_EQ(result.energy_normalization,r::ExperimentEnergy(runs[0].model)*.001);
  EXPECT_NEAR(result.residual_ratios[0],.004,2e-15); EXPECT_NEAR(result.residual_ratios[1],.002,2e-15);
  EXPECT_EQ(result.coarse_medium.time,r::Horizon/2);
}
TEST(QephResponseObserver, NonVacuousFloorsAndRefinementFailuresCannotBecomeAdmission) {
  std::array<r::Run,3> runs{{Synthetic(1),Synthetic(2),Synthetic(4)}};
  for(auto& run:runs) {
    const auto work=Field(run.fields,"element.internal_work[0][0]");
    for(auto& s:run.samples) { s.external_work=0; s.source_work[0]=0; s.values[work]=0; s.residual=0; }
    run.last_accepted=run.samples.back(); run.external_work_at_pulse=0; run.maximum_abs_residual=0; run.residual_time=0;
    std::string error; ASSERT_TRUE(r::ValidateRun(run,true,error))<<error;
  }
  const auto empty_energy=r::Compare(runs);
  EXPECT_FALSE(empty_energy.passed); EXPECT_EQ(empty_energy.energy_normalization,0);
  runs={Synthetic(1),Synthetic(2),Synthetic(4)};
  auto& fine=runs[2]; fine.maximum_abs_residual=runs[1].maximum_abs_residual;
  EXPECT_FALSE(r::Compare(runs).passed); // Nondecreasing all-endpoint residual.
  runs={Synthetic(1),Synthetic(2),Synthetic(4)};
  runs[1].samples[7].carried_velocity_time+=r::H0;
  EXPECT_FALSE(r::Compare(runs).passed);
  runs={Synthetic(1),Synthetic(2),Synthetic(4)}; runs[2].fields[0].scale*=2;
  EXPECT_FALSE(r::Compare(runs).passed);
}
TEST(QephResponseObserver, DirectedArithmeticRejectsRoundedThresholdBoundaryAndPreservesExactZero) {
  namespace b=r::bounds;
  b::Interval difference;
  ASSERT_TRUE(b::NormalizedDifference(0,0,r::Delta,difference));
  EXPECT_EQ(difference.lower,0); EXPECT_EQ(difference.upper,0);
  // The rounded division equals the threshold, although the exact represented
  // numerator/denominator ratio is larger. The directed upper cannot pass.
  const double denominator=std::nextafter(std::nextafter(1.,0.),0.);
  const double numerator=.02*denominator;
  ASSERT_GT(static_cast<long double>(numerator)/denominator,static_cast<long double>(.02));
  EXPECT_EQ(numerator/denominator,.02);
  EXPECT_GT(b::RatioUpper(numerator,denominator),.02);
  double rounded_rhs=.75*.02+1e-8;
  const long double exact_rhs=.75L*static_cast<long double>(.02)+static_cast<long double>(1e-8);
  if(static_cast<long double>(rounded_rhs)<=exact_rhs) rounded_rhs=std::nextafter(rounded_rhs,INFINITY);
  EXPECT_GT(static_cast<long double>(rounded_rhs),exact_rhs);
  EXPECT_FALSE(b::Refines(rounded_rhs,.02,1e-8));
  EXPECT_FALSE(b::Refines(std::numeric_limits<double>::max(),.02,1e-8));
}
TEST(QephResponseObserver, StructuralValidationRejectsIncompleteSkippedMassAndPartitionChanges) {
  auto run=Synthetic(); std::string error; ASSERT_TRUE(r::ValidateRun(run,true,error));
  run.completed=false; EXPECT_FALSE(r::ValidateRun(run,true,error));
  run=Synthetic(); run.samples[3].epoch++; EXPECT_FALSE(r::ValidateRun(run,true,error));
  run=Synthetic(); run.model.mass[0]*=2; EXPECT_FALSE(r::ValidateRun(run,true,error));
  run=Synthetic(); run.samples[5].synchronous_kinetic[0]=1; EXPECT_FALSE(r::ValidateRun(run,true,error));
  run=Synthetic(); run.samples[5].source_work[2]=std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(r::ValidateRun(run,true,error));
}
} // namespace response_test
