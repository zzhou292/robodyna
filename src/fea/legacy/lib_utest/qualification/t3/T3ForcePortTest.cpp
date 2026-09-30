#include "T3ForcePortCases.h"
#include "../native/t3/T3EngineContext.h"

namespace t3_force_port_test {
TEST(T3ForcePort, AllSignedColumnsAndTinyTranslatedShapesMatchCompleteNativeHistory) {
  ForEachParityCase([](const auto& input,const auto& values,const auto& in) {
    const auto r=Reference(input); const auto base=History(r,values);
    const auto rb=Bytes(r); const auto hb=Bytes(base); const auto ib=Bytes(in);
    port::ForceTrial out;
    ASSERT_EQ(port::EvaluateForce(r,base,in,out),port::Status::kSuccess);
    Check(r,base,in,out);
    EXPECT_EQ(Bytes(r),rb); EXPECT_EQ(Bytes(base),hb); EXPECT_EQ(Bytes(in),ib);
  });
  ::testing::Test::RecordProperty("prescribed_native_configurations",ParityCases);
}
TEST(T3ForcePort, EightSignedPhysicalModesMatchIndependentStressMomentThicknessRateAndWork) {
  for(double scale:{1.,.02}) for(unsigned shape:{0u,1u}) for(unsigned mode=0;mode<8;++mode) for(double sign:{-1.,1.}) {
    SCOPED_TRACE(scale);
    SCOPED_TRACE(mode);
    const auto r=Reference(Triangle(scale,shape)); const auto base=History(r);
    auto in=Interval(r.input,1e-4); Mode(in,mode,sign*.001); port::ForceTrial out;
    ASSERT_EQ(port::EvaluateForce(r,base,in,out),port::Status::kSuccess);
    Check(r,base,in,out,true); Power(r,in,out); Balance(in,out);
    const auto truth=oracle::Independent(Native(r.input),Native(base.data()),Native(in));
    for(unsigned i=0;i<3;++i) oracle::Near(r.input.thickness*r.input.thickness*out.proposed_history.data().bending_stress[i],
      r.input.thickness*r.input.thickness*truth.values[10+i]);
  }
}
TEST(T3ForcePort, FixedResultantsMatchEighteenIndependentVirtualPowerColumns) {
  for(double scale:{1.,.02}) for(unsigned shape:{0u,1u}) {
    const auto r=Reference(Triangle(scale,shape)); auto values=Values(oracle::Seed(r.input.thickness));
    for(unsigned i=0;i<5;++i) values.stress[i]=values.material_stress[i];
    const auto base=History(r,values); const auto in=Interval(r.input,1e-4); port::ForceTrial out;
    ASSERT_EQ(port::EvaluateForce(r,base,in,out),port::Status::kSuccess);
    Check(r,base,in,out,true); Power(r,in,out); Balance(in,out);
    for(unsigned i=0;i<2;++i) EXPECT_EQ(out.proposed_history.data().internal_work[i],values.internal_work[i]);
    EXPECT_EQ(out.proposed_history.data().equivalent_strain_rate,0.);
  }
}
TEST(T3ForcePort, NonzeroHistoryLoadHoldReverseAndAliasedAcceptanceKeepOldTotalWorkOnce) {
  const auto r=Reference(Triangle(.02)); auto history=History(r,Values(oracle::Seed(r.input.thickness)));
  const auto initial=history;
  for(double rate:{.001,0.,-.001,0.}) {
    auto in=Interval(r.input,1e-4); in.base_time=history.stamp().time; in.sample_index=history.stamp().sample_index+1;
    Mode(in,0,rate); const auto saved=Bytes(history); port::ForceTrial out,repeat;
    ASSERT_EQ(port::EvaluateForce(r,history,in,out),port::Status::kSuccess);
    ASSERT_EQ(port::EvaluateForce(r,history,in,repeat),port::Status::kSuccess);
    Check(r,history,in,out,true); Exact(out,repeat); EXPECT_EQ(Bytes(history),saved);
    if(rate==0) {
      for(unsigned i=0;i<5;++i) {
        EXPECT_EQ(out.proposed_history.data().stress[i],out.proposed_history.data().material_stress[i]);
        EXPECT_EQ(out.proposed_history.data().material_stress[i],history.data().material_stress[i]);
      }
      for(unsigned i=0;i<2;++i) EXPECT_EQ(out.proposed_history.data().internal_work[i],history.data().internal_work[i]);
      EXPECT_EQ(out.proposed_history.data().equivalent_strain_rate,0.);
    }
    history=out.proposed_history;
  }
  for(unsigned i=0;i<5;++i) oracle::Near(history.data().material_stress[i],initial.data().material_stress[i]);
  EXPECT_EQ(history.stamp().sample_index,4u);
  // The accepted-by-value history may be borrowed from the output being reused.
  port::ForceTrial aliased; aliased.proposed_history=history;
  auto in=Interval(r.input,1e-4); in.base_time=history.stamp().time; in.sample_index=5; Mode(in,3,.001);
  port::ForceTrial expected; ASSERT_EQ(port::EvaluateForce(r,history,in,expected),port::Status::kSuccess);
  ASSERT_EQ(port::EvaluateForce(r,aliased.proposed_history,in,aliased),port::Status::kSuccess);
  Exact(aliased,expected);
}
TEST(T3ForcePort, ChangedCurrentAreaPreservesImmutableReferenceAndRawStrainOperationOrder) {
  const auto r=Reference(Triangle(.02)); const auto rb=Bytes(r);
  const auto base=History(r); auto in=Interval(r.input,0.000123456789);
  in.position[1].x*=1.25; in.position[2].y*=.75; in.position[2].z=.003;
  Mode(in,2,.00321); port::ForceTrial out;
  ASSERT_EQ(port::EvaluateForce(r,base,in,out),port::Status::kSuccess);
  Check(r,base,in,out,true); Power(r,in,out); Balance(in,out);
  EXPECT_NE(out.kinematics.area,r.area); EXPECT_EQ(Bytes(r),rb);
  port::Kinematics rates; ASSERT_EQ(port::EvaluatePrescribed(r,in,rates),port::Status::kSuccess);
  ExactRates(out.kinematics,rates);
  for(unsigned i=0;i<8;++i)
    EXPECT_EQ(out.proposed_history.data().strain_curvature[i],base.data().strain_curvature[i]+rates.raw_rate[i]*(in.dt/rates.area));
  EXPECT_EQ(out.diagnostics.effective_thickness,r.input.thickness);
  EXPECT_LT(out.diagnostics.unscaled_element_dt*out.diagnostics.native_sound_speed,out.kinematics.characteristic_length);
}
TEST(T3ForcePort, CyclicReversedOrdersAndWorldRotationKeepNativeFrameAndCovariantLoads) {
  const auto original=Triangle(.02); const auto rotation=kt::Rotation();
  for(unsigned order=0;order<6;++order) {
    auto input=original;
    for(unsigned n=0;n<3;++n) { const unsigned j=(order%3+(order>=3?3-n:n))%3;
      input.position[n]=original.position[j]; input.node_ids[n]=original.node_ids[j]; }
    const auto r=Reference(input); const auto values=Values(oracle::Seed(input.thickness)); const auto base=History(r,values);
    auto in=Interval(input,1e-4); Mode(in,7,.001); port::ForceTrial out;
    ASSERT_EQ(port::EvaluateForce(r,base,in,out),port::Status::kSuccess); Check(r,base,in,out,true);
    Power(r,in,out); Balance(in,out);
    auto rotated=input; auto transformed_in=in;
    for(unsigned n=0;n<3;++n) {
      rotated.position[n]=kt::Rotate(rotation,input.position[n]); transformed_in.position[n]=rotated.position[n];
      transformed_in.velocity[n]=kt::Rotate(rotation,in.velocity[n]);
      transformed_in.angular_velocity[n]=kt::Rotate(rotation,in.angular_velocity[n]);
    }
    const auto rr=Reference(rotated); const auto rh=History(rr,values); port::ForceTrial transformed;
    ASSERT_EQ(port::EvaluateForce(rr,rh,transformed_in,transformed),port::Status::kSuccess);
    Check(rr,rh,transformed_in,transformed,true);
    const double fs=input.young_modulus*input.thickness*Length(in),cs=fs*Length(in);
    for(unsigned n=0;n<3;++n) for(unsigned c=0;c<3;++c) {
      Field(Component(transformed.internal_force[n],c),Component(kt::Rotate(rotation,out.internal_force[n]),c),fs,2e-11);
      Field(Component(transformed.internal_couple[n],c),Component(kt::Rotate(rotation,out.internal_couple[n]),c),cs,2e-11);
    }
  }
}
TEST(T3ForcePort, ExactStaticTranslationAndDrillingNullKeepFiniteRigidResidualDistinct) {
  const auto r=Reference(Triangle(.02)); const auto base=History(r); auto in=Interval(r.input,1e-4);
  for(unsigned variant=0;variant<3;++variant) {
    for(unsigned n=0;n<3;++n) { in.velocity[n]=variant==1?port::Vec3{.375,-.25,.125}:port::Vec3{};
      in.angular_velocity[n]=variant==2?port::Vec3{0,0,17}:port::Vec3{}; }
    port::ForceTrial out; ASSERT_EQ(port::EvaluateForce(r,base,in,out),port::Status::kSuccess);
    Check(r,base,in,out,true);
    for(unsigned n=0;n<3;++n) for(unsigned c=0;c<3;++c) {
      EXPECT_EQ(Component(out.internal_force[n],c),0.); EXPECT_EQ(Component(out.internal_couple[n],c),0.);
    }
  }
  for(unsigned n=0;n<3;++n) { in.velocity[n]={-in.position[n].y,in.position[n].x,0}; in.angular_velocity[n]={0,0,1}; }
  port::ForceTrial rigid; ASSERT_EQ(port::EvaluateForce(r,base,in,rigid),port::Status::kSuccess);
  Check(r,base,in,rigid,true);
  EXPECT_LT(rigid.kinematics.normalized_rate[0],0.); EXPECT_NE(rigid.proposed_history.data().stress[0],0.);
}
TEST(T3ForcePort, NativeConstantsStiffnessDimensionsAndSignedSharedScatterStaySeparateFromMass) {
  namespace constant=port::detail::force_constant;
  EXPECT_EQ(constant::em20,0x1.79ca10c924223p-67); EXPECT_EQ(constant::em30,0x1.4484bfeebc29fp-100);
  EXPECT_EQ(constant::onep414,0x1.69fbe76c8b439p+0); EXPECT_EQ(constant::viscosity,0x1.eb851eb851eb8p-7);
  const double side=.02; const port::Vec3 x[4]{{0,0,0},{side,0,0},{0,side,0},{side,side,0}};
  std::array<double,12> force{},couple{},expected_force{},expected_couple{};
  force.fill(.375); couple.fill(-.25); expected_force=force; expected_couple=couple;
  std::array<double,4> stiffness{},rotary{},expected_stiffness{},expected_rotary{};
  for(const std::array<int,3> nodes:{std::array<int,3>{1,2,3},std::array<int,3>{2,4,3}}) {
    auto input=Triangle(side);
    for(unsigned n=0;n<3;++n) { input.position[n]=x[nodes[n]-1]; input.node_ids[n]=nodes[n]; }
    const auto r=Reference(input); const auto h=History(r,Values(oracle::Seed(input.thickness)));
    auto in=Interval(input,1e-4); Mode(in,2,nodes[0]==1?.001:-.001); port::ForceTrial out;
    ASSERT_EQ(port::EvaluateForce(r,h,in,out),port::Status::kSuccess); Check(r,h,in,out,true);
    const auto geometry=kt::Independent(Native(in));
    const long double e=input.young_modulus,nu=input.poisson_ratio,t=input.thickness;
    const long double g=e/(2*(1+nu)),a11=e/(1-nu*nu),sound=std::sqrt(e/input.density);
    const long double length=geometry.length*(std::sqrt(1.L+.015L*.015L)-.015L);
    const long double k=geometry.area*t*a11/(length*length),kr=k*(t*t/12+.5L*(5.L/6)*geometry.area*g/a11);
    oracle::Near(out.diagnostics.translational_stiffness,k); oracle::Near(out.diagnostics.rotational_stiffness,kr);
    oracle::Near(out.diagnostics.unscaled_element_dt,length/sound);
    oracle::Near(out.diagnostics.shear_factor,5.L/6); oracle::Near(out.diagnostics.transverse_shear_modulus,g*5/6);
    const auto values=Native(kt::MakeReference(Native(input)),out);
    std::array<double,9> f{},c{}; native::detail::PackVectors(values.internal_force,f); native::detail::PackVectors(values.internal_couple,c);
    const double local_stiffness[]{out.diagnostics.translational_stiffness,out.diagnostics.rotational_stiffness};
    { const std::lock_guard<std::mutex> lock(native::detail::NativeEngineContext());
      native::detail::t3_r3_scatter(nodes.data(),f.data(),c.data(),local_stiffness,force.data(),couple.data(),stiffness.data(),rotary.data()); }
    for(unsigned n=0;n<3;++n) {
      const unsigned node=nodes[n]-1; expected_stiffness[node]+=local_stiffness[0]; expected_rotary[node]+=local_stiffness[1];
      for(unsigned axis=0;axis<3;++axis) { expected_force[3*node+axis]-=f[3*n+axis]; expected_couple[3*node+axis]-=c[3*n+axis]; }
    }
  }
  EXPECT_EQ(force,expected_force); EXPECT_EQ(couple,expected_couple);
  EXPECT_EQ(stiffness,expected_stiffness); EXPECT_EQ(rotary,expected_rotary);
}
TEST(T3ForcePort, AllHistorySlotsIdentityLateFailuresAndAliasedRetryPreserveOutputs) {
  const auto r=Reference(Triangle(.02)); const auto base=History(r); auto in=Interval(r.input,1e-4); Mode(in,0,.001);
  port::ForceTrial out; ASSERT_EQ(port::EvaluateForce(r,base,in,out),port::Status::kSuccess);
  const auto saved=Bytes(out); const auto clean=out;
  for(unsigned fault=0;fault<InvalidForceCases;++fault) {
    SCOPED_TRACE(fault);
    auto ref=r; auto h=base; auto sample=in; Fault(fault,ref,h,sample);
    const auto rb=Bytes(ref); const auto hb=Bytes(h); const auto ib=Bytes(sample);
    EXPECT_NE(port::EvaluateForce(ref,h,sample,out),port::Status::kSuccess);
    EXPECT_EQ(Bytes(out),saved); EXPECT_EQ(Bytes(ref),rb); EXPECT_EQ(Bytes(h),hb); EXPECT_EQ(Bytes(sample),ib);
    if(fault==9||fault==10) EXPECT_EQ(port::EvaluateForce(ref,h,sample,out),port::Status::kNonfiniteResult);
  }
  auto staged=base; const auto history_bytes=Bytes(staged);
  std::array<double,26> fields{}; native::detail::PackHistory(Native(base.data()),fields);
  for(unsigned field=0;field<26;++field) {
    SCOPED_TRACE(field);
    auto invalid=fields; invalid[field]=std::numeric_limits<double>::quiet_NaN();
    EXPECT_EQ(port::PreparePrescribedHistory(r,Values(native::detail::UnpackHistory(invalid.data())),{},staged),port::Status::kInvalidInput);
    EXPECT_EQ(Bytes(staged),history_bytes);
  }
  for(unsigned field:{21u,24u,25u}) {
    auto invalid=fields; invalid[field]=-1;
    EXPECT_EQ(port::PreparePrescribedHistory(r,Values(native::detail::UnpackHistory(invalid.data())),{},staged),port::Status::kInvalidInput);
    EXPECT_EQ(Bytes(staged),history_bytes);
  }
  // A distinct coordinate sign bit still denotes a distinct bound reference.
  auto signed_zero=r; signed_zero.input.position[0].x=-0.;
  EXPECT_EQ(port::EvaluateForce(signed_zero,base,in,out),port::Status::kInvalidReference); EXPECT_EQ(Bytes(out),saved);
  ASSERT_EQ(port::EvaluateForce(r,base,in,out),port::Status::kSuccess); Exact(out,clean);
  auto next=in; next.base_time=out.proposed_history.stamp().time; next.sample_index=2; Mode(next,0,1e6);
  const auto alias_bytes=Bytes(out);
  EXPECT_EQ(port::EvaluateForce(r,out.proposed_history,next,out),port::Status::kNonfiniteResult);
  EXPECT_EQ(Bytes(out),alias_bytes);
  next=in; next.base_time=out.proposed_history.stamp().time; next.sample_index=2;
  const auto h=out.proposed_history; port::ForceTrial expected;
  ASSERT_EQ(port::EvaluateForce(r,h,next,expected),port::Status::kSuccess);
  ASSERT_EQ(port::EvaluateForce(r,out.proposed_history,next,out),port::Status::kSuccess); Exact(out,expected);
}
} // namespace t3_force_port_test
