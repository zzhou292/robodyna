#include "QephForceFixture.h"
#include "lib_utest/qualification/native/qeph/NativeQephBridge.h"

extern "C" void qeph_q2_scatter(const double*,const double*,const int*,double*);

namespace {
using namespace qeph_force_port_test;

TEST(QephForcePort, ActualNativeAllFieldsForGeometryRateAndFullHistoryVariants) {
  for(unsigned fixture=0;fixture<kCases;++fixture) for(unsigned shift=0;shift<4;++shift)
    for(bool transformed:{false,true}) {
      SCOPED_TRACE(fixture);
      SCOPED_TRACE(shift);
      SCOPED_TRACE(transformed);
      const auto raw=Case(fixture),input=qeph_startup_test::Reparameterize(raw,shift,transformed);
      port::ReferenceData reference; native::Reference native_ref;
      ASSERT_EQ(port::InitializeReference(input,reference),port::Status::kSuccess);
      ASSERT_EQ(native::Initialize(NativeInput(input),native_ref),native::Status::kSuccess);
      for(unsigned pattern=0;pattern<3;++pattern) for(bool seeded:{false,true}) {
        SCOPED_TRACE(pattern);
        SCOPED_TRACE(seeded);
        const auto p=qeph_kinematics_test::Reparameterize(Pattern(raw,pattern),shift,transformed);
        const auto values=Seed(input,seeded); port::History history; native::History native_history;
        ASSERT_EQ(port::PreparePrescribedHistory(reference,values,{p.base_time,p.sample_index-1},history),port::Status::kSuccess);
        ASSERT_EQ(native::PreparePrescribedHistory(native_ref,NativeValues(values),
            {p.base_time,p.sample_index-1},native_history),native::Status::kSuccess);
        const auto base_bytes=Bytes(history);
        const auto reference_bytes=Bytes(reference);
        const auto interval_bytes=Bytes(p);
        port::ForceTrial result; native::ForceTrial truth;
        ASSERT_EQ(port::EvaluateForce(reference,history,p,result),port::Status::kSuccess);
        ASSERT_EQ(native::EvaluateForce(native_ref,native_history,NativeInterval(p),truth),native::Status::kSuccess);
        ForceAgreement(result,truth,input,p);
        port::Kinematics standalone;
        ASSERT_EQ(port::EvaluatePrescribed(reference,p,standalone),port::Status::kSuccess);
        qeph_kinematics_test::Agreement(result.kinematics,standalone,p,true,0.);
        EXPECT_EQ(Bytes(history),base_bytes); EXPECT_EQ(Bytes(reference),reference_bytes); EXPECT_EQ(Bytes(p),interval_bytes);
        EXPECT_TRUE(result.proposed_history.matches_reference(reference));
      }
    }
  RecordProperty("native_parity_configurations",432);
  RecordProperty("dynamics_qualified","false");
}

TEST(QephForcePort, IndependentStressAndPhysicalMomentModesAtSourceScale) {
  for(unsigned fixture:{0u,2u,4u}) {
    const auto input=Case(fixture); port::ReferenceData reference; port::History history;
    ASSERT_EQ(port::InitializeReference(input,reference),port::Status::kSuccess);
    ASSERT_EQ(port::InitializeHistory(reference,{},history),port::Status::kSuccess);
    for(unsigned mode=0;mode<8;++mode) {
      SCOPED_TRACE(fixture);
      SCOPED_TRACE(mode);
      auto p=Next(input,history); const double rate=mode<5?.02:.02/Scale(input);
      ApplyMode(p,mode,rate); port::ForceTrial result;
      ASSERT_EQ(port::EvaluateForce(reference,history,p,result),port::Status::kSuccess);
      ModeStressTruth(result,input,mode,rate,p.dt); Balance(result,p);
    }
  }
}

TEST(QephForcePort, LoadHoldReversalRetainsMaterialStressAndNativePriorTotalWork) {
  const auto input=Case(0); port::ReferenceData reference; port::History history;
  ASSERT_EQ(port::InitializeReference(input,reference),port::Status::kSuccess);
  ASSERT_EQ(port::InitializeHistory(reference,{},history),port::Status::kSuccess);
  constexpr double dt=.001;
  const double a=input.young_modulus/(1-input.poisson_ratio*input.poisson_ratio),b=input.poisson_ratio*a;
  // Independent physical decimal constant, evaluated in wider arithmetic;
  // native expression rounding is covered by the unchanged Q2 budget.
  const long double viscosity=1.414L*.015L*input.density*
      std::sqrt(static_cast<long double>(input.young_modulus)/input.density)*std::sqrt(2.L);
  double material[2]{},previous_total=0,work=0,thickness=input.thickness,strain=0;
  for(double rate:{.02,.02,0.,-.02,-.02}) {
    auto p=Next(input,history,dt); ApplyMode(p,0,rate); port::ForceTrial result;
    ASSERT_EQ(port::EvaluateForce(reference,history,p,result),port::Status::kSuccess);
    const double increment=rate*dt;
    material[0]+=a*increment; material[1]+=b*increment;
    const double total=material[0]+static_cast<double>(viscosity*rate);
    work+=.5*(input.thickness*2.)*(previous_total+total)*increment;
    thickness*=1-input.poisson_ratio*increment/(1-input.poisson_ratio); strain+=increment;
    const auto& h=result.proposed_history.data();
    Independent(h.material_stress[0],material[0]); Independent(h.material_stress[1],material[1]);
    Independent(h.stress[0],total); Independent(h.stress[1],material[1]+static_cast<double>(.5L*viscosity*rate));
    Independent(h.internal_work[0],work,kWorkAbsolute); Independent(h.internal_work[1],0.,kWorkAbsolute);
    Independent(h.thickness,thickness); Independent(h.strain_curvature[0],strain);
    for(double value:h.stabilization) EXPECT_EQ(value,0.);
    previous_total=total; history=result.proposed_history;
  }
  Independent(history.data().material_stress[0],0.);
  EXPECT_GT(std::abs(history.data().stress[0]),1.);
}

TEST(QephForcePort, AllTwelveStabilizationValuesAndViscousWorkPersistAcrossHold) {
  const auto input=Case(1); port::ReferenceData reference; port::History zero,seeded;
  ASSERT_EQ(port::InitializeReference(input,reference),port::Status::kSuccess);
  ASSERT_EQ(port::InitializeHistory(reference,{},zero),port::Status::kSuccess);
  auto seed=zero.data();
  for(unsigned i=0;i<12;++i) seed.stabilization[i]=.03*(i+1)*(i%2?-1.:1.);
  seed.internal_work[0]=.012; seed.internal_work[1]=.023; seed.hourglass_viscous_work=.034;
  ASSERT_EQ(port::PreparePrescribedHistory(reference,seed,{},seeded),port::Status::kSuccess);
  auto p=Next(input,zero,.001);
  for(unsigned n=0;n<4;++n) {
    const double s=n%2?-1.:1.; p.velocity_midpoint[n]={s*.003,s*.004,s*.002};
    p.omega_midpoint[n]={s*.005,s*.006,0};
  }
  port::ForceTrial from_zero,from_seed;
  ASSERT_EQ(port::EvaluateForce(reference,zero,p,from_zero),port::Status::kSuccess);
  ASSERT_EQ(port::EvaluateForce(reference,seeded,p,from_seed),port::Status::kSuccess);
  double change=0;
  for(unsigned i=0;i<12;++i) {
    const double delta=from_zero.proposed_history.data().stabilization[i]; change+=std::abs(delta);
    Independent(from_seed.proposed_history.data().stabilization[i],seed.stabilization[i]+delta);
  }
  EXPECT_GT(change,0.); EXPECT_GT(from_zero.proposed_history.data().hourglass_viscous_work,0.);
  Independent(from_seed.diagnostics.hourglass_viscous_work_increment,
              from_zero.diagnostics.hourglass_viscous_work_increment,1e-17);
  const auto held_interval=Next(input,from_seed.proposed_history,.001); port::ForceTrial held;
  ASSERT_EQ(port::EvaluateForce(reference,from_seed.proposed_history,held_interval,held),port::Status::kSuccess);
  auto expected=from_seed.proposed_history.data();
  for(unsigned i=0;i<5;++i) expected.stress[i]=expected.material_stress[i];
  HistoryAgreement(held.proposed_history.data(),expected,input,LengthScale(held_interval),0.);
  Balance(from_seed,p); Balance(held,held_interval);
}

TEST(QephForcePort, StaticZeroDrillingAndRigidNativeCharacterizationRemainDistinct) {
  for(unsigned fixture:{0u,1u,2u,3u}) {
    const auto input=Case(fixture); port::ReferenceData reference; port::History history;
    ASSERT_EQ(port::InitializeReference(input,reference),port::Status::kSuccess);
    ASSERT_EQ(port::InitializeHistory(reference,{},history),port::Status::kSuccess);
    auto p=Next(input,history,.001); port::ForceTrial zero;
    ASSERT_EQ(port::EvaluateForce(reference,history,p,zero),port::Status::kSuccess);
    for(unsigned n=0;n<4;++n) {
      EXPECT_EQ(Length(zero.internal_force[n]),0.); EXPECT_EQ(Length(zero.internal_couple[n]),0.);
    }
    if(fixture%2==0) {
      for(unsigned n=0;n<4;++n) p.omega_midpoint[n].z=(n+1)*(n%2?-1.:1.);
      port::ForceTrial drilling;
      ASSERT_EQ(port::EvaluateForce(reference,history,p,drilling),port::Status::kSuccess);
      ForceAgreement(drilling,zero,input,p,0.);
    }
    p=Rigid(input,Unit({1,2,3}),.02); p.base_time=0; p.sample_index=1;
    port::ForceTrial rigid;
    ASSERT_EQ(port::EvaluateForce(reference,history,p,rigid),port::Status::kSuccess);
    Balance(rigid,p);
    double norm=0; for(const auto f:rigid.internal_force) norm+=Length(f);
    EXPECT_GT(norm,0.); // Qualified source phase has real shear residual; no false zero.
  }
  RecordProperty("source_dynamics_qualified","false");
}

TEST(QephForcePort, CommonWorldRotationTransformsForceAndCoupleWithoutChangingHistory) {
  for(unsigned fixture=0;fixture<kCases;++fixture) {
    const auto a=Case(fixture),b=qeph_startup_test::Reparameterize(a,0,true);
    port::ReferenceData ar,br; port::History ah,bh;
    ASSERT_EQ(port::InitializeReference(a,ar),port::Status::kSuccess);
    ASSERT_EQ(port::InitializeReference(b,br),port::Status::kSuccess);
    const auto values=Seed(a,true);
    ASSERT_EQ(port::PreparePrescribedHistory(ar,values,{.125,72},ah),port::Status::kSuccess);
    ASSERT_EQ(port::PreparePrescribedHistory(br,values,{.125,72},bh),port::Status::kSuccess);
    const auto ap=Pattern(a,1),bp=qeph_kinematics_test::Reparameterize(ap,0,true);
    port::ForceTrial at,bt;
    ASSERT_EQ(port::EvaluateForce(ar,ah,ap,at),port::Status::kSuccess);
    ASSERT_EQ(port::EvaluateForce(br,bh,bp,bt),port::Status::kSuccess);
    const double l=LengthScale(ap),e=a.young_modulus,t=a.thickness;
    HistoryAgreement(bt.proposed_history.data(),at.proposed_history.data(),a,l,kCovariance);
    for(unsigned n=0;n<4;++n) {
      Field(bt.internal_force[n],Rotate(CommonRotation(),at.internal_force[n]),e*t*l,kCovariance,"covariant_force",n);
      Field(bt.internal_couple[n],Rotate(CommonRotation(),at.internal_couple[n]),e*t*l*l,kCovariance,"covariant_couple",n);
    }
  }
}

TEST(QephForcePort, CoincidentCurrentCornerRetainsThePreparedReferenceAndFiniteForceContract) {
  const auto input=Case(0);port::ReferenceData reference;port::History history;
  ASSERT_EQ(port::InitializeReference(input,reference),port::Status::kSuccess);
  ASSERT_EQ(port::PreparePrescribedHistory(reference,Seed(input,true),{.125,72},history),port::Status::kSuccess);
  const auto old_history=Bytes(history),old_reference=Bytes(reference);
  auto interval=Next(input,history,.001);interval.position_endpoint[3]=interval.position_endpoint[0];
  port::ForceTrial result;
  ASSERT_EQ(port::EvaluateForce(reference,history,interval,result),port::Status::kSuccess);
  EXPECT_GT(result.kinematics.area,0.);
  EXPECT_TRUE(port::detail::ValidForceDiagnostics(result.diagnostics));
  EXPECT_EQ(Bytes(history),old_history);EXPECT_EQ(Bytes(reference),old_reference);
}

TEST(QephForcePort, HistoryBindingMalformedStampsAndLateFailurePreserveEveryByteThenRetry) {
  const auto input=Case(0); port::ReferenceData reference; port::History h;
  ASSERT_EQ(port::InitializeReference(input,reference),port::Status::kSuccess);
  ASSERT_EQ(port::PreparePrescribedHistory(reference,Seed(input,true),{.125,72},h),port::Status::kSuccess);
  const auto valid=Next(input,h,.001); port::ForceTrial output;
  ASSERT_EQ(port::EvaluateForce(reference,h,valid,output),port::Status::kSuccess);
  const auto saved=Bytes(output);
  const auto base_bytes=Bytes(h);
  const auto ref_bytes=Bytes(reference);
  for(unsigned kind=0;kind<6;++kind) {
    auto bad=valid;
    if(kind==0) bad.base_time=std::nextafter(bad.base_time,1.);
    if(kind==1) ++bad.sample_index;
    if(kind==2) bad.dt=std::numeric_limits<double>::denorm_min();
    if(kind==3) for(auto& x:bad.position_endpoint)x=bad.position_endpoint[0]; // True zero-area collapse.
    if(kind==4) bad.omega_midpoint[3].z=std::numeric_limits<double>::infinity();
    if(kind==5) ApplyMode(bad,0,4./bad.dt); // Late finite-input THKN rejection.
    const auto bad_bytes=Bytes(bad);
    EXPECT_NE(port::EvaluateForce(reference,h,bad,output),port::Status::kSuccess);
    EXPECT_EQ(Bytes(output),saved); EXPECT_EQ(Bytes(h),base_bytes);
    EXPECT_EQ(Bytes(reference),ref_bytes); EXPECT_EQ(Bytes(bad),bad_bytes);
  }
  port::History maximum;
  ASSERT_EQ(port::InitializeHistory(reference,{.125,UINT64_MAX},maximum),port::Status::kSuccess);
  auto overflow=valid; overflow.sample_index=0;
  EXPECT_EQ(port::EvaluateForce(reference,maximum,overflow,output),port::Status::kInvalidInput);
  EXPECT_EQ(Bytes(output),saved);
  for(unsigned field=0;field<6;++field) {
    auto other=input;
    if(field==0) ++other.node_ids[3];
    if(field==1) other.position[3].x=std::nextafter(other.position[3].x,0.);
    if(field==2) other.density=std::nextafter(other.density,20.);
    if(field==3) other.young_modulus=std::nextafter(other.young_modulus,3e6);
    if(field==4) other.poisson_ratio=std::nextafter(other.poisson_ratio,.4);
    if(field==5) other.thickness=std::nextafter(other.thickness,.2);
    port::ReferenceData foreign;
    ASSERT_EQ(port::InitializeReference(other,foreign),port::Status::kSuccess);
    EXPECT_FALSE(h.matches_reference(foreign));
    EXPECT_EQ(port::EvaluateForce(foreign,h,valid,output),port::Status::kInvalidReference);
    EXPECT_EQ(Bytes(output),saved);
  }
  for(unsigned kind=0;kind<4;++kind) {
    auto invalid=h.data();
    if(kind==0) invalid.stabilization[11]=std::numeric_limits<double>::quiet_NaN();
    if(kind==1) invalid.hourglass_viscous_work=std::numeric_limits<double>::infinity();
    if(kind==2) invalid.thickness=0;
    if(kind==3) invalid.active=.5;
    EXPECT_EQ(port::PreparePrescribedHistory(reference,invalid,h.stamp(),h),port::Status::kInvalidInput);
    EXPECT_EQ(Bytes(h),base_bytes);
  }
  port::ForceTrial clean;
  ASSERT_EQ(port::EvaluateForce(reference,h,valid,clean),port::Status::kSuccess);
  ASSERT_EQ(port::EvaluateForce(reference,h,valid,output),port::Status::kSuccess);
  ForceAgreement(output,clean,input,valid,0.);
}

TEST(ShellElasticLaw1, SharedPurePointHasIndependentConstitutiveValuesAndStagedFailures) {
  using namespace tl::material;
  ShellElasticLaw1Coefficients c;
  ASSERT_TRUE(PrepareShellElasticLaw1(200e9,.3,7890,c));
  Independent(c.g,200e9/(2*1.3)); Independent(c.a11,200e9/.91);
  ShellElasticLaw1Stress base,out; const double dx[8]{1e-6,2e-6,3e-6,4e-6,5e-6,6e-3,7e-3,8e-3};
  ASSERT_TRUE(UpdateShellElasticLaw1(c,c.g*5./6.,.001648/12.,dx,base,out));
  Independent(out.stress[0],c.a11*dx[0]+c.a12*dx[1]);
  const auto saved=Bytes(out);
  const auto coeff=Bytes(c);
  EXPECT_FALSE(PrepareShellElasticLaw1(200e9,-.1,7890,c)); EXPECT_EQ(Bytes(c),coeff);
  EXPECT_FALSE(UpdateShellElasticLaw1(c,c.g,0.,dx,base,out)); EXPECT_EQ(Bytes(out),saved);
  auto bad=base; bad.bending_stress[2]=std::numeric_limits<double>::infinity();
  EXPECT_FALSE(UpdateShellElasticLaw1(c,c.g,.001648/12.,dx,bad,out)); EXPECT_EQ(Bytes(out),saved);
  auto big=c; big.a11=std::numeric_limits<double>::max();
  EXPECT_FALSE(UpdateShellElasticLaw1(big,c.g,2.,dx,base,out)); EXPECT_EQ(Bytes(out),saved);
}

TEST(QephForcePort, PositiveInternalCacheMatchesNativeSignedScatterAndStiffnessUnits) {
  const auto input=Case(0); port::ReferenceData reference; port::History h;
  ASSERT_EQ(port::InitializeReference(input,reference),port::Status::kSuccess);
  ASSERT_EQ(port::InitializeHistory(reference,{},h),port::Status::kSuccess);
  auto p=Next(input,h); ApplyMode(p,2,1e-4); port::ForceTrial trial;
  ASSERT_EQ(port::EvaluateForce(reference,h,p,trial),port::Status::kSuccess);
  const auto& d=trial.diagnostics;
  const long double speed=std::sqrt(static_cast<long double>(input.young_modulus)/input.density);
  const long double length=trial.kinematics.characteristic_length*(std::sqrt(1+.015L*.015L)-.015L);
  const long double a=static_cast<long double>(input.young_modulus)/(1-.25L*.25L);
  const long double k=.5L*(input.thickness*2)*a/(length*length);
  Independent(d.native_sound_speed,static_cast<double>(speed));
  Independent(d.translational_stiffness,static_cast<double>(k));
  Independent(d.rotational_stiffness,static_cast<double>(k*(input.thickness*input.thickness+2)/12));
  Independent(d.unscaled_element_dt,static_cast<double>(length/speed));
  std::array<double,24> values{}; std::array<double,32> rhs{};
  for(unsigned n=0;n<4;++n) for(unsigned axis=0;axis<3;++axis) {
    values[3*n+axis]=port::detail::Component(trial.internal_force[n],axis);
    values[12+3*n+axis]=port::detail::Component(trial.internal_couple[n],axis);
  }
  const double coefficients[]{d.translational_stiffness,d.rotational_stiffness,
      trial.kinematics.nodal_factors[0],trial.kinematics.nodal_factors[1]};
  const int connectivity[]{3,1,4,2};
  {
    const std::lock_guard<std::mutex> lock(native::detail::NativeContext());
    qeph_q2_scatter(values.data(),coefficients,connectivity,rhs.data());
  }
  for(unsigned n=0;n<4;++n) {
    const unsigned node=connectivity[n]-1;
    for(unsigned axis=0;axis<3;++axis) {
      EXPECT_EQ(rhs[3*node+axis],-values[3*n+axis]);
      EXPECT_EQ(rhs[12+3*node+axis],-values[12+3*n+axis]);
    }
    EXPECT_EQ(rhs[24+node],coefficients[0]*coefficients[2+n%2]);
    EXPECT_EQ(rhs[28+node],coefficients[1]*coefficients[2+n%2]);
  }
}
} // namespace
