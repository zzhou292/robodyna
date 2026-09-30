#include "T3ForceTestOracle.h"
#include "T3EngineContext.h"
#include <limits>

namespace {
namespace native=tl::qualification::t3;
namespace kt=native::kinematic_test;
namespace oracle=native::force_test;
using native::Status;

void CheckUnchangedKinematics(const native::Reference& reference,const native::PrescribedInterval& in,
                             const native::Kinematics& actual) {
  native::Kinematics prior;
  ASSERT_EQ(native::EvaluatePrescribed(reference,in,prior),Status::kSuccess);
  EXPECT_EQ(prior.raw_rate,actual.raw_rate); EXPECT_EQ(prior.normalized_rate,actual.normalized_rate);
  EXPECT_EQ(prior.derivative,actual.derivative); EXPECT_EQ(prior.corrected_velocity_difference,actual.corrected_velocity_difference);
  EXPECT_EQ(prior.area,actual.area); EXPECT_EQ(prior.characteristic_length,actual.characteristic_length);
  for(unsigned k=0;k<9;++k) EXPECT_EQ(prior.frame.v[k],actual.frame.v[k]);
}
void Balance(const native::ReferenceInput& r,const native::ForceTrial& out) {
  kt::Wide f{},m{};
  for(unsigned n=0;n<3;++n) {
    f=kt::Add(f,kt::Widen(out.internal_force[n]));
    m=kt::Add(m,kt::Add(kt::Cross(native::test::Difference(r.position[n],r.position[0]),
          kt::Widen(out.internal_force[n])),kt::Widen(out.internal_couple[n])));
  }
  for(unsigned k=0;k<3;++k) { oracle::Near(double(f[k]),0); oracle::Near(double(m[k]),0); }
}

TEST(T3ForceCheck, EightPhysicalModesRetainIndependentStressMomentsThicknessAndRate) {
  for(double length:{1.,.02}) for(double skew:{0.,.375}) for(unsigned mode=0;mode<8;++mode) {
    SCOPED_TRACE(length);
    SCOPED_TRACE(skew);
    SCOPED_TRACE(mode);
    auto r=kt::Triangle(length); r.position[2].x=skew*length;
    const auto reference=kt::MakeReference(r); const auto base=oracle::MakeHistory(reference);
    auto in=kt::Interval(r,1e-4); oracle::Mode(in,mode,1e-3);
    native::ForceTrial value;
    ASSERT_EQ(native::EvaluateForce(reference,base,in,value),Status::kSuccess);
    oracle::Check(reference,base.data(),in,value); CheckUnchangedKinematics(reference,in,value.kinematics);
    oracle::CheckFixedResultantPower(r,value); Balance(r,value);
    // Stress-like MOM converts back to physical E*t^3/12 bending resultants.
    const auto expected=oracle::Independent(r,base.data(),in);
    for(unsigned k=0;k<3;++k)
      oracle::Near(r.thickness*r.thickness*value.proposed_history.data().bending_stress[k],
                   r.thickness*r.thickness*expected.values[10+k]);
  }
}

TEST(T3ForceCheck, StaticSeededResultantsMatchAllEighteenVirtualPowerColumns) {
  for(double length:{1.,.02}) {
    const auto r=kt::Triangle(length); const auto reference=kt::MakeReference(r);
    auto h=oracle::Seed(r.thickness); h.stress=h.material_stress;
    const auto base=oracle::MakeHistory(reference,h);
    const auto in=kt::Interval(r,1e-4); native::ForceTrial out;
    ASSERT_EQ(native::EvaluateForce(reference,base,in,out),Status::kSuccess);
    oracle::Check(reference,h,in,out); oracle::CheckFixedResultantPower(r,out); Balance(r,out);
    EXPECT_EQ(out.proposed_history.data().material_stress,h.material_stress);
    EXPECT_EQ(out.proposed_history.data().internal_work,h.internal_work);
    EXPECT_EQ(out.proposed_history.data().equivalent_strain_rate,0);
  }
}

TEST(T3ForceCheck, LoadHoldReversalPreservesMaterialHistoryAndPriorTotalWork) {
  const auto r=kt::Triangle(.02); const auto reference=kt::MakeReference(r);
  auto history=oracle::MakeHistory(reference,oracle::Seed(r.thickness));
  const auto initial=history;
  for(double rate:{.001,0.,-.001,0.}) {
    auto in=kt::Interval(r,1e-4); in.base_time=history.stamp().time;
    in.sample_index=history.stamp().sample_index+1; oracle::Mode(in,0,rate);
    const auto before=kt::Bytes(history); native::ForceTrial out,repeat;
    ASSERT_EQ(native::EvaluateForce(reference,history,in,out),Status::kSuccess);
    ASSERT_EQ(native::EvaluateForce(reference,history,in,repeat),Status::kSuccess);
    oracle::Check(reference,history.data(),in,out);
    EXPECT_EQ(kt::Bytes(history),before);
    std::array<double,26> a{},b{}; native::detail::PackHistory(out.proposed_history.data(),a);
    native::detail::PackHistory(repeat.proposed_history.data(),b); EXPECT_EQ(a,b);
    if(rate==0) {
      EXPECT_EQ(out.proposed_history.data().stress,out.proposed_history.data().material_stress);
      EXPECT_EQ(out.proposed_history.data().material_stress,history.data().material_stress);
      EXPECT_EQ(out.proposed_history.data().equivalent_strain_rate,0);
      EXPECT_EQ(out.proposed_history.data().internal_work,history.data().internal_work);
    }
    history=out.proposed_history;
  }
  for(unsigned k=0;k<5;++k) oracle::Near(history.data().material_stress[k],initial.data().material_stress[k]);
  EXPECT_EQ(history.data().bending_stress,initial.data().bending_stress);
  EXPECT_EQ(history.stamp().sample_index,4u);
}

TEST(T3ForceCheck, WorldCovarianceCyclicOrderingAndCurrentFrameRemainIndependent) {
  const auto original=kt::Triangle(.02); const auto q=kt::Rotation();
  for(unsigned shift=0;shift<3;++shift) {
    auto r=original;
    for(unsigned n=0;n<3;++n) { r.position[n]=original.position[(n+shift)%3]; r.node_ids[n]=original.node_ids[(n+shift)%3]; }
    const auto reference=kt::MakeReference(r); const auto h=oracle::Seed(r.thickness);
    const auto base=oracle::MakeHistory(reference,h); auto in=kt::Interval(r,1e-4); oracle::Mode(in,7,1e-3);
    native::ForceTrial out; ASSERT_EQ(native::EvaluateForce(reference,base,in,out),Status::kSuccess);
    oracle::Check(reference,h,in,out); oracle::CheckFixedResultantPower(r,out);
    auto rotated=r; auto rotated_in=in;
    for(unsigned n=0;n<3;++n) {
      rotated.position[n]=kt::Rotate(q,r.position[n]); rotated_in.position[n]=rotated.position[n];
      rotated_in.velocity[n]=kt::Rotate(q,in.velocity[n]);
      rotated_in.angular_velocity[n]=kt::Rotate(q,in.angular_velocity[n]);
    }
    const auto rotated_ref=kt::MakeReference(rotated); const auto rotated_base=oracle::MakeHistory(rotated_ref,h);
    native::ForceTrial transformed;
    ASSERT_EQ(native::EvaluateForce(rotated_ref,rotated_base,rotated_in,transformed),Status::kSuccess);
    oracle::Check(rotated_ref,h,rotated_in,transformed);
    for(unsigned n=0;n<3;++n) {
      const auto f=kt::Rotate(q,out.internal_force[n]),c=kt::Rotate(q,out.internal_couple[n]);
      const auto L=native::test::Independent(r).length; const auto force_scale=r.young_modulus*r.thickness*L;
      for(unsigned k=0;k<3;++k) {
        const auto a=kt::Widen(transformed.internal_force[n]),b=kt::Widen(f);
        EXPECT_LE(std::abs(a[k]-b[k]),2e-11L*(force_scale+std::abs(b[k])));
        const auto x=kt::Widen(transformed.internal_couple[n]),y=kt::Widen(c);
        EXPECT_LE(std::abs(x[k]-y[k]),2e-11L*(force_scale*L+std::abs(y[k])));
      }
    }
  }
}

TEST(T3ForceCheck, ZeroDrillingAndFiniteRigidRatesHaveDistinctNativeMeaning) {
  const auto r=kt::Triangle(.02); const auto reference=kt::MakeReference(r); const auto base=oracle::MakeHistory(reference);
  auto in=kt::Interval(r,1e-4); native::ForceTrial zero;
  ASSERT_EQ(native::EvaluateForce(reference,base,in,zero),Status::kSuccess);
  for(const auto& f:zero.internal_force) EXPECT_EQ((std::array<double,3>{f.x,f.y,f.z}),(std::array<double,3>{}));
  in.angular_velocity.fill({0,0,17}); native::ForceTrial drilling;
  ASSERT_EQ(native::EvaluateForce(reference,base,in,drilling),Status::kSuccess);
  for(const auto& f:drilling.internal_force) EXPECT_EQ((std::array<double,3>{f.x,f.y,f.z}),(std::array<double,3>{}));
  for(const auto& c:drilling.internal_couple) EXPECT_EQ((std::array<double,3>{c.x,c.y,c.z}),(std::array<double,3>{}));
  for(unsigned n=0;n<3;++n) {
    in.velocity[n]={-r.position[n].y,r.position[n].x,0}; in.angular_velocity[n]={0,0,1};
  }
  native::ForceTrial rigid;
  ASSERT_EQ(native::EvaluateForce(reference,base,in,rigid),Status::kSuccess);
  oracle::Check(reference,base.data(),in,rigid);
  EXPECT_LT(rigid.kinematics.normalized_rate[0],0); // Native finite-h correction, not exact rigid null.
  EXPECT_NE(rigid.proposed_history.data().stress[0],0);
}

TEST(T3ForceCheck, NativeStiffnessUnitsAndSignedSharedNodeScatterAreIndependent) {
  const auto r=kt::Triangle(.02); const auto reference=kt::MakeReference(r);
  const auto base=oracle::MakeHistory(reference,oracle::Seed(r.thickness));
  auto in=kt::Interval(r,1e-4); oracle::Mode(in,2,.001); native::ForceTrial out;
  ASSERT_EQ(native::EvaluateForce(reference,base,in,out),Status::kSuccess);
  const auto geometry=kt::Independent(in);
  const long double E=r.young_modulus,nu=r.poisson_ratio,t=r.thickness;
  const long double c=std::sqrt(E/r.density),g=E/(2*(1+nu)),a11=E/(1-nu*nu);
  const long double l=geometry.length*(std::sqrt(1.L+.015L*.015L)-.015L);
  const long double k=geometry.area*t*a11/(l*l),kr=k*(t*t/12+.5L*(5.L/6)*geometry.area*g/a11);
  oracle::Near(out.diagnostics.native_sound_speed,c); oracle::Near(out.diagnostics.shear_factor,5.L/6);
  oracle::Near(out.diagnostics.transverse_shear_modulus,g*5/6);
  oracle::Near(out.diagnostics.translational_stiffness,k); oracle::Near(out.diagnostics.rotational_stiffness,kr);
  oracle::Near(out.diagnostics.unscaled_element_dt,l/c);
  std::array<double,9> force{},couple{}; native::detail::PackVectors(out.internal_force,force);
  native::detail::PackVectors(out.internal_couple,couple);
  const std::array<double,2> stiffness{out.diagnostics.translational_stiffness,out.diagnostics.rotational_stiffness};
  std::array<double,12> f{},m{},expected_f{},expected_m{}; f.fill(.375); m.fill(-.25); expected_f=f; expected_m=m;
  std::array<double,4> kn{},km{},expected_kn{},expected_km{};
  for(const std::array<int,3> nodes:{std::array<int,3>{1,2,3},std::array<int,3>{2,4,3}}) {
    const std::lock_guard<std::mutex> lock(native::detail::NativeEngineContext());
    native::detail::t3_r3_scatter(nodes.data(),force.data(),couple.data(),stiffness.data(),f.data(),m.data(),kn.data(),km.data());
    for(unsigned n=0;n<3;++n) {
      const unsigned node=unsigned(nodes[n]-1); expected_kn[node]+=stiffness[0]; expected_km[node]+=stiffness[1];
      for(unsigned axis=0;axis<3;++axis) {
        expected_f[3*node+axis]-=force[3*n+axis]; expected_m[3*node+axis]-=couple[3*n+axis];
      }
    }
  }
  EXPECT_EQ(f,expected_f); EXPECT_EQ(m,expected_m); EXPECT_EQ(kn,expected_kn); EXPECT_EQ(km,expected_km);
}

TEST(T3ForceCheck, ForeignHistoryInvalidFieldsAndStampsPreserveEveryByte) {
  const auto r=kt::Triangle(.02); const auto reference=kt::MakeReference(r); const auto base=oracle::MakeHistory(reference);
  auto in=kt::Interval(r,1e-4); native::ForceTrial out;
  ASSERT_EQ(native::EvaluateForce(reference,base,in,out),Status::kSuccess); const auto saved=kt::Bytes(out);
  for(unsigned fault=0;fault<8;++fault) {
    auto sample=in; auto h=base; auto ref=reference;
    if(fault==0)ref=native::Reference{};
    if(fault==1) { auto foreign=r; ++foreign.node_ids[0]; ref=kt::MakeReference(foreign); }
    if(fault==2)sample.base_time=.25;
    if(fault==3)++sample.sample_index;
    if(fault==4)sample.dt=0;
    if(fault==5)sample.velocity[1].x=std::numeric_limits<double>::quiet_NaN();
    if(fault==6)h=native::History{};
    if(fault==7)h=oracle::MakeHistory(reference,base.data(),{0,UINT64_MAX});
    EXPECT_NE(native::EvaluateForce(ref,h,sample,out),Status::kSuccess); EXPECT_EQ(kt::Bytes(out),saved);
  }
  auto staged=base; const auto prior=kt::Bytes(staged);
  for(unsigned field=0;field<26;++field) {
    std::array<double,26> values{}; native::detail::PackHistory(base.data(),values);
    values[field]=std::numeric_limits<double>::quiet_NaN();
    EXPECT_EQ(native::PreparePrescribedHistory(reference,native::detail::UnpackHistory(values.data()),{},staged),Status::kInvalidInput);
    EXPECT_EQ(kt::Bytes(staged),prior);
  }
  for(unsigned field:{21u,24u,25u}) {
    std::array<double,26> values{}; native::detail::PackHistory(base.data(),values); values[field]=-1;
    EXPECT_EQ(native::PreparePrescribedHistory(reference,native::detail::UnpackHistory(values.data()),{},staged),Status::kInvalidInput);
    EXPECT_EQ(kt::Bytes(staged),prior);
  }
}

TEST(T3ForceCheck, LateThicknessAndForceOverflowPreserveBaseThenCleanRetry) {
  const auto r=kt::Triangle(.02); const auto reference=kt::MakeReference(r); const auto base=oracle::MakeHistory(reference);
  auto good=kt::Interval(r,1e-4); oracle::Mode(good,0,.001);
  native::ForceTrial output; ASSERT_EQ(native::EvaluateForce(reference,base,good,output),Status::kSuccess);
  const auto saved=kt::Bytes(output);
  const auto base_bytes=kt::Bytes(base);
  const auto reference_bytes=kt::Bytes(reference);
  auto thinning=good; oracle::Mode(thinning,0,1e6);
  EXPECT_EQ(native::EvaluateForce(reference,base,thinning,output),Status::kNonfiniteResult);
  EXPECT_EQ(kt::Bytes(output),saved);
  auto huge=base.data(); huge.material_stress[0]=std::numeric_limits<double>::max();
  huge.material_stress[1]=std::numeric_limits<double>::max();
  huge.material_stress[2]=std::numeric_limits<double>::max();
  const auto huge_base=oracle::MakeHistory(reference,huge); const auto huge_bytes=kt::Bytes(huge_base);
  auto large=good; large.position=kt::Triangle(500).position; large.velocity={};
  EXPECT_EQ(native::EvaluateForce(reference,huge_base,large,output),Status::kNonfiniteResult);
  EXPECT_EQ(kt::Bytes(output),saved); EXPECT_EQ(kt::Bytes(huge_base),huge_bytes);
  EXPECT_EQ(kt::Bytes(base),base_bytes); EXPECT_EQ(kt::Bytes(reference),reference_bytes);
  ASSERT_EQ(native::EvaluateForce(reference,base,good,output),Status::kSuccess);
  oracle::Check(reference,base.data(),good,output);
}
} // namespace
