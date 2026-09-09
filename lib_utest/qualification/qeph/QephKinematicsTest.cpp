#include "QephKinematicsFixture.h"

namespace {
using namespace qeph_kinematics_test;

TEST(QephKinematicsPort, ActualNativeAllFieldsAcrossScaleWarpageSkewAndPrescribedRates) {
  for(unsigned fixture=0;fixture<kCases;++fixture) for(unsigned shift=0;shift<4;++shift)
    for(bool transform:{false,true}) {
      SCOPED_TRACE(fixture);
      SCOPED_TRACE(shift);
      SCOPED_TRACE(transform);
      const auto original=Case(fixture),input=qeph_startup_test::Reparameterize(original,shift,transform);
      port::ReferenceData ref; native::Reference native_ref;
      ASSERT_EQ(port::InitializeReference(input,ref),port::Status::kSuccess);
      ASSERT_EQ(native::Initialize(NativeInput(input),native_ref),native::Status::kSuccess);
      const auto saved=Bytes(ref);
      for(unsigned pattern=0;pattern<3;++pattern) {
        SCOPED_TRACE(pattern);
        const auto interval=qeph_kinematics_test::Reparameterize(Pattern(original,pattern),shift,transform);
        port::Kinematics output; native::Kinematics expected;
        ASSERT_EQ(native::EvaluatePrescribed(native_ref,NativeInterval(interval),expected),native::Status::kSuccess);
        ASSERT_EQ(port::EvaluatePrescribed(ref,interval,output),port::Status::kSuccess);
        qeph_kinematics_test::Agreement(output,expected,interval);
        EXPECT_EQ(Bytes(ref),saved);
      }
    }
  RecordProperty("native_parity_configurations",216);
  RecordProperty("force_or_dynamics_qualified","false");
}

TEST(QephKinematicsPort, IndependentStaticSaddleNormalsAndNativePlanarSwitch) {
  for(unsigned fixture=0;fixture<kOrthogonalCases;++fixture) {
    const auto input=Case(fixture); port::ReferenceData ref; port::Kinematics result;
    const auto interval=Stationary(input);
    ASSERT_EQ(port::InitializeReference(input,ref),port::Status::kSuccess);
    ASSERT_EQ(port::EvaluatePrescribed(ref,interval,result),port::Status::kSuccess);
    const double a=input.position[1].x,b=input.position[2].y,w=input.position[0].z;
    Field(result.area,4*a*b,4*a*b); Field(result.raw_warpage_abs,w,Scale(input));
    constexpr double xi[]{-1,1,1,-1},eta[]{-1,-1,1,1};
    for(unsigned n=0;n<4;++n) {
      const Vec3 normal=Unit({-b*w*eta[n],-a*w*xi[n],a*b});
      Field(result.local_normals[n],normal,1);
      Field(result.local_position[n],input.position[n],Scale(input));
    }
    ZeroRates(result,interval);
  }
  for(double warp:{1e-6,1e-3}) {
    auto input=Case(0);
    for(unsigned n=0;n<4;++n) input.position[n].z=n%2?-warp:warp;
    port::ReferenceData ref; port::Kinematics result;
    ASSERT_EQ(port::InitializeReference(input,ref),port::Status::kSuccess);
    ASSERT_EQ(port::EvaluatePrescribed(ref,Stationary(input),result),port::Status::kSuccess);
    EXPECT_EQ(result.planar,warp*warp<1.25e-8);
    Field(result.raw_warpage_abs,warp,Scale(input)); EXPECT_GT(result.raw_warpage_abs,0.);
    Field(result.effective_warpage,result.planar?0:warp,Scale(input));
    if(result.planar) {
      for(double v:result.projection_inverse) EXPECT_EQ(v,0.);
      for(const auto& v:result.projection_columns) { EXPECT_EQ(v.x,0.); EXPECT_EQ(v.y,0.); EXPECT_EQ(v.z,0.); }
    }
  }
}

TEST(QephKinematicsPort, NativeRealLiteralPrecisionHasIndependentCharacteristicLengthOracle) {
  for(unsigned fixture=0;fixture<kOrthogonalCases;++fixture) {
    SCOPED_TRACE(fixture);
    const auto input=Case(fixture); port::ReferenceData ref; port::Kinematics result;
    ASSERT_EQ(port::InitializeReference(input,ref),port::Status::kSuccess);
    const auto interval=Stationary(input);
    ASSERT_EQ(port::EvaluatePrescribed(ref,interval,result),port::Status::kSuccess);
    const double expected=NativeRectangleLength(input.position[1].x,input.position[2].y);
    Field(result.characteristic_length,expected,LengthScale(interval),kRoundoff,"characteristic_length");
  }
  // Frozen first-run fixture0 values identify the actual literal-kind error;
  // this is a reference precision regression, not a larger error allowance.
  EXPECT_GT(std::abs(NativeRectangleLength(1,.5)-1.0120397997335147),2e-8);
}

TEST(QephKinematicsPort, IndependentEightAffineRatesKeepOrderAndFiniteStepCorrection) {
  for(unsigned fixture:{0u,2u,4u}) {
    const auto input=Case(fixture); port::ReferenceData ref;
    ASSERT_EQ(port::InitializeReference(input,ref),port::Status::kSuccess);
    for(unsigned mode=0;mode<8;++mode) {
      SCOPED_TRACE(fixture);
      SCOPED_TRACE(mode);
      port::Kinematics output;
      ASSERT_EQ(port::EvaluatePrescribed(ref,Affine(input,mode),output),port::Status::kSuccess);
      AffineTruth(output,input,mode);
    }
  }
}

TEST(QephKinematicsPort, ExactRigidPhaseTruthPreservesGeneralAxisShearResidual) {
  for(unsigned fixture:{0u,2u,4u}) {
    const auto input=Case(fixture); port::ReferenceData ref;
    ASSERT_EQ(port::InitializeReference(input,ref),port::Status::kSuccess);
    for(Vec3 omega:{Vec3{0,0,1},Unit({1,2,3})}) for(double dt:{.04,.02,.01}) {
      const auto interval=Rigid(input,omega,dt); port::Kinematics output;
      ASSERT_EQ(port::EvaluatePrescribed(ref,interval,output),port::Status::kSuccess);
      RigidTruth(output,interval,omega);
      if(omega.x!=0) {
        const double expected=-omega.x*(1-std::cos(.5*dt))-omega.y*omega.z*std::sin(.5*dt);
        Field(output.regular_rate[4],expected,1);
        EXPECT_GT(std::abs(output.regular_rate[4]),.2*dt); // Source O(h), not a false zero/order claim.
      }
    }
  }
  RecordProperty("source_dynamics_qualified","false");
}

TEST(QephKinematicsPort, CommonWorldRotationCovarianceIncludesWarpedProjectionAndSkew) {
  for(unsigned fixture=0;fixture<kCases;++fixture) {
    const auto input=Case(fixture),changed=qeph_startup_test::Reparameterize(input,0,true);
    port::ReferenceData ref,other;
    ASSERT_EQ(port::InitializeReference(input,ref),port::Status::kSuccess);
    ASSERT_EQ(port::InitializeReference(changed,other),port::Status::kSuccess);
    for(unsigned pattern=0;pattern<3;++pattern) {
      SCOPED_TRACE(fixture);
      SCOPED_TRACE(pattern);
      const auto in=Pattern(input,pattern),out=qeph_kinematics_test::Reparameterize(in,0,true);
      port::Kinematics a,b;
      ASSERT_EQ(port::EvaluatePrescribed(ref,in,a),port::Status::kSuccess);
      ASSERT_EQ(port::EvaluatePrescribed(other,out,b),port::Status::kSuccess);
      qeph_kinematics_test::Agreement(b,a,in,false,kCovariance);
      for(unsigned n=0;n<3;++n) Field(Column(b.frame,n),Rotate(CommonRotation(),Column(a.frame,n)),1,kCovariance);
    }
  }
}

TEST(QephKinematicsPort, MalformedInputsAndLateArithmeticPreserveOutputReferenceAndRetry) {
  port::ReferenceData ref;
  ASSERT_EQ(port::InitializeReference(Case(1),ref),port::Status::kSuccess);
  port::Kinematics output;
  const auto valid=Pattern(Case(1),1);
  ASSERT_EQ(port::EvaluatePrescribed(ref,valid,output),port::Status::kSuccess);
  const auto saved_output=Bytes(output);
  const auto saved_ref=Bytes(ref);
  for(unsigned kind=0;kind<5;++kind) {
    auto bad=ref; CorruptReference(bad,kind); const auto saved_bad=Bytes(bad);
    EXPECT_EQ(port::EvaluatePrescribed(bad,valid,output),port::Status::kInvalidReference);
    EXPECT_EQ(Bytes(output),saved_output); EXPECT_EQ(Bytes(bad),saved_bad);
  }
  for(unsigned kind=0;kind<8;++kind) {
    SCOPED_TRACE(kind);
    const auto bad=InvalidInterval(kind); const auto saved_bad=Bytes(bad);
    const auto status=port::EvaluatePrescribed(ref,bad,output);
    EXPECT_NE(status,port::Status::kSuccess);
    if(kind==5) EXPECT_EQ(status,port::Status::kNonfiniteResult);
    EXPECT_EQ(Bytes(output),saved_output); EXPECT_EQ(Bytes(ref),saved_ref); EXPECT_EQ(Bytes(bad),saved_bad);
  }
  port::Kinematics clean;
  ASSERT_EQ(port::EvaluatePrescribed(ref,valid,clean),port::Status::kSuccess);
  ASSERT_EQ(port::EvaluatePrescribed(ref,valid,output),port::Status::kSuccess);
  qeph_kinematics_test::Agreement(output,clean,valid);
  // Caller sequence is a label; no implicit increment/history is introduced.
  auto later=valid; later.sample_index=std::numeric_limits<std::uint64_t>::max(); later.base_time=7;
  ASSERT_EQ(port::EvaluatePrescribed(ref,later,output),port::Status::kSuccess);
  EXPECT_EQ(output.sample_index,later.sample_index); EXPECT_EQ(output.base_time,7);
}
} // namespace
