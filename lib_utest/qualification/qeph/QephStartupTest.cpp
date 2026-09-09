#include "QephStartupFixture.h"
#include "lib_src/elements/ReissnerFrame.h"

namespace {
using namespace qeph_startup_test;
static_assert(std::is_same_v<Vec3,tl::fea::reissner::Vec3>);
static_assert(std::is_same_v<tl::math::Matrix3,tl::fea::reissner::Matrix3>);
static_assert(sizeof(Vec3)==3*sizeof(double));
static_assert(sizeof(tl::math::Matrix3)==9*sizeof(double));

TEST(QephStartup, ActualNativeParityAcrossSourceScaleWarpageRotationAndCyclicIds) {
  for (unsigned fixture=0;fixture<kCases;++fixture) for (unsigned shift=0;shift<4;++shift)
    for (bool transform:{false,true}) {
      SCOPED_TRACE(fixture);
      SCOPED_TRACE(shift);
      SCOPED_TRACE(transform);
      const auto input=Reparameterize(Case(fixture),shift,transform); native::Reference reference;
      ASSERT_EQ(native::Initialize(NativeInput(input),reference),native::Status::kSuccess);
      port::ReferenceData output;
      ASSERT_EQ(port::InitializeReference(input,output),port::Status::kSuccess);
      SameInput(output.input,input); Agreement(output,reference.data());
    }
}

TEST(QephStartup, IndependentRectanglesAndSaddlesUseProjectedAreaAndDistinctInertias) {
  for (unsigned fixture=0;fixture<kOrthogonalCases;++fixture) {
    const auto input=Case(fixture); port::ReferenceData output;
    ASSERT_EQ(port::InitializeReference(input,output),port::Status::kSuccess);
    const double a=input.position[1].x,b=input.position[2].y,area=4*a*b;
    Near(output.area,area,area);
    for (unsigned i=0;i<9;++i) Near(output.frame.v[i],i%4==0?1.:0.,1.);
    const double dx[]{-b,b,b,-b},dy[]{-a,-a,a,a};
    const Vec3 local[]{{0,0,0},{2*a,0,0},{2*a,2*b,0},{0,2*b,0}};
    const long double mass=static_cast<long double>(input.density)*input.thickness*area/4;
    const long double physical=mass*input.thickness*input.thickness/12;
    const long double added=mass*area/12;
    for (unsigned n=0;n<4;++n) {
      Near(output.derivative_x[n],dx[n],a+b); Near(output.derivative_y[n],dy[n],a+b);
      Near(output.local_position[n],local[n],a+b);
      Near(output.nodal_mass[n],static_cast<double>(mass),static_cast<double>(mass));
      Near(output.physical_inertia[n],static_cast<double>(physical),static_cast<double>(physical));
      Near(output.added_inertia[n],static_cast<double>(added),static_cast<double>(added));
      Near(output.isotropic_inertia[n],static_cast<double>(physical+added),static_cast<double>(physical+added));
      EXPECT_GT(output.added_inertia[n],output.physical_inertia[n]);
    }
  }
}

TEST(QephStartup, IndependentSkewBasisAndCofactorsAcrossScaleCyclicOrderAndWorldRotation) {
  // For edges (2,0),(.5,1), the IREP0 angle bisector is proportional to
  // (2+sqrt(5),-1), not either edge or a diagonal. Cyclic relabeling rotates
  // this basis by quarter turns; this closed form does not invoke frame code.
  const double ex=(2+std::sqrt(5.))/std::hypot(2+std::sqrt(5.),1.);
  const double ey=-1/std::hypot(2+std::sqrt(5.),1.);
  const Vec3 basis[]{{ex,ey,0},{-ey,ex,0},{-ex,-ey,0},{ey,-ex,0}};
  // Independent constant-Jacobian Q4 cofactor rows, in original world XY.
  const double bx[]{-.5,.5,.5,-.5},by[]{-.75,-1.25,.75,1.25};
  for (unsigned fixture=kOrthogonalCases;fixture<kCases;++fixture)
    for (unsigned shift=0;shift<4;++shift) for (bool transform:{false,true}) {
      SCOPED_TRACE(fixture);
      SCOPED_TRACE(shift);
      SCOPED_TRACE(transform);
      const auto original=Case(fixture),input=Reparameterize(original,shift,transform);
      const double scale=original.position[2].y,area=2*scale*scale;
      const Vec3 e1=basis[shift],e2=basis[(shift+1)%4];
      Vec3 world_e1=e1,world_e2=e2,world_e3{0,0,1};
      if (transform) {
        const auto q=CommonRotation();
        world_e1=Rotate(q,e1); world_e2=Rotate(q,e2); world_e3=Rotate(q,world_e3);
      }
      port::ReferenceData output;
      ASSERT_EQ(port::InitializeReference(input,output),port::Status::kSuccess);
      Near(output.area,area,area);
      Near({output.frame.v[0],output.frame.v[3],output.frame.v[6]},world_e1,1.);
      Near({output.frame.v[1],output.frame.v[4],output.frame.v[7]},world_e2,1.);
      Near({output.frame.v[2],output.frame.v[5],output.frame.v[8]},world_e3,1.);
      const long double mass=static_cast<long double>(input.density)*input.thickness*area/4;
      const long double physical=mass*input.thickness*input.thickness/12,added=mass*area/12;
      for (unsigned n=0;n<4;++n) {
        const unsigned k=(n+shift)%4;
        const auto p=Difference(original.position[k],original.position[shift]);
        Near(output.local_position[n],{p.x*e1.x+p.y*e1.y,p.x*e2.x+p.y*e2.y,0},Scale(input));
        Near(output.derivative_x[n],scale*(bx[k]*e1.x+by[k]*e1.y),Scale(input));
        Near(output.derivative_y[n],scale*(bx[k]*e2.x+by[k]*e2.y),Scale(input));
        Near(output.nodal_mass[n],static_cast<double>(mass),static_cast<double>(mass));
        Near(output.physical_inertia[n],static_cast<double>(physical),static_cast<double>(physical));
        Near(output.added_inertia[n],static_cast<double>(added),static_cast<double>(added));
        Near(output.isotropic_inertia[n],static_cast<double>(physical+added),static_cast<double>(physical+added));
      }
    }
}

TEST(QephStartup, DeterminantBoundaryAndLateArithmeticFailurePreserveAllOutputBytes) {
  port::ReferenceData output;
  ASSERT_EQ(port::InitializeReference(Case(2),output),port::Status::kSuccess);
  const auto before=Bytes(output);
  for (unsigned kind=0;kind<9;++kind) {
    SCOPED_TRACE(kind);
    EXPECT_NE(port::InitializeReference(Failure(kind),output),port::Status::kSuccess);
    EXPECT_EQ(Bytes(output),before);
  }
  for (double half_side:{std::nextafter(2.5e-11,0.),2.5e-11,
                         std::nextafter(2.5e-11,std::numeric_limits<double>::infinity())}) {
    EXPECT_EQ(port::InitializeReference(Threshold(half_side),output),port::Status::kUnsupportedGeometry);
    EXPECT_EQ(Bytes(output),before);
  }
  // Predeclared clear margin; no value is selected after a failed run.
  const auto safely_above=Threshold(2.5e-11*(1+512*std::numeric_limits<double>::epsilon()));
  native::Reference reference;
  ASSERT_EQ(native::Initialize(NativeInput(safely_above),reference),native::Status::kSuccess);
  ASSERT_EQ(port::InitializeReference(safely_above,output),port::Status::kSuccess);
  Agreement(output,reference.data());
  port::ReferenceData clean;
  ASSERT_EQ(port::InitializeReference(Case(2),clean),port::Status::kSuccess);
  ASSERT_EQ(port::InitializeReference(Case(2),output),port::Status::kSuccess);
  SameResult(output,clean);
}

TEST(QephStartup, ReinitializationSupportsAliasedImmutableInputWithoutChangingItsValues) {
  port::ReferenceData output;
  const auto input=Reparameterize(Case(5),3,true);
  ASSERT_EQ(port::InitializeReference(input,output),port::Status::kSuccess);
  const auto clean=output;
  ASSERT_EQ(port::InitializeReference(output.input,output),port::Status::kSuccess);
  SameResult(output,clean);
  output.input.poisson_ratio=.5;
  const auto before=Bytes(output);
  EXPECT_EQ(port::InitializeReference(output.input,output),port::Status::kInvalidInput);
  EXPECT_EQ(Bytes(output),before);
}
}  // namespace
