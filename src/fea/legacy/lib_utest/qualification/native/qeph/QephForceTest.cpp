#include "QephForceTestFixture.h"
#include "NativeQephBridge.h"

namespace {
using namespace qeph_force_test;
extern "C" void qeph_q2_scatter(const double*,const double*,const int*,double*);

void Balance(const ForceTrial& result,const std::array<Vec3,4>& position,double absolute=kAbsolute) {
  Vec3 force{},moment{};
  for (unsigned n=0;n<4;++n) {
    force=Add(force,result.internal_force[n]);
    moment=Add(moment,Add(Cross(position[n],result.internal_force[n]),result.internal_couple[n]));
  }
  Near(force,{},absolute); Near(moment,{},absolute);
}

TEST(QephForce, EightAffineModesMatchIndependentLaw1StressAndReportedThickness) {
  const auto input=Rectangle(); Reference reference;
  ASSERT_EQ(Initialize(input,reference),Status::kSuccess);
  constexpr double rate=1e-4,dt=1e-6;
  const double e=rate*dt,a=input.young_modulus/(1-input.poisson_ratio*input.poisson_ratio);
  const double b=a*input.poisson_ratio,g=input.young_modulus/(2*(1+input.poisson_ratio));
  const double viscosity=1.414*.015*input.density*std::sqrt(input.young_modulus/input.density)*
                         std::sqrt(2.)/dt;
  for (unsigned mode=0;mode<8;++mode) {
    SCOPED_TRACE(mode); History initial;
    ASSERT_EQ(InitializeHistory(reference,{},initial),Status::kSuccess);
    auto interval=Interval(input,initial,dt); Mode(interval,mode,rate);
    ForceTrial result; ASSERT_EQ(EvaluateForce(reference,initial,interval,result),Status::kSuccess);
    const auto& h=result.proposed_history.data();
    std::array<double,5> material{},total{}; std::array<double,3> moment{};
    if (mode==0) { material[0]=a*e; material[1]=b*e; }
    if (mode==1) { material[0]=b*e; material[1]=a*e; }
    if (mode==2) material[2]=g*e;
    if (mode==3||mode==4) material[mode]=g*(5./6)*e;
    if (mode==5) moment={{a*input.thickness*e/12,b*input.thickness*e/12,0}};
    if (mode==6) moment={{b*input.thickness*e/12,a*input.thickness*e/12,0}};
    if (mode==7) moment[2]=g*input.thickness*e/12;
    total=material;
    if (mode<2) { total[mode]+=viscosity*e; total[1-mode]+=.5*viscosity*e; }
    if (mode==2) total[2]+=viscosity*e/3;
    Near(h.material_stress,material); Near(h.stress,total); Near(h.bending_stress,moment);
    std::array<double,8> strain{}; strain[mode]=e;
    Near(h.strain_curvature,strain,2e-12);
    // Tiny temporal shear correction is bounded by Q1's independent rate gate.
    const double thickness=input.thickness*(1-(mode<2?input.poisson_ratio*e/(1-input.poisson_ratio):0));
    Near(h.thickness,thickness); EXPECT_DOUBLE_EQ(result.diagnostics.effective_thickness,input.thickness);
    if (mode<5) {
      for (double r:result.kinematics.hourglass_rate) EXPECT_NEAR(r,0.,2e-12);
      const double work=.5*input.thickness*2*total[mode]*e;
      Near(h.internal_work[0],work,kEnergy); Near(h.internal_work[1],0.,kEnergy);
      Near(h.hourglass_viscous_work,0.,kEnergy);
    }
    Kinematics q1; ASSERT_EQ(EvaluatePrescribed(reference,interval,q1),Status::kSuccess);
    SameGeometry(result.kinematics,q1);
  }
}

TEST(QephForce, ConstantStressAndBendingMatchIndependentSpatialVirtualWork) {
  const auto input=Rectangle(); Reference reference; History seed;
  ASSERT_EQ(Initialize(input,reference),Status::kSuccess);
  HistoryValues values; values.thickness=input.thickness;
  values.material_stress={{2.,-3.,.7,.4,-.6}};
  values.stress={{99.,98.,97.,96.,95.}}; // Must not become material stress at rest.
  values.bending_stress={{1.1,-.8,.3}};
  ASSERT_EQ(PreparePrescribedHistory(reference,values,{},seed),Status::kSuccess);
  const auto stationary=Interval(input,seed); ForceTrial force;
  ASSERT_EQ(EvaluateForce(reference,seed,stationary,force),Status::kSuccess);
  Near(force.proposed_history.data().stress,values.material_stress);
  Balance(force,input.position);
  for (unsigned mode=0;mode<8;++mode) {
    SCOPED_TRACE(mode); auto virtual_field=stationary; Mode(virtual_field,mode,1.);
    const double expected=mode<5?2*input.thickness*values.material_stress[mode]:
                                2*input.thickness*input.thickness*values.bending_stress[mode-5];
    Near(Power(force,virtual_field),expected);
  }
  Near(force.proposed_history.data().internal_work,std::array<double,2>{},kEnergy);
}

TEST(QephForce, NativeStiffnessAndCompleteScatterPreserveUnitsAndInternalSign) {
  const auto input=Rectangle(); Reference reference; History initial;
  ASSERT_EQ(Initialize(input,reference),Status::kSuccess);
  ASSERT_EQ(InitializeHistory(reference,{},initial),Status::kSuccess);
  auto interval=Interval(input,initial); Mode(interval,2,1e-4);
  ForceTrial force; ASSERT_EQ(EvaluateForce(reference,initial,interval,force),Status::kSuccess);
  const auto& d=force.diagnostics;
  Near(d.membrane_viscosity,.015); Near(d.stabilization_viscosity,.015);
  const double c=std::sqrt(input.young_modulus/input.density);
  const double length=force.kinematics.characteristic_length*(std::sqrt(1+.015*.015)-.015);
  const double a=input.young_modulus/(1-input.poisson_ratio*input.poisson_ratio);
  const double stiffness=.5*(input.thickness*2)*a/(length*length);
  Near(d.native_sound_speed,c); Near(d.translational_stiffness,stiffness);
  Near(d.rotational_stiffness,stiffness*(input.thickness*input.thickness+2)/12);
  Near(d.unscaled_element_dt,length/c);
  std::array<double,24> packed{};
  const auto f=native::detail::PackNodes(force.internal_force),m=native::detail::PackNodes(force.internal_couple);
  std::copy(f.begin(),f.end(),packed.begin()); std::copy(m.begin(),m.end(),packed.begin()+12);
  const double coefficients[]{d.translational_stiffness,d.rotational_stiffness,1.25,.75};
  const int connectivity[]{3,1,4,2}; std::array<double,32> rhs{};
  {
    const std::lock_guard<std::mutex> guard(native::detail::NativeContext());
    qeph_q2_scatter(packed.data(),coefficients,connectivity,rhs.data());
  }
  for (unsigned n=0;n<4;++n) {
    const unsigned node=connectivity[n]-1;
    for (unsigned axis=0;axis<3;++axis) {
      EXPECT_DOUBLE_EQ(rhs[3*node+axis],-packed[3*n+axis]);
      EXPECT_DOUBLE_EQ(rhs[12+3*node+axis],-packed[12+3*n+axis]);
    }
    const double factor=n%2?coefficients[3]:coefficients[2];
    Near(rhs[24+node],factor*d.translational_stiffness);
    Near(rhs[28+node],factor*d.rotational_stiffness);
  }
}

TEST(QephForce, NonzeroPlanarAndWarpedHistoryForceAndCoupleAreWorldCovariant) {
  const auto q=tl::math::Product(Rotation({.4,-.7,.3}),Rotation({-.2,.25,1.1}));
  const Vec3 translation{1.25,-.75,.5};
  for (double warp:{0.,.05}) {
    SCOPED_TRACE(warp); const auto input=Rectangle(warp); auto rotated=input;
    for (auto& x:rotated.position) x=Add(Rotate(q,x),translation);
    Reference a,b; History ha,hb;
    ASSERT_EQ(Initialize(input,a),Status::kSuccess);
    ASSERT_EQ(Initialize(rotated,b),Status::kSuccess);
    HistoryValues seed; seed.thickness=input.thickness;
    seed.material_stress={{2,-3,.7,.4,-.6}}; seed.stress=seed.material_stress;
    seed.bending_stress={{1.1,-.8,.3}};
    for (unsigned i=0;i<12;++i) seed.stabilization[i]=.02*(i+1);
    ASSERT_EQ(PreparePrescribedHistory(a,seed,{},ha),Status::kSuccess);
    ASSERT_EQ(PreparePrescribedHistory(b,seed,{},hb),Status::kSuccess);
    auto ia=Interval(input,ha,1e-3),ib=Interval(rotated,hb,1e-3);
    for (unsigned n=0;n<4;++n) {
      const auto x=input.position[n];
      ia.velocity_midpoint[n]={.01*x.x+.02*x.y,-.015*x.y,.005*x.x};
      ia.omega_midpoint[n]={.007*x.y,-.006*x.x,.004};
      ib.velocity_midpoint[n]=Rotate(q,ia.velocity_midpoint[n]);
      ib.omega_midpoint[n]=Rotate(q,ia.omega_midpoint[n]);
    }
    ForceTrial fa,fb;
    ASSERT_EQ(EvaluateForce(a,ha,ia,fa),Status::kSuccess);
    ASSERT_EQ(EvaluateForce(b,hb,ib,fb),Status::kSuccess);
    SameHistory(fa.proposed_history.data(),fb.proposed_history.data(),kCovariance);
    Balance(fa,ia.position_endpoint,kCovariance); Balance(fb,ib.position_endpoint,kCovariance);
    for (unsigned n=0;n<4;++n) {
      Near(Rotate(q,fa.internal_force[n]),fb.internal_force[n],kCovariance);
      Near(Rotate(q,fa.internal_couple[n]),fb.internal_couple[n],kCovariance);
    }
  }
}

TEST(QephForce, GeneralRigidPathReportsActualNativeForceAndWorkWithoutFalseTemporalOrder) {
  const Vec3 omega=Scale({1,2,3},1/std::sqrt(14.));
  for (double warp:{0.,.05}) for (double dt:{.04,.02,.01}) {
    SCOPED_TRACE(warp);
    SCOPED_TRACE(dt);
    const auto input=Rectangle(warp); Reference reference; History initial;
    ASSERT_EQ(Initialize(input,reference),Status::kSuccess);
    ASSERT_EQ(InitializeHistory(reference,{},initial),Status::kSuccess);
    auto interval=Interval(input,initial,dt);
    const auto endpoint=Rotation(Scale(omega,dt)),midpoint=Rotation(Scale(omega,.5*dt));
    for (unsigned n=0;n<4;++n) {
      interval.position_endpoint[n]=Rotate(endpoint,input.position[n]);
      interval.velocity_midpoint[n]=Cross(omega,Rotate(midpoint,input.position[n]));
      interval.omega_midpoint[n]=omega;
    }
    ForceTrial result; ASSERT_EQ(EvaluateForce(reference,initial,interval,result),Status::kSuccess);
    Kinematics q1; ASSERT_EQ(EvaluatePrescribed(reference,interval,q1),Status::kSuccess);
    SameGeometry(result.kinematics,q1);
    double maximum=0;
    for (auto force:result.internal_force) maximum=std::max(maximum,Norm(force));
    EXPECT_GT(maximum,0.);
    const std::string prefix=(warp==0?"planar_":"warped_")+Text(dt);
    RecordProperty(prefix+"_maximum_force_N",Text(maximum));
    RecordProperty(prefix+"_membrane_work_J",Text(result.proposed_history.data().internal_work[0]));
    RecordProperty(prefix+"_bending_work_J",Text(result.proposed_history.data().internal_work[1]));
    RecordProperty(prefix+"_viscous_hourglass_work_J",Text(result.proposed_history.data().hourglass_viscous_work));
  }
  RecordProperty("source_dynamics_qualified","false");
}
}  // namespace
