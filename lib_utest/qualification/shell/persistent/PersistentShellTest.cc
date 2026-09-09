#include "PersistentShellTestHelpers.h"
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>

namespace tl::qualification::shell {
namespace {
using namespace test;
constexpr double Dt=.001;
std::string Precise(double value){std::ostringstream out;out<<std::setprecision(17)<<value;return out.str();}

TEST(PersistentElastic, TraceFreeLoadUnloadReloadRetainsExactLinearState){
  Configuration c=Config();Motion m=Rectangle();PersistentShell shell;
  ASSERT_EQ(shell.Initialize(c,m.view()).status,Status::Ok);
  const Strain increment={2e-5,-2e-5,1e-5,0,0,.002,-.002,.001};
  Strain total{};double previous_work=0;bool unloading=false;
  for(double scale:{1.0,1.0,0.0,-.5,-.5,-1.0,.75}){
    const Strain delta=Scale(increment,scale);Rates(m,delta,Dt);total=Add(total,delta);
    Snapshot out;ASSERT_TRUE(Step(shell,m,Dt,&out));
    CheckElastic(out.elements[0],total,.12,.01);Near(out.elements[0].thickness,.01,1e-14);
    Near(out.elements[0].step_thickness,.01,1e-14);
    const double work=out.elements[0].work[0]+out.elements[0].work[1];
    Near(out.elements[0].material_work_increment,work-previous_work,2e-9);
    Near(out.elements[0].hourglass_work_increment,0,1e-12);
    if(work<previous_work-1e-10)unloading=true;
    if(scale==0){Near(work,previous_work,1e-12);EXPECT_GT(Norm(Get(out.force,0)),0);}
    previous_work=work;Balance(m,out);
  }
  EXPECT_TRUE(unloading);EXPECT_EQ(shell.accepted().epoch,7u);Near(shell.accepted().time,.007,1e-15);
}

TEST(PersistentElastic, SplittingFixedGeometryIncrementPreservesStressAndWork){
  const Strain total={3e-5,-3e-5,2e-5,1e-5,-2e-5,.0015,-.0015,.0007};
  Configuration c=Config();Motion coarse=Rectangle(),fine=Rectangle();PersistentShell one,many;
  ASSERT_EQ(one.Initialize(c,coarse.view()).status,Status::Ok);ASSERT_EQ(many.Initialize(c,fine.view()).status,Status::Ok);
  Rates(coarse,total,4*Dt);Snapshot a,b;ASSERT_TRUE(Step(one,coarse,4*Dt,&a));
  for(int i=0;i<4;++i){Rates(fine,Scale(total,.25),Dt);ASSERT_TRUE(Step(many,fine,Dt,&b));}
  CheckElastic(a.elements[0],total,.12,.01);CheckElastic(b.elements[0],total,.12,.01);
  for(int ip=0;ip<3;++ip)for(int j=0;j<5;++j)Near(a.elements[0].points[ip].stress[j],b.elements[0].points[ip].stress[j],2e-5);
  for(int j=0;j<2;++j)Near(a.elements[0].work[j],b.elements[0].work[j],2e-9);
  for(int i=0;i<4;++i){Near(Get(a.force,i),Get(b.force,i));Near(Get(a.couple,i),Get(b.couple,i),2e-9);}
}

TEST(PersistentAssembly, MembraneBendingAndShearFollowIndependentBoundaryWork){
  Configuration c=Config();Motion m=Rectangle();PersistentShell shell;
  ASSERT_EQ(shell.Initialize(c,m.view()).status,Status::Ok);
  const Strain total={2e-5,-2e-5,1e-5,-1e-5,2e-5,.002,-.002,.001};
  Rates(m,total,Dt);Snapshot out;ASSERT_TRUE(Step(shell,m,Dt,&out));CheckElastic(out.elements[0],total,.12,.01);
  std::array<Vec,MaxNodes> f{},couple{};AddExpectedForces(m,c.elements[0],total,.01,&f,&couple);
  for(int i=0;i<4;++i){Near(Get(out.force,i),f[i]);Near(Get(out.couple,i),couple[i],2e-9);}
  Balance(m,out);
  double rhs_work=0;for(int i=0;i<4;++i)rhs_work+=Dot(Get(out.force,i),Get(m.v,i))+Dot(Get(out.couple,i),Get(m.w,i));
  // Fresh elastic linear state: endpoint restoring work is twice the stored energy.
  Near(-.5*Dt*rhs_work,out.elements[0].work[0]+out.elements[0].work[1],2e-9);
}

TEST(PersistentAssembly, ProperCoordinateTransformPreservesLoadedHistoryAndResultants){
  Configuration c=Config();Motion base=Rectangle(),rotated=Rotated(base);PersistentShell a,b;
  ASSERT_EQ(a.Initialize(c,base.view()).status,Status::Ok);ASSERT_EQ(b.Initialize(c,rotated.view()).status,Status::Ok);
  const Strain inc={2e-5,-2e-5,1e-5,0,0,.002,-.002,.001};
  for(double scale:{1.0,0.0,-.4,.2}){
    base=Rectangle();Rates(base,Scale(inc,scale),Dt);rotated=Rotated(base);
    Snapshot sa,sb;ASSERT_TRUE(Step(a,base,Dt,&sa));ASSERT_TRUE(Step(b,rotated,Dt,&sb));
    for(int i=0;i<4;++i){Near(Get(sb.force,i),Rotate(Get(sa.force,i)),2e-6);Near(Get(sb.couple,i),Rotate(Get(sa.couple,i)),2e-8);}
    for(int ip=0;ip<3;++ip)for(int j=0;j<5;++j)Near(sb.elements[0].points[ip].stress[j],sa.elements[0].points[ip].stress[j],2e-4);
    for(int j=0;j<2;++j)Near(sb.elements[0].work[j],sa.elements[0].work[j],2e-9);
    Near(sb.elements[0].thickness,sa.elements[0].thickness,1e-13);Balance(rotated,sb);
  }
}

TEST(PersistentAssembly, SharedPhysicalNodesAccumulateBothSectionsWithoutDuplicatingCoordinates){
  Configuration c=Config();c.element_count=2;c.node_count=6;c.elements[0].nodes={0,1,2,3};
  c.elements[1].nodes={1,4,5,2};c.elements[1].thickness=.015;
  Motion m;m.count=6;Put(m.x,0,{-.4,-.15,0});Put(m.x,1,{0,-.15,0});Put(m.x,2,{0,.15,0});
  Put(m.x,3,{-.4,.15,0});Put(m.x,4,{.4,-.15,0});Put(m.x,5,{.4,.15,0});
  PersistentShell shell,permuted;Configuration reversed=c;std::swap(reversed.elements[0],reversed.elements[1]);
  ASSERT_EQ(shell.Initialize(c,m.view()).status,Status::Ok);ASSERT_EQ(permuted.Initialize(reversed,m.view()).status,Status::Ok);
  const Strain total={2e-5,-2e-5,1e-5,0,0,0,0,0};Rates(m,total,Dt);
  Snapshot a,b;ASSERT_TRUE(Step(shell,m,Dt,&a));ASSERT_TRUE(Step(permuted,m,Dt,&b));
  ASSERT_EQ(a.node_count,6u);ASSERT_EQ(a.element_count,2u);
  std::array<Vec,MaxNodes> f{},couple{};
  for(int e=0;e<2;++e){CheckElastic(a.elements[e],total,.12,c.elements[e].thickness);AddExpectedForces(m,c.elements[e],total,c.elements[e].thickness,&f,&couple);}
  for(int i=0;i<6;++i){Near(Get(a.force,i),f[i]);Near(Get(a.couple,i),couple[i],2e-9);Near(Get(a.force,i),Get(b.force,i));Near(Get(a.position,i),Get(m.x,i),1e-14);}
  EXPECT_GT(Norm(Get(a.force,1)),0); // Different thicknesses give an accounted-for traction jump.
  Balance(m,a);
}

TEST(PersistentThickness, ITHKControlsStepSnapshotNotWhetherPhysicalThicknessChanges){
  Configuration fixed=Config(),updated=fixed;updated.update_thickness=true;
  Motion m=Rectangle();PersistentShell a,b;
  ASSERT_EQ(a.Initialize(fixed,m.view()).status,Status::Ok);ASSERT_EQ(b.Initialize(updated,m.view()).status,Status::Ok);
  const double de=.004,alpha=Poisson*de/(1-Poisson),t0=.01;
  Strain inc{};inc[0]=de;Rates(m,inc,Dt);Snapshot sa,sb;
  for(int step=1;step<=8;++step){
    ASSERT_TRUE(Step(a,m,Dt,&sa));ASSERT_TRUE(Step(b,m,Dt,&sb));
    Near(sa.elements[0].step_thickness,t0,1e-14);
    Near(sb.elements[0].step_thickness,t0*std::pow(1-alpha,step-1),1e-13);
    Near(sa.elements[0].thickness,t0*(1-step*alpha),1e-13);
    Near(sb.elements[0].thickness,t0*std::pow(1-alpha,step),1e-13);
    const auto sigma=PlaneStress(step*de,0,0);
    for(int j=0;j<3;++j){Near(sa.elements[0].normalized_force[j],sigma[j],2e-4);Near(sb.elements[0].normalized_force[j],sigma[j],2e-4);}
    Strain total{};total[0]=step*de;std::array<Vec,MaxNodes> expected{},moments{};
    AddExpectedForces(m,updated.elements[0],total,sb.elements[0].step_thickness,&expected,&moments);
    for(int i=0;i<4;++i)Near(Get(sb.force,i),expected[i],2e-4);
  }
  EXPECT_GT(sb.elements[0].thickness,sa.elements[0].thickness);
  EXPECT_LT(Norm(Get(sb.force,0)),Norm(Get(sa.force,0)));
}

TEST(PersistentGeometry, CurrentAndFrozenMetricsFollowTheirDeclaredUniformStretchConvention){
  auto frozen=Config(),current=frozen;current.geometry=GeometryMode::Current;
  Motion rest=Rectangle(),m=rest;PersistentShell a,b;
  ASSERT_EQ(a.Initialize(frozen,rest.view()).status,Status::Ok);ASSERT_EQ(b.Initialize(current,rest.view()).status,Status::Ok);
  const double dlambda=.0002;double current_strain=0;
  Snapshot sa,sb;
  for(int step=1;step<=6;++step){
    const double lambda=1+step*dlambda;
    for(int i=0;i<4;++i){const Vec p=Get(rest.x,i);Put(m.x,i,{lambda*p.x,p.y,0});Put(m.v,i,{dlambda*p.x/Dt,0,0});}
    ASSERT_TRUE(Step(a,m,Dt,&sa));ASSERT_TRUE(Step(b,m,Dt,&sb));
    current_strain+=dlambda/lambda;
    Near(sa.elements[0].area,.12,2e-13);Near(sb.elements[0].area,.12*lambda,2e-13);
    Near(sa.elements[0].generalized_strain[0],step*dlambda,2e-12);
    Near(sb.elements[0].generalized_strain[0],current_strain,2e-12);
    EXPECT_EQ(sa.elements[0].off,2);EXPECT_EQ(sb.elements[0].off,1);
    const auto sig_a=PlaneStress(step*dlambda,0,0),sig_b=PlaneStress(current_strain,0,0);
    for(int ip=0;ip<3;++ip)for(int j=0;j<3;++j){Near(sa.elements[0].points[ip].stress[j],sig_a[j],2e-4);Near(sb.elements[0].points[ip].stress[j],sig_b[j],2e-4);}
    Near(sb.elements[0].thickness,.01*(1-Poisson/(1-Poisson)*current_strain),2e-13);
    // Current one-point gradient integrates d(lambda)/lambda at the interval's
    // end. This is an operator/time-convention check, not finite-strain accuracy.
    Balance(m,sb);
  }
  EXPECT_LT(sb.elements[0].generalized_strain[0],sa.elements[0].generalized_strain[0]);
}

TEST(PersistentTransaction, RejectAfterEachKernelPreservesAllAcceptedFieldsAndCleanRetry){
  auto c=Config();c.stabilization.H1=.02;c.stabilization.H2=.03;c.stabilization.HELAS=.2;
  Motion m=Rectangle();PersistentShell shell;ASSERT_EQ(shell.Initialize(c,m.view()).status,Status::Ok);
  Strain increment={1e-5,-1e-5,0,0,0,.001,-.001,0};Rates(m,increment,Dt);
  const int signs[]={1,-1,1,-1};for(int i=0;i<4;++i)m.v[3*i]+=.01*signs[i];
  Snapshot loaded;ASSERT_TRUE(Step(shell,m,Dt,&loaded));EXPECT_NE(loaded.elements[0].hour[0],0);
  for(RejectAfter phase:{RejectAfter::Geometry,RejectAfter::Material,RejectAfter::Assembly}){
    const auto before=Bytes(shell.accepted());TrialToken clean;ASSERT_TRUE(Evaluate(shell,m,Dt,&clean));
    const auto expected=Bytes(*shell.trial());
    TrialToken rejected;const auto report=shell.Evaluate(MakeRequest(shell,m,Dt,phase),&rejected);
    EXPECT_EQ(report.status,Status::Rejected)<<report.message;EXPECT_EQ(shell.trial(),nullptr);
    EXPECT_EQ(Bytes(shell.accepted()),before);EXPECT_NE(shell.Commit(clean),Status::Ok);
    TrialToken retry;ASSERT_TRUE(Evaluate(shell,m,Dt,&retry));EXPECT_EQ(Bytes(*shell.trial()),expected);
    EXPECT_EQ(Bytes(shell.accepted()),before);ASSERT_EQ(shell.Commit(retry),Status::Ok);
  }
}

TEST(PersistentTransaction, InvalidCapacityOwnerEpochAndSupersededAttemptsCannotPublish){
  auto c=Config();Motion m=Rectangle();PersistentShell limited,owner,other;
  auto tiny=c;tiny.max_device_bytes=1;EXPECT_EQ(limited.Initialize(tiny,m.view()).status,Status::ResourceLimit);
  ASSERT_EQ(owner.Initialize(c,m.view()).status,Status::Ok);ASSERT_EQ(other.Initialize(c,m.view()).status,Status::Ok);
  Strain strain{};strain[0]=1e-5;strain[1]=-1e-5;Rates(m,strain,Dt);
  const auto before=Bytes(owner.accepted());TrialToken old,newer;ASSERT_TRUE(Evaluate(owner,m,Dt,&old));ASSERT_TRUE(Evaluate(owner,m,Dt,&newer));
  EXPECT_EQ(owner.Commit(old),Status::StaleTrial);EXPECT_EQ(other.Commit(newer),Status::StaleTrial);EXPECT_EQ(Bytes(owner.accepted()),before);
  ASSERT_EQ(owner.Commit(newer),Status::Ok);const auto accepted=Bytes(owner.accepted());EXPECT_NE(owner.Commit(newer),Status::Ok);
  TrialToken token;Request wrong=MakeRequest(owner,m,Dt);wrong.time_begin=0;
  EXPECT_EQ(owner.Evaluate(wrong,&token).status,Status::InvalidInput);EXPECT_EQ(Bytes(owner.accepted()),accepted);
  Motion bad=m;bad.v[0]=std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(owner.Evaluate(MakeRequest(owner,bad,Dt),&token).status,Status::InvalidInput);EXPECT_EQ(Bytes(owner.accepted()),accepted);
  bad=m;bad.x[8]=.02;EXPECT_EQ(owner.Evaluate(MakeRequest(owner,bad,Dt),&token).status,Status::UnsupportedGeometry);
  EXPECT_EQ(Bytes(owner.accepted()),accepted);
  TrialToken clean;ASSERT_TRUE(Evaluate(owner,m,Dt,&clean));const auto clean_candidate=Bytes(*owner.trial());
  bad=m;
  // Finite transverse rates pass input validation but overflow the quadratic
  // K2 membrane correction. This exercises actual post-kernel invalidation.
  for(int i=0;i<4;++i)bad.v[3*i+2]=(i==0||i==3)?1e200:-1e200;
  const Report overflow=owner.Evaluate(MakeRequest(owner,bad,Dt),&token);
  EXPECT_EQ(overflow.status,Status::InvalidOutput)<<overflow.message;
  EXPECT_EQ(owner.trial(),nullptr);EXPECT_EQ(Bytes(owner.accepted()),accepted);
  EXPECT_NE(owner.Commit(clean),Status::Ok);
  ASSERT_TRUE(Evaluate(owner,m,Dt,&token));EXPECT_EQ(Bytes(*owner.trial()),clean_candidate);
  EXPECT_EQ(Bytes(owner.accepted()),accepted);owner.Discard();EXPECT_EQ(owner.trial(),nullptr);
  EXPECT_NE(owner.Commit(token),Status::Ok);EXPECT_EQ(Bytes(owner.accepted()),accepted);
}

TEST(PersistentRigidTrajectory, SecantPathDriftIsMeasuredBoundedAndRefinesWithoutObjectivityClaim){
  RecordProperty("production_objectivity_qualified","false");
  // Exact endpoint coordinates and secant interval velocities, not same-instant
  // v=Omega cross x. The native IHBE1 algebra has a nonzero finite-step residual.
  const double duration=.016,omega=.5,t0=.01;std::array<double,3> membrane_drift{},shear_drift{};
  for(int level=0;level<3;++level){
    const int steps=16*(1<<level);const double dt=duration/steps,delta=omega*dt;
    auto c=Config();c.geometry=GeometryMode::Current;c.update_thickness=false;
    Motion rest=Rectangle(),m=rest;PersistentShell shell;ASSERT_EQ(shell.Initialize(c,rest.view()).status,Status::Ok);
    const double de=-2*std::cos(delta)*std::pow(std::sin(.5*delta),2);
    const double dg=delta-std::sin(delta);Snapshot out;
    for(int step=1;step<=steps;++step){
      const double a=step*delta,b=(step-1)*delta;
      for(int i=0;i<4;++i){const auto p=Get(rest.x,i);
        const Vec end{std::cos(a)*p.x,p.y,-std::sin(a)*p.x};
        const Vec begin{std::cos(b)*p.x,p.y,-std::sin(b)*p.x};
        Put(m.x,i,end);Put(m.v,i,Scale(Sub(end,begin),1/dt));Put(m.w,i,{0,omega,0});
      }
      ASSERT_TRUE(Step(shell,m,dt,&out));
      Near(out.elements[0].generalized_strain[0],step*de,5e-13,5e-7);
      Near(out.elements[0].generalized_strain[4],step*dg,5e-13,5e-5);
      for(int j:{1,2,3,5,6,7})Near(out.elements[0].generalized_strain[j],0,5e-13);
      Near(out.elements[0].step_thickness,t0,1e-14);
      Near(out.elements[0].thickness,t0*(1-Poisson/(1-Poisson)*step*de),2e-13);
    }
    membrane_drift[level]=std::abs(out.elements[0].generalized_strain[0]);
    shear_drift[level]=std::abs(out.elements[0].generalized_strain[4]);
    const std::string key="level_"+std::to_string(level)+"_";
    RecordProperty(key+"membrane_strain_drift",Precise(membrane_drift[level]));
    RecordProperty(key+"shear_strain_drift",Precise(shear_drift[level]));
    RecordProperty(key+"spurious_work_J",Precise(out.elements[0].work[0]+out.elements[0].work[1]));
    RecordProperty(key+"spurious_force_N",Precise(Norm(Get(out.force,0))));
    // Guard this small-angle characterization. These are not production
    // objectivity tolerances; stress-free rigid motion should have zero strain.
    EXPECT_GT(membrane_drift[level],1e-8);EXPECT_LE(membrane_drift[level],3e-6);
    EXPECT_LE(shear_drift[level],1e-8);EXPECT_LE(std::abs(out.elements[0].work[0]+out.elements[0].work[1]),.002);
    EXPECT_EQ(out.elements[0].hourglass_work_increment,0);EXPECT_EQ(out.elements[0].points[0].plastic_strain,0);
  }
  for(int level=1;level<3;++level){
    EXPECT_LT(membrane_drift[level],.55*membrane_drift[level-1]);
    EXPECT_GT(membrane_drift[level],.45*membrane_drift[level-1]);
    EXPECT_LT(shear_drift[level],.30*shear_drift[level-1]);
    EXPECT_GT(shear_drift[level],.20*shear_drift[level-1]);
  }
}
}  // namespace
}  // namespace tl::qualification::shell
