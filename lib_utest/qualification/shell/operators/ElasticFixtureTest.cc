#include "ElasticFixture.h"

#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <limits>

namespace shell_spike {
namespace {
using Increment=std::array<double,8>;
const ThicknessRule kRules[]={ThicknessRule::OriginalMidpoint3,ThicknessRule::Gauss3};
Vec3 Add(Vec3 a,Vec3 b){return {a.x+b.x,a.y+b.y,a.z+b.z};}
Vec3 Mul(Vec3 a,double s){return {a.x*s,a.y*s,a.z*s};}
Vec3 Cross(Vec3 a,Vec3 b){return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};}
void Near(double a,double b,double absolute=1e-8){EXPECT_NEAR(a,b,absolute+2e-11*std::abs(b));}
void Near(Vec3 a,Vec3 b,double eps=1e-9){Near(a.x,b.x,eps);Near(a.y,b.y,eps);Near(a.z,b.z,eps);}
Vec3 ToWorld(const ElasticInput& in,Vec3 local){
  return Add(Add(Mul(in.basis[0],local.x),Mul(in.basis[1],local.y)),Mul(in.basis[2],local.z));
}

// Analytic affine nodal fields on the rectangle. No donor shape derivatives or
// generated quadrature table is used in setting or checking the expected state.
ElasticInput Patch(const Increment& g,double thickness=.1){
  ElasticInput in;in.thickness=thickness;
  const int sx[]={-1,1,1,-1},sy[]={-1,-1,1,1};
  for(int i=0;i<4;++i){
    const double x=sx[i]*in.half_x,y=sy[i]*in.half_y;
    in.velocity[i]={(g[0]*x+g[2]*y)/in.dt,g[1]*y/in.dt,0};
    in.angular_velocity[i]={(-g[6]*y-.5*g[7]*x-g[3])/in.dt,
                            (g[5]*x+.5*g[7]*y+g[4])/in.dt,0};
  }
  return in;
}

void AssertAnalytic(const ElasticInput& in,const Increment& g,
                    ThicknessRule rule,const ElasticResult& out){
  EXPECT_EQ(out.rule,rule);EXPECT_EQ(out.device_bytes,1248u);
  const double E=kElasticYoung,nu=kElasticNu,h=in.thickness;
  const double a=E/(1-nu*nu),b=nu*a,G=E/(2*(1+nu));
  const double area=4*in.half_x*in.half_y;
  const double ratio=rule==ThicknessRule::Gauss3?1:8.0/9.0;
  const double I=h*h*h/12*ratio;
  std::array<double,5> n{{h*(a*g[0]+b*g[1]),h*(b*g[0]+a*g[1]),h*G*g[2],
                         h*G*kElasticShearFactor*g[3],h*G*kElasticShearFactor*g[4]}};
  std::array<double,3> m{{I*(a*g[5]+b*g[6]),I*(b*g[5]+a*g[6]),I*G*g[7]}};
  for(int j=0;j<8;++j)Near(out.generalized_increment[j],g[j],2e-14);
  Near(out.area,area,1e-12);
  for(int j=0;j<5;++j){Near(out.forces[j],n[j],1e-6);Near(out.normalized_forces[j],n[j]/h,1e-5);}
  for(int j=0;j<3;++j){Near(out.moments[j],m[j],1e-7);Near(out.normalized_moments[j],m[j]/(h*h),1e-5);}
  const double e_mem=.5*area*(g[0]*n[0]+g[1]*n[1]+g[2]*n[2]+g[3]*n[3]+g[4]*n[4]);
  const double e_bend=.5*area*(g[5]*m[0]+g[6]*m[1]+g[7]*m[2]);
  Near(out.energy[0],e_mem,1e-8);Near(out.energy[1],e_bend,1e-8);
  Near(out.thickness,h*(1-nu/(1-nu)*(g[0]+g[1])),1e-13);
  EXPECT_EQ(out.reference_thickness,h);EXPECT_EQ(out.off,2);
  Near(out.mean_yield,kElasticYield,1e-3);
  for(int ip=0;ip<3;++ip){
    // Closed-form coordinates, independently derived from the named rule.
    const double z=(ip-1)*h*(rule==ThicknessRule::Gauss3?std::sqrt(3.0/5.0)/2:1.0/3.0);
    const double ex=g[0]+z*g[5],ey=g[1]+z*g[6],xy=g[2]+z*g[7];
    const std::array<double,5> stress{{a*ex+b*ey,b*ex+a*ey,G*xy,
                                      G*kElasticShearFactor*g[3],G*kElasticShearFactor*g[4]}};
    for(int j=0;j<5;++j)Near(out.points[ip].stress[j],stress[j],1e-5);
    EXPECT_EQ(out.points[ip].plastic_strain,0);EXPECT_EQ(out.points[ip].plastic_increment,0);
    EXPECT_EQ(out.points[ip].filtered_plastic_rate,0);EXPECT_EQ(out.points[ip].backstress,(std::array<double,3>{}));
    EXPECT_EQ(out.points[ip].temperature,kElasticTemperature);
  }
}

TEST(ShellElastic, AffineMembranePlaneStressAndThicknessBothRules){
  const Increment g{{2e-5,-1e-5,3e-5,0,0,0,0,0}};
  const ElasticInput in=Patch(g);
  for(auto rule:kRules){
    ElasticResult out;Attempt a=EvaluateElastic(in,rule,&out);
    ASSERT_EQ(a.status,Status::Ok)<<a.message;AssertAnalytic(in,g,rule,out);
  }
}

TEST(ShellElastic, CorrectedPureBendingUsesPhysicalPlateStiffness){
  Increment g{};g[5]=2e-3;ElasticInput in=Patch(g);
  // Corner samples of -kdot*x^2/2 are a uniform translation. This is a
  // rotational curvature patch, not a claim of Q4 quadratic-deflection accuracy.
  for(auto& v:in.velocity)v.z=-.5*g[5]*in.half_x*in.half_x/in.dt;
  ElasticResult out;Attempt a=EvaluateElastic(in,ThicknessRule::Gauss3,&out);
  ASSERT_EQ(a.status,Status::Ok)<<a.message;AssertAnalytic(in,g,ThicknessRule::Gauss3,out);
  const double D=kElasticYoung*std::pow(in.thickness,3)/(12*(1-kElasticNu*kElasticNu));
  Near(out.moments[0],D*g[5]);Near(out.moments[1],kElasticNu*D*g[5]);
  Near(out.energy[1],.5*(4*in.half_x*in.half_y)*D*g[5]*g[5]);
  for(int j=0;j<5;++j)Near(out.forces[j],0,1e-6);
}

TEST(ShellElastic, OriginalMidpointBendingDefectIsExactlyEightNinths){
  Increment g{};g[5]=2e-3;const ElasticInput in=Patch(g);
  ElasticResult original,gauss;
  Attempt a=EvaluateElastic(in,ThicknessRule::OriginalMidpoint3,&original);ASSERT_EQ(a.status,Status::Ok)<<a.message;
  a=EvaluateElastic(in,ThicknessRule::Gauss3,&gauss);ASSERT_EQ(a.status,Status::Ok)<<a.message;
  AssertAnalytic(in,g,ThicknessRule::OriginalMidpoint3,original);
  AssertAnalytic(in,g,ThicknessRule::Gauss3,gauss);
  EXPECT_NEAR(original.moments[0]/gauss.moments[0],8.0/9.0,1e-12);
  EXPECT_NEAR(original.energy[1]/gauss.energy[1],8.0/9.0,1e-12);
  EXPECT_GT(std::abs(original.moments[0]-gauss.moments[0]),.1*std::abs(gauss.moments[0]));
}

TEST(ShellElastic, BiaxialCurvatureAndEngineeringTwist){
  for(const Increment g:{Increment{{0,0,0,0,0,8e-4,-3e-4,0}},Increment{{0,0,0,0,0,0,0,1e-3}}}){
    const ElasticInput in=Patch(g);
    for(auto rule:kRules){ElasticResult out;Attempt a=EvaluateElastic(in,rule,&out);
      ASSERT_EQ(a.status,Status::Ok)<<a.message;AssertAnalytic(in,g,rule,out);}
  }
}

TEST(ShellElastic, TransverseShearFromMeanRotationHasNoQuadraticMembraneTerm){
  const Increment g{{0,0,0,2e-5,-3e-5,0,0,0}};const ElasticInput in=Patch(g);
  for(auto rule:kRules){ElasticResult out;Attempt a=EvaluateElastic(in,rule,&out);
    ASSERT_EQ(a.status,Status::Ok)<<a.message;AssertAnalytic(in,g,rule,out);}
}

TEST(ShellElastic, MixedIncrementHasIndependentMembraneAndBendingEnergy){
  const Increment g{{2e-5,-1e-5,3e-5,1e-5,-2e-5,7e-4,-4e-4,3e-4}};
  const ElasticInput in=Patch(g);
  for(auto rule:kRules){ElasticResult out;Attempt a=EvaluateElastic(in,rule,&out);
    ASSERT_EQ(a.status,Status::Ok)<<a.message;AssertAnalytic(in,g,rule,out);
    EXPECT_GT(out.energy[0],0);EXPECT_GT(out.energy[1],0);}
}

TEST(ShellElastic, CurvatureReversalChangesMomentAndPreservesEnergy){
  Increment g{{0,0,0,0,0,8e-4,-3e-4,5e-4}};
  for(auto rule:kRules){
    ElasticResult positive,negative;
    Attempt a=EvaluateElastic(Patch(g),rule,&positive);ASSERT_EQ(a.status,Status::Ok)<<a.message;
    Increment reverse=g;for(auto& x:reverse)x=-x;
    a=EvaluateElastic(Patch(reverse),rule,&negative);ASSERT_EQ(a.status,Status::Ok)<<a.message;
    for(int j=0;j<3;++j)Near(negative.moments[j],-positive.moments[j]);
    Near(negative.energy[1],positive.energy[1]);
    for(int j=0;j<5;++j){Near(negative.forces[j],0,1e-6);Near(positive.forces[j],0,1e-6);}
  }
}

TEST(ShellElastic, ThicknessScalingIsLinearMembraneAndCubicBending){
  const Increment g{{2e-5,-1e-5,3e-5,0,0,8e-4,-3e-4,5e-4}};
  for(auto rule:kRules){
    ElasticResult thin,thick;
    Attempt a=EvaluateElastic(Patch(g,.05),rule,&thin);ASSERT_EQ(a.status,Status::Ok)<<a.message;
    a=EvaluateElastic(Patch(g,.1),rule,&thick);ASSERT_EQ(a.status,Status::Ok)<<a.message;
    for(int j=0;j<3;++j){Near(thick.forces[j],2*thin.forces[j],1e-6);Near(thick.moments[j],8*thin.moments[j]);}
    Near(thick.energy[0],2*thin.energy[0]);Near(thick.energy[1],8*thin.energy[1]);
  }
}

TEST(ShellElastic, InPlaneInstantaneousSpinHasZeroStressAndEnergy){
  ElasticInput in;const int sx[]={-1,1,1,-1},sy[]={-1,-1,1,1};
  for(int i=0;i<4;++i){
    Vec3 p{sx[i]*in.half_x,sy[i]*in.half_y,0};
    in.velocity[i]=Cross({0,0,.002},p);in.angular_velocity[i]={0,0,.002};
  }
  for(auto rule:kRules){ElasticResult out;Attempt a=EvaluateElastic(in,rule,&out);
    ASSERT_EQ(a.status,Status::Ok)<<a.message;AssertAnalytic(in,Increment{},rule,out);}
}

Vec3 Rotate(Vec3 p){
  const double a=.53,b=-.37;
  Vec3 q{std::cos(a)*p.x-std::sin(a)*p.y,std::sin(a)*p.x+std::cos(a)*p.y,p.z};
  return {q.x,std::cos(b)*q.y-std::sin(b)*q.z,std::sin(b)*q.y+std::cos(b)*q.z};
}
Tensor RotateTensor(const Tensor& t){
  const std::array<Vec3,3> r{{Rotate({1,0,0}),Rotate({0,1,0}),Rotate({0,0,1})}};
  Tensor out{};
  for(int i=0;i<3;++i)for(int j=0;j<3;++j){
    const double u[]={r[i].x,r[i].y,r[i].z},v[]={r[j].x,r[j].y,r[j].z};
    for(int a=0;a<3;++a)for(int b=0;b<3;++b)out[3*a+b]+=t[3*i+j]*u[a]*v[b];
  }
  return out;
}

TEST(ShellElastic, ProperThreeDimensionalCoordinateCovariance){
  const Increment g{{2e-5,-1e-5,3e-5,1e-5,-2e-5,7e-4,-4e-4,3e-4}};
  const ElasticInput base=Patch(g);ElasticInput moved=base;
  moved.center={2,-3,4};for(auto& e:moved.basis)e=Rotate(e);
  for(auto& v:moved.velocity)v=Rotate(v);for(auto& v:moved.angular_velocity)v=Rotate(v);
  for(auto rule:kRules){
    ElasticResult before,after;Attempt a=EvaluateElastic(base,rule,&before);ASSERT_EQ(a.status,Status::Ok)<<a.message;
    a=EvaluateElastic(moved,rule,&after);ASSERT_EQ(a.status,Status::Ok)<<a.message;
    AssertAnalytic(moved,g,rule,after);
    const Tensor n=RotateTensor(before.membrane_world),m=RotateTensor(before.bending_world);
    for(int i=0;i<9;++i){Near(after.membrane_world[i],n[i],1e-6);Near(after.bending_world[i],m[i],1e-7);}
    Near(after.shear_world,Rotate(before.shear_world),1e-6);
    Near(after.energy[0],before.energy[0]);Near(after.energy[1],before.energy[1]);
  }
}

TEST(ShellElastic, MaterialWorldResultantsConnectToExistingK3BoundaryTraction){
  const Increment g{{2e-5,-1e-5,3e-5,1e-5,-2e-5,7e-4,-4e-4,3e-4}};
  ElasticInput in=Patch(g);in.center={2,-3,4};
  for(auto& e:in.basis)e=Rotate(e);
  for(auto& v:in.velocity)v=Rotate(v);for(auto& v:in.angular_velocity)v=Rotate(v);
  const double E=kElasticYoung,nu=kElasticNu,h=in.thickness;
  const double A=E/(1-nu*nu),B=nu*A,G=E/(2*(1+nu));
  const double nx=h*(A*g[0]+B*g[1]),ny=h*(B*g[0]+A*g[1]),nxy=h*G*g[2];
  const double qx=h*kElasticShearFactor*G*g[4],qy=h*kElasticShearFactor*G*g[3];
  const double area=4*in.half_x*in.half_y;
  const int sx[]={-1,1,1,-1},sy[]={-1,-1,1,1};
  // Closed-form integrated nodal shape gradients for a centered rectangle,
  // obtained from the adjacent straight boundary edge normals.
  const double bx[]={-in.half_y,in.half_y,in.half_y,-in.half_y};
  const double by[]={-in.half_x,-in.half_x,in.half_x,in.half_x};
  for(auto rule:kRules){
    ElasticResult material;Attempt a=EvaluateElastic(in,rule,&material);
    ASSERT_EQ(a.status,Status::Ok)<<a.message;
    Input assembly;
    for(int i=0;i<4;++i){
      assembly.positions.push_back(Add(in.center,ToWorld(in,{sx[i]*in.half_x,sy[i]*in.half_y,0})));
      assembly.velocity.push_back(in.velocity[i]);assembly.angular_velocity.push_back(in.angular_velocity[i]);
    }
    Element element;element.nodes={0,1,2,3};element.thickness=material.reference_thickness;
    element.membrane=material.membrane_world;element.bending=material.bending_world;element.shear=material.shear_world;
    assembly.elements.push_back(element);
    Result out;a=Evaluate(assembly,&out);ASSERT_EQ(a.status,Status::Ok)<<a.message;
    const double I=h*h*h/12*(rule==ThicknessRule::Gauss3?1:8.0/9.0);
    const double mx=I*(A*g[5]+B*g[6]),my=I*(B*g[5]+A*g[6]),mxy=I*G*g[7];
    Vec3 total_force{},total_moment{};
    for(int i=0;i<4;++i){
      const Vec3 local_force{-nx*bx[i]-nxy*by[i],-nxy*bx[i]-ny*by[i],-qx*bx[i]-qy*by[i]};
      const Vec3 local_couple{mxy*bx[i]+my*by[i]+area*qy/4,-mx*bx[i]-mxy*by[i]-area*qx/4,0};
      Near(out.forces[i],ToWorld(in,local_force),1e-6);Near(out.moments[i],ToWorld(in,local_couple),1e-6);
      total_force=Add(total_force,out.forces[i]);
      total_moment=Add(total_moment,Add(Cross(assembly.positions[i],out.forces[i]),out.moments[i]));
    }
    Near(total_force,{},1e-6);Near(total_moment,{},1e-5);
  }
}

void Same(const ElasticResult& a,const ElasticResult& b){
  EXPECT_EQ(a.rule,b.rule);EXPECT_EQ(a.generalized_increment,b.generalized_increment);
  EXPECT_EQ(a.forces,b.forces);EXPECT_EQ(a.moments,b.moments);
  EXPECT_EQ(a.normalized_forces,b.normalized_forces);EXPECT_EQ(a.normalized_moments,b.normalized_moments);
  EXPECT_EQ(a.membrane_world,b.membrane_world);EXPECT_EQ(a.bending_world,b.bending_world);
  EXPECT_EQ(a.area,b.area);EXPECT_EQ(a.thickness,b.thickness);EXPECT_EQ(a.reference_thickness,b.reference_thickness);
  EXPECT_EQ(a.off,b.off);EXPECT_EQ(a.mean_yield,b.mean_yield);EXPECT_EQ(a.element_rate,b.element_rate);
  EXPECT_EQ(a.energy,b.energy);EXPECT_EQ(a.device_bytes,b.device_bytes);
  for(int j=0;j<3;++j){
    EXPECT_EQ(a.frame[j].x,b.frame[j].x);EXPECT_EQ(a.frame[j].y,b.frame[j].y);EXPECT_EQ(a.frame[j].z,b.frame[j].z);
    EXPECT_EQ(a.points[j].stress,b.points[j].stress);EXPECT_EQ(a.points[j].backstress,b.points[j].backstress);
    EXPECT_EQ(a.points[j].plastic_strain,b.points[j].plastic_strain);
    EXPECT_EQ(a.points[j].plastic_increment,b.points[j].plastic_increment);
    EXPECT_EQ(a.points[j].filtered_plastic_rate,b.points[j].filtered_plastic_rate);
    EXPECT_EQ(a.points[j].temperature,b.points[j].temperature);
  }
  EXPECT_EQ(a.shear_world.x,b.shear_world.x);EXPECT_EQ(a.shear_world.y,b.shear_world.y);EXPECT_EQ(a.shear_world.z,b.shear_world.z);
}

TEST(ShellElasticValidation, InvalidDomainsYieldBoundAndBudgetDoNotPublish){
  ElasticResult committed;committed.thickness=123;committed.points[2].stress[0]=456;
  const ElasticResult before=committed;
  for(auto rule:kRules){
    EXPECT_EQ(EvaluateElastic(ElasticInput{},rule,nullptr).status,Status::InvalidInput);
    for(int kind=0;kind<7;++kind){
      ElasticInput in;Status expected=Status::InvalidInput;
      if(kind==0)in.dt=0;
      if(kind==1)in.thickness=-1;
      if(kind==2){in.half_x=0;expected=Status::UnsupportedGeometry;}
      if(kind==3)in.velocity[0].x=std::numeric_limits<double>::quiet_NaN();
      if(kind==4)in.basis[2].z=-1;
      if(kind==5)in.velocity[0].x=1e8;
      if(kind==6)in.dt=std::numeric_limits<double>::infinity();
      EXPECT_EQ(EvaluateElastic(in,rule,&committed).status,expected);Same(committed,before);
    }
    Options limited;limited.max_device_bytes=1247;
    EXPECT_EQ(EvaluateElastic(ElasticInput{},rule,&committed,limited).status,Status::ResourceLimit);Same(committed,before);
  }
  EXPECT_EQ(EvaluateElastic(ElasticInput{},static_cast<ThicknessRule>(99),&committed).status,Status::InvalidInput);Same(committed,before);
}

TEST(ShellElasticTransaction, RejectPreservesCompleteResultThenRetryUsesFreshRulePoints){
  Increment g{{2e-5,-1e-5,3e-5,0,0,8e-4,-3e-4,5e-4}};
  const ElasticInput in=Patch(g);ElasticResult committed;
  Attempt a=EvaluateElastic(in,ThicknessRule::OriginalMidpoint3,&committed);ASSERT_EQ(a.status,Status::Ok)<<a.message;
  const ElasticResult before=committed;
  Options reject;reject.reject_after_assembly=true;
  a=EvaluateElastic(in,ThicknessRule::Gauss3,&committed,reject);ASSERT_EQ(a.status,Status::TrialRejected)<<a.message;
  Same(committed,before);
  a=EvaluateElastic(in,ThicknessRule::Gauss3,&committed);ASSERT_EQ(a.status,Status::Ok)<<a.message;
  AssertAnalytic(in,g,ThicknessRule::Gauss3,committed);
  EXPECT_NE(committed.points[0].stress[0],before.points[0].stress[0]);
  ElasticResult fresh;a=EvaluateElastic(in,ThicknessRule::Gauss3,&fresh);ASSERT_EQ(a.status,Status::Ok)<<a.message;
  Same(committed,fresh);
  // Named fields, not raw object bytes/padding: the supplied input is unchanged.
  const ElasticInput expected=Patch(g);
  EXPECT_EQ(in.dt,expected.dt);EXPECT_EQ(in.thickness,expected.thickness);
  for(int i=0;i<4;++i){
    EXPECT_EQ(in.velocity[i].x,expected.velocity[i].x);EXPECT_EQ(in.velocity[i].y,expected.velocity[i].y);EXPECT_EQ(in.velocity[i].z,expected.velocity[i].z);
    EXPECT_EQ(in.angular_velocity[i].x,expected.angular_velocity[i].x);EXPECT_EQ(in.angular_velocity[i].y,expected.angular_velocity[i].y);EXPECT_EQ(in.angular_velocity[i].z,expected.angular_velocity[i].z);
  }
}

}  // namespace
}  // namespace shell_spike
