#include "ShellFixture.h"

#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <limits>

namespace shell_spike {
namespace {

Vec3 Add(Vec3 a, Vec3 b) { return {a.x+b.x,a.y+b.y,a.z+b.z}; }
Vec3 Sub(Vec3 a, Vec3 b) { return {a.x-b.x,a.y-b.y,a.z-b.z}; }
Vec3 Mul(Vec3 a, double s) { return {a.x*s,a.y*s,a.z*s}; }
double Dot(Vec3 a, Vec3 b) { return a.x*b.x+a.y*b.y+a.z*b.z; }
Vec3 Cross(Vec3 a, Vec3 b) { return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x}; }
double Norm(Vec3 a) { return std::hypot(a.x,a.y,a.z); }
Vec3 Apply(const Tensor& a,Vec3 b) {
  return {a[0]*b.x+a[1]*b.y+a[2]*b.z,a[3]*b.x+a[4]*b.y+a[5]*b.z,a[6]*b.x+a[7]*b.y+a[8]*b.z};
}
Tensor PlaneTensor(double xx,double yy,double xy) { return {xx,xy,0,xy,yy,0,0,0,0}; }

Input Square() {
  Input in;
  in.positions={{0,0,0},{1,0,0},{1,1,0},{0,1,0}};
  in.velocity.resize(4); in.angular_velocity.resize(4);
  Element e; e.nodes={0,1,2,3}; e.thickness=0.2;
  e.membrane=PlaneTensor(2,3,1);
  in.elements.push_back(e);
  return in;
}
Input CombinedSquare() {
  Input in=Square();
  in.elements[0].bending=PlaneTensor(6,7,8);
  in.elements[0].shear={4,5,0};
  return in;
}

void Near(Vec3 a,Vec3 b,double tolerance=2e-10) {
  EXPECT_NEAR(a.x,b.x,tolerance); EXPECT_NEAR(a.y,b.y,tolerance); EXPECT_NEAR(a.z,b.z,tolerance);
}
void Exactly(Vec3 a,Vec3 b) {
  EXPECT_EQ(a.x,b.x); EXPECT_EQ(a.y,b.y); EXPECT_EQ(a.z,b.z);
}
void SameResult(const Result& a,const Result& b) {
  ASSERT_EQ(a.forces.size(),b.forces.size()); ASSERT_EQ(a.moments.size(),b.moments.size());
  for (std::size_t i=0;i<a.forces.size();++i) Exactly(a.forces[i],b.forces[i]);
  for (std::size_t i=0;i<a.moments.size();++i) Exactly(a.moments[i],b.moments[i]);
  ASSERT_EQ(a.frames.size(),b.frames.size());
  for (std::size_t i=0;i<a.frames.size();++i)
    for (int j=0;j<3;++j) Exactly(a.frames[i][j],b.frames[i][j]);
  EXPECT_EQ(a.areas,b.areas); EXPECT_EQ(a.off,b.off);
  EXPECT_EQ(a.reference_coordinates,b.reference_coordinates);
  EXPECT_EQ(a.hourglass,b.hourglass); EXPECT_EQ(a.energy,b.energy);
  EXPECT_EQ(a.device_bytes,b.device_bytes);
}
void SameInput(const Input& a,const Input& b) {
  ASSERT_EQ(a.positions.size(),b.positions.size());
  ASSERT_EQ(a.elements.size(),b.elements.size());
  for (std::size_t i=0;i<a.positions.size();++i) {
    Exactly(a.positions[i],b.positions[i]); Exactly(a.velocity[i],b.velocity[i]);
    Exactly(a.angular_velocity[i],b.angular_velocity[i]);
  }
  for (std::size_t i=0;i<a.elements.size();++i) {
    EXPECT_EQ(a.elements[i].nodes,b.elements[i].nodes);
    EXPECT_EQ(a.elements[i].thickness,b.elements[i].thickness);
    EXPECT_EQ(a.elements[i].membrane,b.elements[i].membrane);
    EXPECT_EQ(a.elements[i].bending,b.elements[i].bending);
    Exactly(a.elements[i].shear,b.elements[i].shear);
  }
}

// Independent divergence-theorem oracle: constant edge traction is integrated
// along each straight edge, with half assigned to each endpoint. No donor
// PX/PY, convected-frame construction, or element-kernel arithmetic is used.
// Bending uses the same boundary integral of shape gradients. The shear couple
// A/4 is used only on parallelograms, where it also equals the exact integral.
Result BoundaryOracle(const Input& in) {
  Result expected;
  expected.forces.resize(in.positions.size()); expected.moments.resize(in.positions.size());
  for (const Element& e:in.elements) {
    std::array<Vec3,4> p{};
    for (int i=0;i<4;++i) p[i]=in.positions[e.nodes[i]];
    Vec3 normal=Cross(Sub(p[1],p[0]),Sub(p[2],p[0]));
    normal=Mul(normal,1/Norm(normal));
    std::array<Vec3,4> integrated_grad{};
    double area=0;
    for (int i=0;i<4;++i) {
      const int j=(i+1)%4;
      Vec3 edge_normal=Cross(Sub(p[j],p[i]),normal);
      integrated_grad[i]=Add(integrated_grad[i],Mul(edge_normal,0.5));
      integrated_grad[j]=Add(integrated_grad[j],Mul(edge_normal,0.5));
      area+=0.5*Dot(Cross(Sub(p[i],p[0]),Sub(p[j],p[0])),normal);
    }
    for (int i=0;i<4;++i) {
      Vec3 b=integrated_grad[i];
      Vec3 f=Sub(Mul(Apply(e.membrane,b),-1),Mul(normal,Dot(e.shear,b)));
      Vec3 m=Mul(Cross(normal,Add(Apply(e.bending,b),Mul(e.shear,area/4))),-1);
      expected.forces[e.nodes[i]]=Add(expected.forces[e.nodes[i]],f);
      expected.moments[e.nodes[i]]=Add(expected.moments[e.nodes[i]],m);
    }
  }
  return expected;
}
void MatchesBoundary(const Input& in,const Result& actual) {
  Result expected=BoundaryOracle(in);
  ASSERT_EQ(actual.forces.size(),expected.forces.size());
  for (std::size_t i=0;i<actual.forces.size();++i) {
    Near(actual.forces[i],expected.forces[i]); Near(actual.moments[i],expected.moments[i]);
  }
}

TEST(ShellAssembly, SquareMembraneBoundaryTractionAndGeometry) {
  Input in=Square(); Result out;
  Attempt a=Evaluate(in,&out); ASSERT_EQ(a.status,Status::Ok)<<a.message;
  MatchesBoundary(in,out);
  const Vec3 expected[]={{1.5,2,0},{-.5,1,0},{-1.5,-2,0},{.5,-1,0}};
  for(int i=0;i<4;++i) Near(out.forces[i],expected[i]);
  ASSERT_EQ(out.areas.size(),1u); EXPECT_NEAR(out.areas[0],1,1e-12);
  Near(out.frames[0][0],{1,0,0}); Near(out.frames[0][1],{0,1,0}); Near(out.frames[0][2],{0,0,1});
  EXPECT_EQ(out.off,std::vector<double>{2});
  EXPECT_EQ(out.reference_coordinates,(std::vector<double>{1,0,1,1,0,1}));
}

TEST(ShellAssembly, IndividualMembraneBendingAndShearComponents) {
  for (int component=0;component<8;++component) {
    SCOPED_TRACE(component);
    Input in=Square(); in.elements[0].membrane={};
    auto& e=in.elements[0];
    if(component==0) e.membrane=PlaneTensor(2,0,0);
    if(component==1) e.membrane=PlaneTensor(0,3,0);
    if(component==2) e.membrane=PlaneTensor(0,0,1);
    if(component==3) e.bending=PlaneTensor(6,0,0);
    if(component==4) e.bending=PlaneTensor(0,7,0);
    if(component==5) e.bending=PlaneTensor(0,0,8);
    if(component==6) e.shear={4,0,0};
    if(component==7) e.shear={0,5,0};
    Result out; Attempt a=Evaluate(in,&out); ASSERT_EQ(a.status,Status::Ok)<<a.message;
    MatchesBoundary(in,out);
  }
}

TEST(ShellAssembly, CombinedResultantsMatchIndependentLiteralTable) {
  Input in=CombinedSquare(); Result out;
  Attempt a=Evaluate(in,&out); ASSERT_EQ(a.status,Status::Ok)<<a.message;
  const Vec3 f[]={{1.5,2,4.5},{-.5,1,.5},{-1.5,-2,-4.5},{.5,-1,-.5}};
  const Vec3 m[]={{-6.25,6,0},{1.75,0,0},{8.75,-8,0},{.75,-2,0}};
  for(int i=0;i<4;++i) {Near(out.forces[i],f[i]); Near(out.moments[i],m[i]);}
  Vec3 force{},moment{};
  const Vec3 omega{.3,-.7,.4},translation{2,-3,1};
  double rigid_work=0;
  for(int i=0;i<4;++i) {
    force=Add(force,out.forces[i]);
    moment=Add(moment,Add(Cross(in.positions[i],out.forces[i]),out.moments[i]));
    rigid_work+=Dot(out.forces[i],Add(translation,Cross(omega,in.positions[i])))+Dot(out.moments[i],omega);
  }
  Near(force,{}); Near(moment,{}); EXPECT_NEAR(rigid_work,0,1e-10);
}

TEST(ShellAssembly, SkewParallelogramWorldTensorMatchesBoundary) {
  Input in=CombinedSquare(); in.positions={{0,0,0},{2,0,0},{2.5,1,0},{.5,1,0}};
  Result out; Attempt a=Evaluate(in,&out); ASSERT_EQ(a.status,Status::Ok)<<a.message;
  MatchesBoundary(in,out); EXPECT_NEAR(out.areas[0],2,1e-12);
  const Vec3 f[]={{1.75,2.75,5.75},{.25,3.25,4.25},{-1.75,-2.75,-5.75},{-.25,-3.25,-4.25}};
  const Vec3 m[]={{-6.75,7,0},{-2.25,5,0},{11.75,-11,0},{7.25,-9,0}};
  for(int i=0;i<4;++i) {Near(out.forces[i],f[i]);Near(out.moments[i],m[i]);}
}

TEST(ShellAssembly, NonParallelogramMembraneMatchesExactBoundary) {
  Input in=Square(); in.positions={{0,0,0},{2,0,0},{1.7,1,0},{.2,1.2,0}};
  Result out; Attempt a=Evaluate(in,&out); ASSERT_EQ(a.status,Status::Ok)<<a.message;
  MatchesBoundary(in,out);
  // Reverse winding with fixed world membrane tensor; physical edge traction
  // and force remain unchanged when associated nodes are matched.
  in.elements[0].nodes={0,3,2,1};
  Result reversed; a=Evaluate(in,&reversed); ASSERT_EQ(a.status,Status::Ok)<<a.message;
  MatchesBoundary(in,reversed);
  for(int i=0;i<4;++i) Near(reversed.forces[i],out.forces[i]);
}

TEST(ShellAssembly, AffineMembraneAndBendingVirtualWork) {
  Input in=Square(); in.positions={{0,0,0},{2,0,0},{2.5,1,0},{.5,1,0}};
  in.elements[0].bending=PlaneTensor(6,7,8);
  const double alpha=.2,beta=-.1,gamma=.3,kxx=.4,kyy=-.2,kxy=.6;
  Result out; Attempt a=Evaluate(in,&out); ASSERT_EQ(a.status,Status::Ok)<<a.message;
  double work=0;
  for(int i=0;i<4;++i) {
    const double x=in.positions[i].x,y=in.positions[i].y;
    work+=Dot(out.forces[i],{alpha*x+gamma*y,beta*y,0});
    work+=Dot(out.moments[i],{-kyy*y-kxy*x/2,kxx*x+kxy*y/2,0});
  }
  const double exact=-2*(2*alpha+3*beta+1*gamma+6*kxx+7*kyy+8*kxy);
  EXPECT_NEAR(work,exact,2e-10);
}

// Proper non-axis-aligned rotation, independent of the donor frame algorithm.
Vec3 Rotate(Vec3 p) {
  const double a=.63,b=-.41;
  Vec3 q{std::cos(a)*p.x-std::sin(a)*p.y,std::sin(a)*p.x+std::cos(a)*p.y,p.z};
  return {q.x,std::cos(b)*q.y-std::sin(b)*q.z,std::sin(b)*q.y+std::cos(b)*q.z};
}
Tensor RotateTensor(const Tensor& a) {
  const std::array<Vec3,3> basis={Rotate({1,0,0}),Rotate({0,1,0}),Rotate({0,0,1})};
  Tensor result{};
  for(int i=0;i<3;++i) for(int j=0;j<3;++j) {
    const double r[]={basis[i].x,basis[i].y,basis[i].z};
    const double s[]={basis[j].x,basis[j].y,basis[j].z};
    for(int row=0;row<3;++row) for(int col=0;col<3;++col)
      result[3*row+col]+=a[3*i+j]*r[row]*s[col];
  }
  return result;
}

TEST(ShellAssembly, ProperRigidTransformCovariance) {
  Input in=CombinedSquare(); in.positions={{0,0,0},{2,0,0},{2.5,1,0},{.5,1,0}};
  Result base; Attempt a=Evaluate(in,&base); ASSERT_EQ(a.status,Status::Ok)<<a.message;
  for(auto& p:in.positions) p=Add(Rotate(p),{3,-4,5});
  in.elements[0].membrane=RotateTensor(in.elements[0].membrane);
  in.elements[0].bending=RotateTensor(in.elements[0].bending);
  in.elements[0].shear=Rotate(in.elements[0].shear);
  Result out; a=Evaluate(in,&out); ASSERT_EQ(a.status,Status::Ok)<<a.message;
  MatchesBoundary(in,out);
  for(int i=0;i<4;++i) {Near(out.forces[i],Rotate(base.forces[i]));Near(out.moments[i],Rotate(base.moments[i]));}
  for(int i=0;i<3;++i) Near(out.frames[0][i],Rotate(base.frames[0][i]));
}

TEST(ShellAssembly, FixedPhysicalResultantsIndependentOfThickness) {
  Input in=CombinedSquare(); Result base;
  Attempt a=Evaluate(in,&base); ASSERT_EQ(a.status,Status::Ok)<<a.message;
  for(double thickness:{.03,.8,2.0}) {
    in.elements[0].thickness=thickness; Result out;
    a=Evaluate(in,&out); ASSERT_EQ(a.status,Status::Ok)<<a.message;
    for(int i=0;i<4;++i) {Near(out.forces[i],base.forces[i]);Near(out.moments[i],base.moments[i]);}
  }
}

TEST(ShellAssembly, GeometryScalingMatchesBoundaryUnits) {
  Input base=CombinedSquare();
  for(double scale:{.1,1.0,10.0}) {
    Input in=base; for(auto& p:in.positions) p=Mul(p,scale);
    Result out; Attempt a=Evaluate(in,&out); ASSERT_EQ(a.status,Status::Ok)<<a.message;
    MatchesBoundary(in,out); EXPECT_NEAR(out.areas[0],scale*scale,1e-10);
  }
}

TEST(ShellAssembly, SharedEdgeAtomicAssemblyAndFreshOutputs) {
  Input in=Square();
  in.positions={{0,0,0},{1,0,0},{1,1,0},{0,1,0},{2,0,0},{2,1,0}};
  in.velocity.resize(6); in.angular_velocity.resize(6);
  Element second=in.elements[0]; second.nodes={1,4,5,2}; second.thickness=.7;
  in.elements.push_back(second);
  Result out; Attempt a=Evaluate(in,&out); ASSERT_EQ(a.status,Status::Ok)<<a.message;
  MatchesBoundary(in,out);
  Result previous=out;
  a=Evaluate(in,&out); ASSERT_EQ(a.status,Status::Ok)<<a.message;
  // Repetition must not double prior atomic accumulations.
  for(int i=0;i<6;++i) {Near(out.forces[i],previous.forces[i]);Near(out.moments[i],previous.moments[i]);}
}

TEST(ShellAssembly, SixteenDistinctElementsExerciseSoAStridesAndBudget) {
  Input in;
  for(int e=0;e<16;++e) {
    const int first=4*e;
    in.positions.insert(in.positions.end(),{{3.0*e,0,0},{3.0*e+1,0,0},{3.0*e+1,1,0},{3.0*e,1,0}});
    Element el; el.nodes={first,first+1,first+2,first+3}; el.thickness=.02*(e+1);
    el.membrane=PlaneTensor(e+1,2*e+3,.1*e); el.bending=PlaneTensor(.2*e,-.3*e,.4*e);
    el.shear={.3*e,-.7*e,0}; in.elements.push_back(el);
  }
  in.velocity.resize(64); in.angular_velocity.resize(64);
  Result out; Attempt a=Evaluate(in,&out); ASSERT_EQ(a.status,Status::Ok)<<a.message;
  MatchesBoundary(in,out);
  EXPECT_EQ(out.device_bytes,19584u); EXPECT_EQ(a.device_bytes,out.device_bytes);
  EXPECT_TRUE(std::all_of(out.hourglass.begin(),out.hourglass.end(),[](double x){return x==0;}));
  EXPECT_TRUE(std::all_of(out.energy.begin(),out.energy.end(),[](double x){return x==0;}));
}

TEST(ShellAssembly, PermutedSparseNodeIndicesAndUnusedNode) {
  Input in=CombinedSquare();
  in.positions={{99,10,0},{1,1,0},{0,0,0},{0,1,0},{1,0,0}};
  in.velocity.resize(5); in.angular_velocity.resize(5); in.elements[0].nodes={2,4,1,3};
  Result out; Attempt a=Evaluate(in,&out); ASSERT_EQ(a.status,Status::Ok)<<a.message;
  MatchesBoundary(in,out); Exactly(out.forces[0],{}); Exactly(out.moments[0],{});
}

TEST(ShellAssembly, RigidVelocitiesWithZeroResultantsHaveNoSpuriousForces) {
  Input in=Square(); in.elements[0].membrane={};
  for(int i=0;i<4;++i) {
    in.velocity[i]=Add({2,-3,4},Cross({.2,.3,-.1},in.positions[i]));
    in.angular_velocity[i]={.2,.3,-.1};
  }
  Result out; Attempt a=Evaluate(in,&out); ASSERT_EQ(a.status,Status::Ok)<<a.message;
  for(int i=0;i<4;++i) {Exactly(out.forces[i],{});Exactly(out.moments[i],{});}
  EXPECT_TRUE(std::all_of(out.hourglass.begin(),out.hourglass.end(),[](double x){return x==0;}));
  EXPECT_TRUE(std::all_of(out.energy.begin(),out.energy.end(),[](double x){return x==0;}));
}

TEST(ShellValidation, EmptyNullAndBudgetDoNotPublishResults) {
  Result committed; committed.forces={{7,8,9}}; committed.device_bytes=123;
  const Result before=committed;
  EXPECT_EQ(Evaluate(Input{},&committed).status,Status::EmptyInput); SameResult(committed,before);
  EXPECT_EQ(Evaluate(Square(),nullptr).status,Status::InvalidInput);
  Options limit; limit.max_device_bytes=1;
  EXPECT_EQ(Evaluate(Square(),&committed,limit).status,Status::ResourceLimit); SameResult(committed,before);
  Input too_many=Square(); too_many.elements.resize(17,too_many.elements[0]);
  EXPECT_EQ(Evaluate(too_many,&committed).status,Status::ResourceLimit); SameResult(committed,before);
}

TEST(ShellValidation, InvalidInputsAndUnsupportedGeometryDoNotPublish) {
  Result committed; committed.forces={{7,8,9}}; const Result before=committed;
  for(int kind=0;kind<11;++kind) {
    SCOPED_TRACE(kind); Input in=Square(); Status expected=Status::InvalidInput;
    if(kind==0) in.elements[0].nodes[2]=4;
    if(kind==1) in.elements[0].nodes[0]=-1;
    if(kind==2) {in.elements[0].nodes[3]=2;expected=Status::UnsupportedGeometry;}
    if(kind==3) in.elements[0].thickness=0;
    if(kind==4) in.positions[0].x=std::numeric_limits<double>::quiet_NaN();
    if(kind==5) in.velocity.pop_back();
    if(kind==6) {in.positions[3].z=.1;expected=Status::UnsupportedGeometry;}
    if(kind==7) {in.elements[0].nodes={0,2,1,3};expected=Status::UnsupportedGeometry;}
    if(kind==8) {in.positions[2]={.2,.2,0};expected=Status::UnsupportedGeometry;}
    if(kind==9) in.elements[0].membrane[2]=1;
    if(kind==10) in.elements[0].shear.z=1;
    EXPECT_EQ(Evaluate(in,&committed).status,expected); SameResult(committed,before);
  }
}

TEST(ShellTransaction, RejectedGpuTrialPreservesInputAndCommittedStateThenRetries) {
  Input in=CombinedSquare(); Result committed;
  Attempt a=Evaluate(in,&committed); ASSERT_EQ(a.status,Status::Ok)<<a.message;
  const Result before=committed;
  in.elements[0].membrane=PlaneTensor(9,8,7);
  in.velocity[1]={.3,-.7,.2}; in.angular_velocity[2]={.1,.2,.3};
  const Input input_before=in;
  Options reject; reject.reject_after_assembly=true;
  a=Evaluate(in,&committed,reject); ASSERT_EQ(a.status,Status::TrialRejected)<<a.message;
  SameInput(in,input_before); SameResult(committed,before);
  a=Evaluate(in,&committed); ASSERT_EQ(a.status,Status::Ok)<<a.message;
  SameInput(in,input_before); MatchesBoundary(in,committed);
  EXPECT_NE(committed.forces[0].x,before.forces[0].x);
}

}  // namespace
}  // namespace shell_spike
