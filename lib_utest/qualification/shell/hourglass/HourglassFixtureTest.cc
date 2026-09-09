#include "HourglassFixture.h"
#include "HourglassNative.h"
#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <limits>

namespace shell_hourglass {
namespace {
Vec3 Add(Vec3 a,Vec3 b){return {a.x+b.x,a.y+b.y,a.z+b.z};}
Vec3 Sub(Vec3 a,Vec3 b){return {a.x-b.x,a.y-b.y,a.z-b.z};}
Vec3 Mul(Vec3 a,double s){return {a.x*s,a.y*s,a.z*s};}
double Dot(Vec3 a,Vec3 b){return a.x*b.x+a.y*b.y+a.z*b.z;}
Vec3 Cross(Vec3 a,Vec3 b){return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};}
double Norm(Vec3 a){return std::hypot(a.x,a.y,a.z);}
void Near(double a,double b,double eps=2e-10){EXPECT_NEAR(a,b,eps+2e-10*std::abs(b));}
void Near(Vec3 a,Vec3 b,double eps=2e-10){Near(a.x,b.x,eps);Near(a.y,b.y,eps);Near(a.z,b.z,eps);}
struct Motion {
  std::vector<Vec3> x,v,w;
  BorrowedKinematics view()const{return {x.data(),v.data(),w.data(),x.size()};}
};
Motion Rectangle(){return {{{-1,-.75,0},{1,-.75,0},{1,.75,0},{-1,.75,0}},std::vector<Vec3>(4),std::vector<Vec3>(4)};}
Motion Distorted(){return {{{0,0,0},{2,0,0},{1.7,1,0},{.2,1.2,0}},std::vector<Vec3>(4),std::vector<Vec3>(4)};}
Configuration Config(Mode mode=Mode::UniformRectangle){
  Configuration c;c.mode=mode;c.node_count=4;Element e;e.nodes={0,1,2,3};c.elements.push_back(e);return c;
}
Parameters Linear(){Parameters p;p.SRH1=.2;p.SRH2=.3;p.SRH3=.4;p.HVLIN=.5;return p;}
Parameters Elastic(){Parameters p;p.H1=.2;p.H2=.3;p.HELAS=.5;return p;}
void Pure(Motion& m,int mode,double amplitude){
  m.v.assign(m.x.size(),{});m.w.assign(m.x.size(),{});
  const int sign[]={1,-1,1,-1};
  for(int i=0;i<4;++i){
    Vec3 value{};if(mode==0||mode==3)value.x=amplitude*sign[i];
    if(mode==1||mode==4)value.y=amplitude*sign[i];if(mode==2)value.z=amplitude*sign[i];
    if(mode<3)m.v[i]=value;else m.w[i]=value;
  }
}

// Native CHVIS3 comparison deliberately uses the same K1 geometric inputs.
// Independent modal/nullspace/work tests below separately check physical
// invariants, so native agreement is not the sole expected-result oracle.
void CompareNative(const State& state,const Motion& in,double dt,const Snapshot& out){
  const auto& config=state.configuration();const auto& accepted=state.accepted();
  const int n=static_cast<int>(config.elements.size());
  std::vector<double> fields(20*n),v(12*n),w(12*n),old(5*n),hour(5*n),f(12*n),m(12*n),work(n);
  const auto& p=config.parameters;const double controls[]={p.HVISC,p.HVLIN,p.HELAS};
  for(int e=0;e<n;++e){
    const auto& a=out.elements[e];const auto& el=config.elements[e];
    const double values[]={a.px1,a.px2,a.py1,a.py2,a.area,a.vhx,a.vhy,el.thickness,
      el.young,el.nu,el.density,el.sound_speed,el.shear_factor,p.H1,p.H2,p.H3,p.SRH1,p.SRH2,p.SRH3,dt};
    for(int j=0;j<20;++j)fields[j*n+e]=values[j];
    for(int j=0;j<5;++j)old[j*n+e]=accepted.elements[e].hour[j];
    for(int node=0;node<4;++node)for(int xyz=0;xyz<3;++xyz){
      v[(3*node+xyz)*n+e]=Dot(a.frame[xyz],in.v[el.nodes[node]]);
      w[(3*node+xyz)*n+e]=Dot(a.frame[xyz],in.w[el.nodes[node]]);
    }
  }
  ASSERT_EQ(crash_chvis3_native(n,static_cast<int>(config.mode),controls,fields.data(),v.data(),w.data(),
    old.data(),hour.data(),f.data(),m.data(),work.data()),0);
  std::vector<Vec3> total_f(config.node_count),total_m(config.node_count);
  for(int e=0;e<n;++e){
    const auto& a=out.elements[e];
    for(int j=0;j<5;++j)Near(a.hour[j],hour[j*n+e]);
    Near(a.energy[0],accepted.elements[e].energy[0]);
    Near(a.energy[1]-accepted.elements[e].energy[1],work[e]);
    for(int node=0;node<4;++node){
      Vec3 ff{},mm{};
      for(int xyz=0;xyz<3;++xyz){ff=Add(ff,Mul(a.frame[xyz],f[(3*node+xyz)*n+e]));mm=Add(mm,Mul(a.frame[xyz],m[(3*node+xyz)*n+e]));}
      int id=config.elements[e].nodes[node];total_f[id]=Add(total_f[id],ff);total_m[id]=Add(total_m[id],mm);
    }
  }
  for(std::size_t i=0;i<config.node_count;++i){Near(out.forces[i],total_f[i]);Near(out.moments[i],total_m[i]);}
}

void EvaluateChecked(const State& state,const Motion& in,double dt,Trial* trial){
  Report r=EvaluateTrial(state,in.view(),dt,trial);ASSERT_EQ(r.status,Status::Ok)<<r.message;
  ASSERT_TRUE(trial->valid());CompareNative(state,in,dt,trial->candidate());
  double power=0,increment=0;
  for(std::size_t i=0;i<in.x.size();++i)power+=Dot(trial->candidate().forces[i],in.v[i])+Dot(trial->candidate().moments[i],in.w[i]);
  for(std::size_t i=0;i<state.accepted().elements.size();++i)
    increment+=trial->candidate().elements[i].energy[1]-state.accepted().elements[i].energy[1];
  Near(increment,-dt*power,1e-9); // endpoint-force work identity, not conservation.
}
void Balance(const Motion& in,const Snapshot& s){
  Vec3 f{},m{};for(std::size_t i=0;i<in.x.size();++i){f=Add(f,s.forces[i]);m=Add(m,Add(Cross(in.x[i],s.forces[i]),s.moments[i]));}
  Near(f,{});Near(m,{});
}

TEST(Hourglass, ZeroMotionAndZeroHistoryAllModes){
  for(Mode mode:{Mode::UniformRectangle,Mode::CorrectedPlanar}){
    auto c=Config(mode);c.parameters=Linear();c.parameters.HELAS=.5;c.parameters.H1=.2;c.parameters.H2=.3;
    State s;ASSERT_EQ(Initialize(c,&s).status,Status::Ok);Motion m=mode==Mode::UniformRectangle?Rectangle():Distorted();
    Trial t;EvaluateChecked(s,m,.01,&t);ASSERT_TRUE(t.valid());
    for(auto f:t.candidate().forces)Near(f,{});for(auto f:t.candidate().moments)Near(f,{});
    for(auto h:t.candidate().elements[0].hour)EXPECT_EQ(h,0);
    EXPECT_EQ(t.candidate().elements[0].energy[1],0);
  }
}

TEST(Hourglass, FiveIndependentRectangleModesHaveRestoringSignAndBalance){
  const int sign[]={1,-1,1,-1};
  for(int mode=0;mode<5;++mode){
    SCOPED_TRACE(mode);auto c=Config();c.parameters=Linear();State s;ASSERT_EQ(Initialize(c,&s).status,Status::Ok);
    Motion m=Rectangle();Pure(m,mode,.2);Trial t;EvaluateChecked(s,m,.01,&t);ASSERT_TRUE(t.valid());
    Balance(m,t.candidate());double power=0;
    const Vec3 first=mode<3?t.candidate().forces[0]:t.candidate().moments[0];
    EXPECT_GT(Norm(first),0);
    for(int i=0;i<4;++i){
      const Vec3 response=mode<3?t.candidate().forces[i]:t.candidate().moments[i];
      Near(response,Mul(first,sign[i]));
      power+=Dot(t.candidate().forces[i],m.v[i])+Dot(t.candidate().moments[i],m.w[i]);
    }
    EXPECT_LT(power,0);EXPECT_GT(t.candidate().elements[0].energy[1],0);
  }
}

TEST(Hourglass, LinearAndQuadraticViscousScalingAndVelocityReversal){
  for(bool quadratic:{false,true}){
    auto c=Config();c.parameters=quadratic?Parameters{}:Linear();
    if(quadratic){c.parameters.H1=.2;c.parameters.HVISC=.5;}
    State s;ASSERT_EQ(Initialize(c,&s).status,Status::Ok);Motion m=Rectangle();
    Pure(m,0,.2);Trial base;EvaluateChecked(s,m,.01,&base);ASSERT_TRUE(base.valid());
    for(double scale:{-1.0,2.0}){
      Pure(m,0,.2*scale);Trial t;EvaluateChecked(s,m,.01,&t);ASSERT_TRUE(t.valid());
      const double factor=quadratic?scale*std::abs(scale):scale;
      for(int i=0;i<4;++i)Near(t.candidate().forces[i],Mul(base.candidate().forces[i],factor));
    }
  }
}

TEST(Hourglass, TransverseAndRotationalQuadraticModesMatchNativeAndRateScaling){
  for(int mode:{2,3,4}){
    SCOPED_TRACE(mode);auto c=Config();
    if(mode==2){c.parameters.H2=.3;c.parameters.HVISC=.5;}
    else{c.parameters.H3=.4;c.parameters.HELAS=.5;}
    // Native rotational quadratic damping uses HELAS*H3. Keep H1/H2 zero
    // there so no accumulated elastic-force history can contaminate the test.
    State s;ASSERT_EQ(Initialize(c,&s).status,Status::Ok);Motion m=Rectangle();
    Pure(m,mode,.2);Trial base;EvaluateChecked(s,m,.01,&base);ASSERT_TRUE(base.valid());
    const Vec3 first=mode==2?base.candidate().forces[0]:base.candidate().moments[0];
    EXPECT_GT(Norm(first),0);Balance(m,base.candidate());
    for(double scale:{-1.0,2.0}){
      Pure(m,mode,.2*scale);Trial t;EvaluateChecked(s,m,.01,&t);ASSERT_TRUE(t.valid());
      const double factor=scale*std::abs(scale);double power=0;
      for(int i=0;i<4;++i){
        Near(t.candidate().forces[i],Mul(base.candidate().forces[i],factor));
        Near(t.candidate().moments[i],Mul(base.candidate().moments[i],factor));
        power+=Dot(t.candidate().forces[i],m.v[i])+Dot(t.candidate().moments[i],m.w[i]);
      }
      EXPECT_LT(power,0);EXPECT_GT(t.candidate().elements[0].energy[1],0);
      for(int j=0;j<3;++j)EXPECT_EQ(t.candidate().elements[0].hour[j],0);
      Balance(m,t.candidate());
    }
  }
}

TEST(Hourglass, ElasticHistoryLoadUnloadReloadAndStationaryRetention){
  auto c=Config();c.parameters=Elastic();State s;ASSERT_EQ(Initialize(c,&s).status,Status::Ok);Motion m=Rectangle();
  bool observed_negative_work=false;const double sequence[]={1,1,0,-.5,-.5,-1,1};
  for(double scale:sequence){
    const double previous=s.accepted().elements[0].energy[1];Pure(m,0,.2*scale);
    Trial t;EvaluateChecked(s,m,.01,&t);ASSERT_TRUE(t.valid());
    if(scale==0){EXPECT_GT(Norm(t.candidate().forces[0]),0);EXPECT_EQ(t.candidate().elements[0].energy[1],previous);}
    if(t.candidate().elements[0].energy[1]<previous)observed_negative_work=true;
    EXPECT_EQ(Commit(&s,&t),Status::Ok);
  }
  EXPECT_TRUE(observed_negative_work);EXPECT_EQ(s.accepted().revision,7u);Near(s.accepted().accepted_time,.07);
}

TEST(Hourglass, InstantaneousRotationalEntriesDoNotPersistAtZeroVelocity){
  auto c=Config();c.parameters=Linear();c.parameters.HELAS=.5;c.parameters.H1=.2;
  State s;ASSERT_EQ(Initialize(c,&s).status,Status::Ok);Motion m=Rectangle();Pure(m,0,.2);
  const int h[]={1,-1,1,-1};for(int i=0;i<4;++i)m.w[i]={.3*h[i],-.1*h[i],0};
  Trial loaded;EvaluateChecked(s,m,.01,&loaded);ASSERT_TRUE(loaded.valid());
  EXPECT_NE(loaded.candidate().elements[0].hour[0],0);EXPECT_NE(loaded.candidate().elements[0].hour[3],0);
  ASSERT_EQ(Commit(&s,&loaded),Status::Ok);m.v.assign(4,{});m.w.assign(4,{});
  Trial stopped;EvaluateChecked(s,m,.01,&stopped);ASSERT_TRUE(stopped.valid());
  EXPECT_EQ(stopped.candidate().elements[0].hour[0],s.accepted().elements[0].hour[0]);
  EXPECT_EQ(stopped.candidate().elements[0].hour[3],0);EXPECT_EQ(stopped.candidate().elements[0].hour[4],0);
}

// Signed 3x3 cofactors of [1,x,y] give the affine-null direction, independently
// of donor PX/PY/VHX formulas. Normalize by h dot gamma = 4.
std::array<double,4> CofactorMode(const Motion& m){
  std::array<double,4> gamma{};const int h[]={1,-1,1,-1};double scale=0;
  for(int omit=0;omit<4;++omit){
    Vec3 p[3];int n=0;for(int i=0;i<4;++i)if(i!=omit)p[n++]=m.x[i];
    gamma[omit]=h[omit]*((p[1].x-p[0].x)*(p[2].y-p[0].y)-(p[2].x-p[0].x)*(p[1].y-p[0].y));
    scale+=h[omit]*gamma[omit];
  }
  for(auto& v:gamma)v*=4/scale;return gamma;
}

TEST(Hourglass, CorrectedTranslationalBranchAnnihilatesAffineFields){
  auto c=Config(Mode::CorrectedPlanar);c.parameters=Linear();c.parameters.HVISC=.5;
  c.parameters.HELAS=.5;c.parameters.H1=.2;c.parameters.H2=.3;
  State s;ASSERT_EQ(Initialize(c,&s).status,Status::Ok);Motion m=Distorted();
  const auto gamma=CofactorMode(m);double g0=0,gx=0,gy=0;
  for(int i=0;i<4;++i){g0+=gamma[i];gx+=gamma[i]*m.x[i].x;gy+=gamma[i]*m.x[i].y;
    const auto p=m.x[i];m.v[i]={.1+.2*p.x-.3*p.y,-.2+.4*p.x+.1*p.y,.3-.2*p.x+.5*p.y};}
  Near(g0,0);Near(gx,0);Near(gy,0);
  Trial t;EvaluateChecked(s,m,.01,&t);ASSERT_TRUE(t.valid());
  for(auto f:t.candidate().forces)Near(f,{},1e-9);for(auto v:t.candidate().elements[0].hour)Near(v,0,1e-9);
}

TEST(Hourglass, CorrectedPureNullModeHasCollinearRestoringForce){
  auto c=Config(Mode::CorrectedPlanar);c.parameters=Linear();State s;ASSERT_EQ(Initialize(c,&s).status,Status::Ok);
  Motion m=Distorted();const auto gamma=CofactorMode(m);
  for(int i=0;i<4;++i)m.v[i]={.2*gamma[i],0,0};
  Trial t;EvaluateChecked(s,m,.01,&t);ASSERT_TRUE(t.valid());Balance(m,t.candidate());
  const double multiplier=t.candidate().forces[0].x/gamma[0];EXPECT_LT(multiplier,0);
  for(int i=0;i<4;++i)Near(t.candidate().forces[i],{multiplier*gamma[i],0,0});
}

TEST(Hourglass, DistortedAngularAffineFieldRetainsNativeUniformModeSemantics){
  auto c=Config(Mode::CorrectedPlanar);c.parameters=Linear();State s;ASSERT_EQ(Initialize(c,&s).status,Status::Ok);
  Motion m=Distorted();for(int i=0;i<4;++i)m.w[i]={.2*m.x[i].x,0,0};
  Trial t;EvaluateChecked(s,m,.01,&t);ASSERT_TRUE(t.valid());
  EXPECT_GT(Norm(t.candidate().moments[0]),1e-8); // Explicit characterization, not affine annihilation.
}

Vec3 Rotate(Vec3 p){const double a=.51,b=-.39;Vec3 q{std::cos(a)*p.x-std::sin(a)*p.y,std::sin(a)*p.x+std::cos(a)*p.y,p.z};
  return {q.x,std::cos(b)*q.y-std::sin(b)*q.z,std::sin(b)*q.y+std::cos(b)*q.z};}
TEST(Hourglass, RotatedHistoryAndAtomicAssemblyMatchNative){
  auto c=Config();c.parameters=Linear();c.parameters.HELAS=.5;c.parameters.H1=.2;c.parameters.H2=.3;
  State base,moved;ASSERT_EQ(Initialize(c,&base).status,Status::Ok);ASSERT_EQ(Initialize(c,&moved).status,Status::Ok);
  for(double amplitude:{.2,-.1,.05}){
    Motion a=Rectangle();Pure(a,2,amplitude);Motion b=a;
    for(auto& p:b.x)p=Add(Rotate(p),{2,-3,4});for(auto& v:b.v)v=Rotate(v);for(auto& w:b.w)w=Rotate(w);
    Trial ta,tb;EvaluateChecked(base,a,.01,&ta);EvaluateChecked(moved,b,.01,&tb);ASSERT_TRUE(ta.valid());ASSERT_TRUE(tb.valid());
    for(int i=0;i<4;++i){Near(tb.candidate().forces[i],Rotate(ta.candidate().forces[i]));Near(tb.candidate().moments[i],Rotate(ta.candidate().moments[i]));}
    for(int j=0;j<5;++j)Near(tb.candidate().elements[0].hour[j],ta.candidate().elements[0].hour[j]);
    EXPECT_EQ(Commit(&base,&ta),Status::Ok);EXPECT_EQ(Commit(&moved,&tb),Status::Ok);
  }
}

TEST(Hourglass, CoordinateRotationAfterCommittedHistoryPreservesCorotatingResponse){
  auto c=Config();c.parameters=Linear();c.parameters.HELAS=.5;c.parameters.H1=.2;c.parameters.H2=.3;
  State reference,transformed;
  ASSERT_EQ(Initialize(c,&reference).status,Status::Ok);ASSERT_EQ(Initialize(c,&transformed).status,Status::Ok);
  Motion load=Rectangle();const int h[]={1,-1,1,-1};
  for(int i=0;i<4;++i){load.v[i]={.2*h[i],-.1*h[i],.15*h[i]};load.w[i]={.1*h[i],-.2*h[i],0};}
  Trial first,second;EvaluateChecked(reference,load,.01,&first);EvaluateChecked(transformed,load,.01,&second);
  ASSERT_TRUE(first.valid());ASSERT_TRUE(second.valid());
  ASSERT_EQ(Commit(&reference,&first),Status::Ok);ASSERT_EQ(Commit(&transformed,&second),Status::Ok);
  EXPECT_NE(reference.accepted().elements[0].hour[0],0);EXPECT_NE(reference.accepted().elements[0].hour[2],0);
  // Change coordinates AFTER history has been accepted. A zero-velocity probe
  // first isolates carried history; the next probe adds a corotating unload.
  // This is a coordinate-covariance test, not a finite-time rigid trajectory.
  for(double amplitude:{0.0,-.25}){
    Motion a=load;
    for(auto& v:a.v)v=Mul(v,amplitude);for(auto& w:a.w)w=Mul(w,amplitude);
    Motion b=a;for(auto& p:b.x)p=Add(Rotate(p),{2,-3,4});
    for(auto& v:b.v)v=Rotate(v);for(auto& w:b.w)w=Rotate(w);
    const double old_work=reference.accepted().elements[0].energy[1];
    Trial ta,tb;EvaluateChecked(reference,a,.01,&ta);EvaluateChecked(transformed,b,.01,&tb);
    ASSERT_TRUE(ta.valid());ASSERT_TRUE(tb.valid());Balance(a,ta.candidate());Balance(b,tb.candidate());
    for(int i=0;i<4;++i){
      Near(tb.candidate().forces[i],Rotate(ta.candidate().forces[i]));
      Near(tb.candidate().moments[i],Rotate(ta.candidate().moments[i]));
    }
    for(int j=0;j<5;++j)Near(tb.candidate().elements[0].hour[j],ta.candidate().elements[0].hour[j]);
    Near(tb.candidate().elements[0].energy[1],ta.candidate().elements[0].energy[1]);
    if(amplitude==0){
      EXPECT_GT(Norm(tb.candidate().forces[0]),0);
      EXPECT_EQ(ta.candidate().elements[0].energy[1],old_work);
      EXPECT_EQ(tb.candidate().elements[0].energy[1],transformed.accepted().elements[0].energy[1]);
      for(int j=0;j<3;++j)EXPECT_EQ(tb.candidate().elements[0].hour[j],transformed.accepted().elements[0].hour[j]);
    }
    ASSERT_EQ(Commit(&reference,&ta),Status::Ok);ASSERT_EQ(Commit(&transformed,&tb),Status::Ok);
  }
  EXPECT_EQ(reference.accepted().revision,3u);EXPECT_EQ(transformed.accepted().revision,3u);
}

TEST(Hourglass, SharedNodeElementsAccumulateIntoOneForceBuffer){
  auto c=Config();c.parameters=Linear();c.parameters.HELAS=.5;c.parameters.H1=.2;c.node_count=6;
  c.elements[0].nodes={0,1,2,3};Element second=c.elements[0];second.nodes={1,4,5,2};second.young=1300;second.thickness=.2;c.elements.push_back(second);
  Motion m{{{0,0,0},{1,0,0},{1,1,0},{0,1,0},{2,0,0},{2,1,0}},std::vector<Vec3>(6),std::vector<Vec3>(6)};
  for(int i=0;i<6;++i){m.v[i]={.03*i*i,-.01*i,.02*i};m.w[i]={.02*i,-.03*i*i,0};}
  State s;ASSERT_EQ(Initialize(c,&s).status,Status::Ok);Trial t;EvaluateChecked(s,m,.01,&t);ASSERT_TRUE(t.valid());Balance(m,t.candidate());
  EXPECT_NE(t.candidate().elements[0].hour[0],t.candidate().elements[1].hour[0]);
}

TEST(Hourglass, SixteenElementsExerciseStateAndGeometryStrides){
  Configuration c;c.node_count=64;c.parameters=Linear();c.parameters.HELAS=.5;c.parameters.H1=.2;
  Motion m;
  for(int e=0;e<16;++e){Element el;el.nodes={4*e,4*e+1,4*e+2,4*e+3};el.thickness=.03+.01*e;el.young=1000+100*e;c.elements.push_back(el);
    Motion q=Rectangle();Pure(q,e%5,.01*(e+1));for(auto p:q.x)m.x.push_back(Add(p,{4.0*e,0,0}));
    m.v.insert(m.v.end(),q.v.begin(),q.v.end());m.w.insert(m.w.end(),q.w.begin(),q.w.end());}
  State s;ASSERT_EQ(Initialize(c,&s).status,Status::Ok);Trial t;EvaluateChecked(s,m,.01,&t);ASSERT_TRUE(t.valid());
  EXPECT_EQ(t.candidate().device_bytes,19584u);Balance(m,t.candidate());
}

std::vector<unsigned char> StateBytes(const Snapshot& s){
  std::vector<unsigned char> out;auto append=[&](const auto& value){const auto* p=reinterpret_cast<const unsigned char*>(&value);out.insert(out.end(),p,p+sizeof(value));};
  append(s.revision);append(s.accepted_time);append(s.device_bytes);
  for(const auto& e:s.elements){append(e.hour);append(e.energy);append(e.off);append(e.reference_coordinates);append(e.edge_diagonal_lengths);
    append(e.frame);append(e.area);append(e.px1);append(e.px2);append(e.py1);append(e.py2);append(e.vhx);append(e.vhy);}
  for(auto v:s.forces)append(v);for(auto v:s.moments)append(v);return out;
}

TEST(HourglassTransaction, CorrectedPlanarPersistentLoadUnloadRejectAndRetry){
  auto c=Config(Mode::CorrectedPlanar);c.parameters=Elastic();State s;ASSERT_EQ(Initialize(c,&s).status,Status::Ok);
  Motion m=Distorted();const auto gamma=CofactorMode(m);bool negative_work=false;
  const double sequence[]={1,1,0,-.5,-.5,-1,1};
  for(int step=0;step<7;++step){
    for(int i=0;i<4;++i)m.v[i]=Mul({.2*gamma[i],-.1*gamma[i],.15*gamma[i]},sequence[step]);
    const auto before=StateBytes(s.accepted());const double old_work=s.accepted().elements[0].energy[1];
    Trial clean;EvaluateChecked(s,m,.01,&clean);ASSERT_TRUE(clean.valid());
    EXPECT_EQ(StateBytes(s.accepted()),before);EXPECT_EQ(clean.candidate().elements[0].off,1);
    Balance(m,clean.candidate());
    const Vec3 coefficient=Mul(clean.candidate().forces[0],1/gamma[0]);
    for(int i=0;i<4;++i)Near(clean.candidate().forces[i],Mul(coefficient,gamma[i]));
    if(sequence[step]==0){
      EXPECT_GT(Norm(clean.candidate().forces[0]),0);EXPECT_EQ(clean.candidate().elements[0].energy[1],old_work);
      for(int j=0;j<3;++j)EXPECT_EQ(clean.candidate().elements[0].hour[j],s.accepted().elements[0].hour[j]);
    }
    if(clean.candidate().elements[0].energy[1]<old_work)negative_work=true;
    if(step==3){
      Trial rejected;Options fault;fault.reject_after_assembly=true;
      EXPECT_EQ(EvaluateTrial(s,m.view(),.01,&rejected,fault).status,Status::TrialRejected);
      EXPECT_FALSE(rejected.valid());EXPECT_EQ(StateBytes(s.accepted()),before);
      Trial retry;EvaluateChecked(s,m,.01,&retry);ASSERT_TRUE(retry.valid());
      EXPECT_EQ(StateBytes(retry.candidate()),StateBytes(clean.candidate()));
      ASSERT_EQ(Commit(&s,&retry),Status::Ok);Discard(&clean);
    }else ASSERT_EQ(Commit(&s,&clean),Status::Ok);
  }
  EXPECT_TRUE(negative_work);EXPECT_EQ(s.accepted().revision,7u);Near(s.accepted().accepted_time,.07);
  EXPECT_NE(s.accepted().elements[0].hour[0],0);EXPECT_NE(s.accepted().elements[0].hour[2],0);
}

TEST(HourglassTransaction, DiscardFailureStaleOwnerAndRetryDoNotLeakState){
  auto c=Config();c.parameters=Elastic();State s,other;ASSERT_EQ(Initialize(c,&s).status,Status::Ok);ASSERT_EQ(Initialize(c,&other).status,Status::Ok);
  Motion m=Rectangle();Pure(m,0,.2);Trial first;EvaluateChecked(s,m,.01,&first);ASSERT_TRUE(first.valid());ASSERT_EQ(Commit(&s,&first),Status::Ok);
  const auto before=StateBytes(s.accepted());const auto input_v=m.v;
  Trial rejected;Options reject;reject.reject_after_assembly=true;
  EXPECT_EQ(EvaluateTrial(s,m.view(),.01,&rejected,reject).status,Status::TrialRejected);EXPECT_FALSE(rejected.valid());EXPECT_EQ(StateBytes(s.accepted()),before);
  Trial a,b;EvaluateChecked(s,m,.01,&a);EvaluateChecked(s,m,.01,&b);ASSERT_TRUE(a.valid());ASSERT_TRUE(b.valid());
  EXPECT_EQ(StateBytes(a.candidate()),StateBytes(b.candidate()));
  EXPECT_EQ(Commit(&other,&a),Status::StaleTrial);EXPECT_EQ(StateBytes(s.accepted()),before);
  EXPECT_EQ(Commit(&s,&a),Status::Ok);const auto accepted=StateBytes(s.accepted());
  EXPECT_EQ(Commit(&s,&b),Status::StaleTrial);EXPECT_EQ(StateBytes(s.accepted()),accepted);Discard(&b);EXPECT_EQ(Commit(&s,&b),Status::NoTrial);
  for(int i=0;i<4;++i){EXPECT_EQ(m.v[i].x,input_v[i].x);EXPECT_EQ(m.v[i].y,input_v[i].y);EXPECT_EQ(m.v[i].z,input_v[i].z);}
  Trial stale;EvaluateChecked(s,m,.01,&stale);ASSERT_TRUE(stale.valid());Options cap;cap.max_device_bytes=1;
  EXPECT_EQ(EvaluateTrial(s,m.view(),.01,&stale,cap).status,Status::ResourceLimit);EXPECT_FALSE(stale.valid());EXPECT_EQ(StateBytes(s.accepted()),accepted);
}

TEST(HourglassValidation, InvalidAndUnsupportedDomainsLeaveAcceptedState){
  auto c=Config();State s;ASSERT_EQ(Initialize(c,&s).status,Status::Ok);const auto before=StateBytes(s.accepted());Motion m=Rectangle();Trial t;
  EXPECT_EQ(Initialize(c,&s).status,Status::InvalidInput);
  EXPECT_EQ(EvaluateTrial(s,m.view(),0,&t).status,Status::InvalidInput);
  BorrowedKinematics missing=m.view();missing.velocity=nullptr;EXPECT_EQ(EvaluateTrial(s,missing,.01,&t).status,Status::InvalidInput);
  m.x[2].z=.1;EXPECT_EQ(EvaluateTrial(s,m.view(),.01,&t).status,Status::UnsupportedGeometry);m=Distorted();
  EXPECT_EQ(EvaluateTrial(s,m.view(),.01,&t).status,Status::UnsupportedGeometry);m=Rectangle();m.v[0].x=std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(EvaluateTrial(s,m.view(),.01,&t).status,Status::InvalidInput);EXPECT_EQ(StateBytes(s.accepted()),before);
  auto bad=c;bad.parameters.H1=-.1;State unused;EXPECT_EQ(Initialize(bad,&unused).status,Status::InvalidInput);
  bad=c;bad.elements[0].nodes[3]=2;EXPECT_EQ(Initialize(bad,&unused).status,Status::UnsupportedGeometry);
  bad=c;bad.elements[0].thickness=std::numeric_limits<double>::denorm_min();
  EXPECT_EQ(Initialize(bad,&unused).status,Status::InvalidInput);
  bad=c;bad.elements[0].young=std::numeric_limits<double>::denorm_min();
  EXPECT_EQ(Initialize(bad,&unused).status,Status::InvalidInput);

  // Finite borrowed data may still overflow the donor quadratic response.
  // Exercise actual output validation, then retry from the same accepted state.
  auto quadratic=Config();quadratic.parameters.H1=.2;quadratic.parameters.HVISC=.5;
  State bounded;ASSERT_EQ(Initialize(quadratic,&bounded).status,Status::Ok);
  m=Rectangle();Pure(m,0,.2);Trial initial;EvaluateChecked(bounded,m,.01,&initial);ASSERT_TRUE(initial.valid());
  ASSERT_EQ(Commit(&bounded,&initial),Status::Ok);const auto accepted=StateBytes(bounded.accepted());
  Trial overflow;EvaluateChecked(bounded,m,.01,&overflow);ASSERT_TRUE(overflow.valid());
  const auto expected=StateBytes(overflow.candidate());Pure(m,0,1e200);
  for(const auto v:m.v)ASSERT_TRUE(std::isfinite(v.x));
  EXPECT_EQ(EvaluateTrial(bounded,m.view(),.01,&overflow).status,Status::InvalidOutput);
  EXPECT_FALSE(overflow.valid());EXPECT_EQ(StateBytes(bounded.accepted()),accepted);
  Pure(m,0,.2);Trial retry;EvaluateChecked(bounded,m,.01,&retry);ASSERT_TRUE(retry.valid());
  EXPECT_EQ(StateBytes(retry.candidate()),expected);EXPECT_EQ(StateBytes(bounded.accepted()),accepted);
  EXPECT_EQ(Commit(&bounded,&retry),Status::Ok);
}

TEST(HourglassTransaction, AcceptedHistoryLimitIsExplicit){
  State s;ASSERT_EQ(Initialize(Config(),&s).status,Status::Ok);Motion m=Rectangle();
  for(int i=0;i<128;++i){Trial t;Report r=EvaluateTrial(s,m.view(),.001,&t);ASSERT_EQ(r.status,Status::Ok)<<r.message;ASSERT_EQ(Commit(&s,&t),Status::Ok);}
  const auto before=StateBytes(s.accepted());Trial t;EXPECT_EQ(EvaluateTrial(s,m.view(),.001,&t).status,Status::HistoryLimit);EXPECT_EQ(StateBytes(s.accepted()),before);
}

}  // namespace
}  // namespace shell_hourglass
