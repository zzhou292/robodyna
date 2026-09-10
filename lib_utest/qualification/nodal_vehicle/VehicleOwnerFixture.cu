#include "VehicleOwnerFixture.h"
#include <math_constants.h>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

namespace tl::fea::vehicle_test {
void Fields::Fill(double value) { for(auto* a:{&x,&v,&w,&q,&rf,&rc})std::fill(a->begin(),a->end(),value); }
Initial::Initial(std::size_t count):n(count),x(3*n),v(3*n),w(3*n),q(4*n),inverse(n,.5),inertia(n,2),fixed(n),rotation_fixed(n) {
  for(std::size_t i=0;i<n;++i) {
    x[3*i]=double(i)/1048576; x[3*i+1]=-.5; x[3*i+2]=.25;
    v[3*i]=.125; q[4*i]=1;
  }
  // Adjacent fully fixed and free last nodes exercise both reaction branches.
  if(n>2) { const auto f=n-2;fixed[f]=7;rotation_fixed[f]=1;inverse[f]=inertia[f]=0;v[3*f]=0; }
}
NodalStateConfig Initial::config() const {
  NodalStateConfig c;c.node_count=n;c.max_nodes=n;c.max_device_bytes=MaxActiveNodalStateDeviceBytes;
  c.fixed_dt=H;c.temporal_scheme=NodalTemporalScheme::StaggeredHalfKickStart;return c;
}
NodalReport Initial::Initialize(FENodalState& owner,const NodalStateConfig& c) const {
  return owner.Initialize(c,{x.data(),v.data(),w.data(),n,q.data()},inverse.data(),
                          {fixed.data(),rotation_fixed.data(),inertia.data()});
}
namespace {
__global__ void WriteSparse(NodalAssemblyView v,bool bad) {
  const auto n=v.accepted.node_count;
  v.forces.force_x[0]=1;
  v.forces.force_y[n/2]=-2;
  v.forces.force_x[n-2]=5;v.forces.couple_z[n-2]=7;
  v.forces.force_x[n-1]=4;
  v.forces.couple_z[n-1]=bad?1.7976931348623157e308:3;
}
__global__ void Corrupt(double* q,std::size_t n) { q[4*n-1]=CUDART_NAN; }
bool Bits(const std::vector<double>& a,const std::vector<double>& b) {
  return a.size()==b.size()&&std::memcmp(a.data(),b.data(),a.size()*sizeof(double))==0;
}
}
void SparseLoads(const NodalAssemblyView& v,bool bad) { WriteSparse<<<1,1,0,v.stream>>>(v,bad); }
void CorruptLastQuaternion(const NodalPreparedView& v) {
  // Fault injection into a completed private candidate only; production callers
  // must never write through retained views. Accepted storage remains untouched.
  Corrupt<<<1,1,0,v.stream>>>(const_cast<double*>(v.kinematics.orientation_wxyz),v.kinematics.node_count);
}
NodalReport Prepare(FENodalState& owner,NodalTrialToken& token,bool bad) {
  NodalAssemblyView v;auto r=owner.BeginTrial(&token,&v);if(r.status!=NodalStatus::Ok)return r;
  SparseLoads(v,bad);r=owner.SealAssembly(token);if(r.status!=NodalStatus::Ok)return r;
  return AdvanceStaggeredPrescribed(owner,token,{v.owner_id,v.accepted.base_epoch,v.attempt,H,.1});
}
void SameFields(const Fields& a,const Fields& b) {
  EXPECT_TRUE(Bits(a.x,b.x));EXPECT_TRUE(Bits(a.v,b.v));EXPECT_TRUE(Bits(a.w,b.w));
  EXPECT_TRUE(Bits(a.q,b.q));EXPECT_TRUE(Bits(a.rf,b.rf));EXPECT_TRUE(Bits(a.rc,b.rc));
}
void InitialFields(const Initial& in,const Fields& out) {
  EXPECT_TRUE(Bits(in.x,out.x));EXPECT_TRUE(Bits(in.v,out.v));
  EXPECT_TRUE(Bits(in.w,out.w));EXPECT_TRUE(Bits(in.q,out.q));
  EXPECT_TRUE(std::all_of(out.rf.begin(),out.rf.end(),[](double x){return x==0;}));
  EXPECT_TRUE(std::all_of(out.rc.begin(),out.rc.end(),[](double x){return x==0;}));
}
void Analytic(const Initial& in,const Fields& out,unsigned steps) {
  const auto n=in.n;const double kicks=(steps-.5)*H,drifts=.5*steps*steps*H*H;
  for(auto i:{std::size_t(0),std::size_t(128),std::size_t(2048),n/2,n-3,n-2,n-1}) {
    SCOPED_TRACE(i);
    const double f[]{i==0?1.:(i==n-1?4.:(i==n-2?5.:0.)),i==n/2?-2.:0.,0.};
    for(unsigned a=0;a<3;++a) {
      const auto j=3*i+a;
      EXPECT_NEAR(out.v[j],in.v[j]+kicks*in.inverse[i]*f[a],2e-15);
      EXPECT_NEAR(out.x[j],in.x[j]+steps*H*in.v[j]+drifts*in.inverse[i]*f[a],2e-15);
      EXPECT_DOUBLE_EQ(out.rf[j],i==n-2?-f[a]:0.);
      EXPECT_DOUBLE_EQ(out.rc[j],i==n-2&&a==2?-7.:0.);
    }
    const double alpha=i==n-1?6.:0.,angle=drifts*alpha;
    EXPECT_NEAR(out.w[3*i+2],kicks*alpha,2e-15);
    EXPECT_DOUBLE_EQ(out.w[3*i],0.);EXPECT_DOUBLE_EQ(out.w[3*i+1],0.);
    EXPECT_NEAR(out.q[4*i],std::cos(angle/2),2e-15);
    EXPECT_NEAR(out.q[4*i+3],std::sin(angle/2),2e-15);
    EXPECT_DOUBLE_EQ(out.q[4*i+1],0.);EXPECT_DOUBLE_EQ(out.q[4*i+2],0.);
  }
}
std::uint64_t FieldBitsHash(const Fields& fields) {
  std::uint64_t h=1469598103934665603ull;
  for(const auto* a:{&fields.x,&fields.v,&fields.w,&fields.q,&fields.rf,&fields.rc})
    for(double value:*a) { std::uint64_t bits;std::memcpy(&bits,&value,sizeof(bits));h=(h^bits)*1099511628211ull; }
  return h;
}
} // namespace tl::fea::vehicle_test
