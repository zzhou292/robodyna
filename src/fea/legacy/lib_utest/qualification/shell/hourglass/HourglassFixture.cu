#include "HourglassFixture.h"
#include <cuda_runtime.h>
#include <algorithm>
#include <cmath>
#include <new>
#include <stdexcept>
#include <utility>

// Immutable OpenRadioss K1/K3 from the pinned donor directory. Original source
// notices and LICENSE.md remain intact. No material or stabilization math edits.
#include "shell_geometry_kernel.h"
#include "shell_force_assembly_kernel.h"
#include "shell_geometry_kernel.cu"
#include "shell_force_assembly_kernel.cu"
static_assert(std::is_same<Real,double>::value,"MYREAL8 required");

namespace shell_hourglass {
namespace {
Vec3 Sub(Vec3 a,Vec3 b){return {a.x-b.x,a.y-b.y,a.z-b.z};}
Vec3 Mul(Vec3 a,double s){return {a.x*s,a.y*s,a.z*s};}
double Dot(Vec3 a,Vec3 b){return a.x*b.x+a.y*b.y+a.z*b.z;}
Vec3 Cross(Vec3 a,Vec3 b){return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};}
double Norm(Vec3 a){return std::hypot(a.x,a.y,a.z);}
bool Finite(Vec3 a){return std::isfinite(a.x)&&std::isfinite(a.y)&&std::isfinite(a.z);}
bool Positive(double x){return std::isfinite(x)&&x>0;}
Report Error(Status s,const char* text,std::size_t bytes=0){return {s,text,bytes};}

Report ValidateConfiguration(const Configuration& c){
  if(c.node_count<4||c.node_count>64||c.elements.empty()||c.elements.size()>16)
    return Error(Status::ResourceLimit,"Requires 1..16 elements and 4..64 nodes");
  if(c.mode!=Mode::UniformRectangle&&c.mode!=Mode::CorrectedPlanar)
    return Error(Status::InvalidInput,"Unsupported S2b mode");
  const auto& p=c.parameters;
  for(double x:{p.H1,p.H2,p.H3,p.SRH1,p.SRH2,p.SRH3,p.HVISC,p.HELAS,p.HVLIN})
    if(!std::isfinite(x)||x<0||x>10)
      return Error(Status::InvalidInput,"S2b coefficients must be finite in [0,10]");
  for(const Element& e:c.elements){
    if(!Positive(e.thickness)||!Positive(e.young)||!Positive(e.density)||
       !Positive(e.sound_speed)||!std::isfinite(e.nu)||e.nu<=-1||e.nu>=.5||
       !std::isfinite(e.shear_factor)||e.shear_factor<0||
       !Positive(e.thickness*e.thickness)||
       !Positive(e.young/(1-e.nu*e.nu))||!Positive(e.young/(2*(1+e.nu))))
      return Error(Status::InvalidInput,"Invalid material/section scalars");
    for(int i=0;i<4;++i){
      if(e.nodes[i]<0||static_cast<std::size_t>(e.nodes[i])>=c.node_count)
        return Error(Status::InvalidInput,"Connectivity out of bounds");
      for(int j=0;j<i;++j)if(e.nodes[i]==e.nodes[j])
        return Error(Status::UnsupportedGeometry,"Four distinct Q4 nodes required");
    }
  }
  return {Status::Ok,{},0};
}

Report ValidateGeometry(const Configuration& config,const BorrowedKinematics& in,
                        const Snapshot& accepted,std::vector<std::array<double,6>>* lengths){
  if(!in.positions||!in.velocity||!in.angular_velocity||in.node_count!=config.node_count)
    return Error(Status::InvalidInput,"Borrowed kinematic arrays/count invalid");
  for(std::size_t i=0;i<in.node_count;++i)
    if(!Finite(in.positions[i])||!Finite(in.velocity[i])||!Finite(in.angular_velocity[i]))
      return Error(Status::InvalidInput,"Nonfinite borrowed kinematics");
  lengths->resize(config.elements.size());
  for(std::size_t e=0;e<config.elements.size();++e){
    std::array<Vec3,4> p{};
    for(int i=0;i<4;++i)p[i]=in.positions[config.elements[e].nodes[i]];
    auto& len=(*lengths)[e];
    for(int i=0;i<4;++i)len[i]=Norm(Sub(p[(i+1)%4],p[i]));
    len[4]=Norm(Sub(p[2],p[0]));len[5]=Norm(Sub(p[3],p[1]));
    const double scale=*std::max_element(len.begin(),len.end());
    if(!std::isfinite(scale)||scale<1e-6||scale>1e6)
      return Error(Status::UnsupportedGeometry,"S2b geometry scale outside [1e-6,1e6]");
    Vec3 n=Cross(Mul(Sub(p[1],p[0]),1/scale),Mul(Sub(p[2],p[0]),1/scale));
    const double norm=Norm(n);if(!(norm>1e-10))return Error(Status::UnsupportedGeometry,"Degenerate plane");
    n=Mul(n,1/norm);
    if(std::abs(Dot(Mul(Sub(p[3],p[0]),1/scale),n))>1e-10)
      return Error(Status::UnsupportedGeometry,"Warped shell outside S2b scope");
    for(int i=0;i<4;++i){
      const Vec3 a=Mul(Sub(p[(i+1)%4],p[i]),1/scale),b=Mul(Sub(p[(i+2)%4],p[(i+1)%4]),1/scale);
      if(!(Dot(Cross(a,b),n)>1e-10))return Error(Status::UnsupportedGeometry,"Q4 must be strictly convex");
      if(config.mode==Mode::UniformRectangle&&std::abs(Dot(a,b))>1e-10)
        return Error(Status::UnsupportedGeometry,"Uniform S2b mode admits rectangles only");
    }
    // S2b studies history on a fixed shape, allowing a whole-fixture rigid
    // transform. Deformation/reference-frame evolution belongs to S2c.
    if(accepted.revision){
      for(int i=0;i<6;++i)
        if(std::abs(len[i]-accepted.elements[e].edge_diagonal_lengths[i])>1e-9*scale)
          return Error(Status::UnsupportedGeometry,"Shape change requires the later phase/state gate");
    }
  }
  return {Status::Ok,{},0};
}

struct CudaFailure:std::runtime_error{using std::runtime_error::runtime_error;};
void Check(cudaError_t e,const char* text){if(e!=cudaSuccess)throw CudaFailure(std::string(text)+": "+cudaGetErrorString(e));}
class Stream {
 public:
  Stream(){Check(cudaStreamCreateWithFlags(&value_,cudaStreamNonBlocking),"create stream");}
  ~Stream(){if(value_)cudaStreamDestroy(value_);}
  Stream(const Stream&)=delete;Stream& operator=(const Stream&)=delete;
  operator cudaStream_t()const{return value_;}
 private:cudaStream_t value_=nullptr;
};
template<class T>class Device {
 public:
  explicit Device(std::size_t n){Check(cudaMalloc(reinterpret_cast<void**>(&p_),n*sizeof(T)),"allocate slab");}
  ~Device(){if(p_)cudaFree(p_);}
  Device(const Device&)=delete;Device& operator=(const Device&)=delete;
  T* data()const{return p_;}
 private:T* p_=nullptr;
};
#include "SlabFields.inc"
#include "DonorLaunch.inc"
}  // namespace

Report Initialize(const Configuration& c,State* state){
  if(!state||state->initialized_)return Error(Status::InvalidInput,"State null or already initialized");
  Report valid=ValidateConfiguration(c);if(valid.status!=Status::Ok)return valid;
  try{
    Configuration config=c;
    Snapshot initial;initial.elements.resize(c.elements.size());
    initial.forces.resize(c.node_count);initial.moments.resize(c.node_count);
    using std::swap;swap(state->configuration_,config);swap(state->accepted_,initial);
    state->initialized_=true;return {Status::Ok,{},0};
  }catch(const std::bad_alloc&){return Error(Status::ResourceLimit,"Host state allocation failed");}
}

void Discard(Trial* t)noexcept{if(t){t->valid_=false;t->owner_=nullptr;}}
Status Commit(State* state,Trial* trial)noexcept{
  if(!state||!trial||!trial->valid_)return Status::NoTrial;
  if(trial->owner_!=state||trial->base_revision_!=state->accepted_.revision)return Status::StaleTrial;
  static_assert(std::is_nothrow_swappable<Snapshot>::value,"Publication cannot allocate or throw");
  using std::swap;swap(state->accepted_,trial->candidate_);Discard(trial);return Status::Ok;
}

Report EvaluateTrial(const State& state,const BorrowedKinematics& in,double dt,
                     Trial* trial,const Options& options){
  if(!trial)return Error(Status::InvalidInput,"Null trial");
  Discard(trial);  // An unsuccessful new attempt cannot leave an older trial valid.
  if(!state.initialized_||!std::isfinite(dt)||dt<1e-8||dt>1)
    return Error(Status::InvalidInput,"Uninitialized state or dt outside [1e-8,1]");
  if(state.accepted_.revision>=128)return Error(Status::HistoryLimit,"128 accepted increments reached");
  const auto& config=state.configuration_;
  const int ne=static_cast<int>(config.elements.size()),nn=static_cast<int>(config.node_count);
  const std::size_t count=ElementPlanes*ne+17*nn;
  const std::size_t bytes=count*sizeof(double)+4*ne*sizeof(int);
  if(bytes>options.max_device_bytes||bytes>1024*1024)
    return Error(Status::ResourceLimit,"Explicit device allocation cap exceeded",bytes);
  try{
    std::vector<std::array<double,6>> lengths;
    Report valid=ValidateGeometry(config,in,state.accepted_,&lengths);
    if(valid.status!=Status::Ok)return valid;
    std::vector<double> host(count,0);std::vector<int> connectivity(4*ne);
    auto h=[&](int field,int e)->double&{return host[field*ne+e];};
    const int nodes_start=ElementPlanes*ne;
    auto put=[&](int component,const Vec3* values){for(int i=0;i<nn;++i){
      host[nodes_start+component*nn+3*i]=values[i].x;
      host[nodes_start+component*nn+3*i+1]=values[i].y;
      host[nodes_start+component*nn+3*i+2]=values[i].z;}};
    put(0,in.positions);put(3,in.velocity);put(6,in.angular_velocity);
    for(int i=0;i<ne;++i){
      const Element& e=config.elements[i];const ElementState& a=state.accepted_.elements[i];
      for(int j=0;j<4;++j)connectivity[j*ne+i]=e.nodes[j];
      h(Off,i)=a.off;
      for(int j=0;j<6;++j)h(Smstr+j,i)=a.reference_coordinates[j];
      for(int j=0;j<5;++j)h(Hour+j,i)=a.hour[j];
      for(int j=0;j<2;++j)h(Energy+j,i)=a.energy[j];
      h(Thickness,i)=e.thickness;h(ThicknessSquared,i)=e.thickness*e.thickness;
      h(Young,i)=e.young;h(Nu,i)=e.nu;h(Rho,i)=e.density;
      h(Sound,i)=e.sound_speed;h(ShearFactor,i)=e.shear_factor;
      h(A11,i)=e.young/(1-e.nu*e.nu);h(ShearModulus,i)=e.young/(2*(1+e.nu));
    }
    Stream stream;Device<double> storage(count);Device<int> indices(connectivity.size());
    auto d=[&](int f){return storage.data()+f*ne;};
    auto node=[&](int f){return storage.data()+nodes_start+f*nn;};
    auto c=[&](int i){return indices.data()+i*ne;};
    Check(cudaMemcpyAsync(storage.data(),host.data(),count*sizeof(double),cudaMemcpyHostToDevice,stream),"upload trial slab");
    Check(cudaMemcpyAsync(indices.data(),connectivity.data(),connectivity.size()*sizeof(int),cudaMemcpyHostToDevice,stream),"upload connectivity");
    const auto& p=config.parameters;
    HourglassParams hg{p.H1,p.H2,p.H3,p.SRH1,p.SRH2,p.SRH3,p.HVISC,p.HELAS,p.HVLIN};
    if(config.mode==Mode::UniformRectangle)LaunchGeometry<1>(d,node,c,ne,hg,stream);
    else LaunchGeometry<2>(d,node,c,ne,hg,stream);
    Check(cudaGetLastError(),"launch K1");
    Check(cudaMemcpyAsync(host.data(),storage.data(),count*sizeof(double),cudaMemcpyDeviceToHost,stream),"download geometry");
    Check(cudaStreamSynchronize(stream),"synchronize K1");
    if(!std::all_of(host.begin(),host.end(),[](double v){return std::isfinite(v);}))
      return Error(Status::InvalidOutput,"Nonfinite geometry trial",bytes);
    for(int i=0;i<ne;++i)if(!(h(Area,i)>0))return Error(Status::InvalidOutput,"Nonpositive geometry area",bytes);
    if(config.mode==Mode::UniformRectangle)LaunchForces<1>(d,node,c,ne,hg,dt,stream);
    else LaunchForces<2>(d,node,c,ne,hg,dt,stream);
    Check(cudaGetLastError(),"launch K3");
    Check(cudaMemcpyAsync(host.data(),storage.data(),count*sizeof(double),cudaMemcpyDeviceToHost,stream),"download forces/history");
    Check(cudaStreamSynchronize(stream),"synchronize K3");
    if(!std::all_of(host.begin(),host.end(),[](double v){return std::isfinite(v);}))
      return Error(Status::InvalidOutput,"Nonfinite force/history trial",bytes);
    Snapshot next;next.revision=state.accepted_.revision+1;
    next.accepted_time=state.accepted_.accepted_time+dt;next.device_bytes=bytes;
    next.elements.resize(ne);next.forces.resize(nn);next.moments.resize(nn);
    for(int i=0;i<ne;++i){
      auto& e=next.elements[i];e.off=h(Off,i);e.area=h(Area,i);
      e.px1=h(Px1,i);e.px2=h(Px2,i);e.py1=h(Py1,i);e.py2=h(Py2,i);e.vhx=h(Vhx,i);e.vhy=h(Vhy,i);
      const double expected_off=config.mode==Mode::UniformRectangle?2:1;
      if(e.off!=expected_off)return Error(Status::InvalidOutput,"Unexpected OFF lifecycle",bytes);
      for(int j=0;j<3;++j)e.frame[j]={h(Frame+3*j,i),h(Frame+3*j+1,i),h(Frame+3*j+2,i)};
      if(std::abs(Norm(e.frame[0])-1)>1e-10||std::abs(Norm(e.frame[1])-1)>1e-10||
         Norm(Sub(Cross(e.frame[0],e.frame[1]),e.frame[2]))>1e-10)
        return Error(Status::InvalidOutput,"Invalid K1 frame",bytes);
      for(int j=0;j<6;++j)e.reference_coordinates[j]=h(Smstr+j,i);
      for(int j=0;j<5;++j)e.hour[j]=h(Hour+j,i);
      for(int j=0;j<2;++j)e.energy[j]=h(Energy+j,i);
      e.edge_diagonal_lengths=lengths[i];
    }
    for(int i=0;i<nn;++i){
      next.forces[i]={host[nodes_start+9*nn+i],host[nodes_start+10*nn+i],host[nodes_start+11*nn+i]};
      next.moments[i]={host[nodes_start+12*nn+i],host[nodes_start+13*nn+i],host[nodes_start+14*nn+i]};
    }
    if(options.reject_after_assembly)return Error(Status::TrialRejected,"Injected post-assembly trial rejection",bytes);
    using std::swap;swap(trial->candidate_,next);trial->base_revision_=state.accepted_.revision;
    trial->owner_=&state;trial->valid_=true;return {Status::Ok,{},bytes};
  }catch(const CudaFailure& e){return {Status::CudaError,e.what(),bytes};}
   catch(const std::bad_alloc&){return Error(Status::ResourceLimit,"Host trial allocation failed",bytes);}
}
}  // namespace shell_hourglass
