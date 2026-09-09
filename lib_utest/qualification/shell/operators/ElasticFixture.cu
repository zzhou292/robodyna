#include "ElasticFixture.h"

#include <cuda_runtime.h>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <new>
#include <stdexcept>
#include <type_traits>
#include <utility>

// Distinct symbol names for the two qualification TUs. The original donor files
// are immutable; the Gauss variant's sole numerical change is the generated
// thickness rule. No material struct crosses the TU boundary.
#ifdef SHELL_ELASTIC_GAUSS
#define shell_geometry_kernel s2a_gauss_geometry_kernel
#define launch_shell_geometry_kernel s2a_gauss_geometry_launcher
#define safe_inv_norm s2a_gauss_safe_inv_norm
#define JohnsonCookParams S2aGaussJohnsonCookParams
#define m2cplr_device s2a_gauss_m2cplr_device
#define shell_strain_material_kernel s2a_gauss_strain_kernel
#define launch_shell_strain_material_kernel s2a_gauss_strain_launcher
#define S2A_ENTRY EvaluateElasticGauss
#else
#define shell_geometry_kernel s2a_original_geometry_kernel
#define launch_shell_geometry_kernel s2a_original_geometry_launcher
#define safe_inv_norm s2a_original_safe_inv_norm
#define JohnsonCookParams S2aOriginalJohnsonCookParams
#define m2cplr_device s2a_original_m2cplr_device
#define shell_strain_material_kernel s2a_original_strain_kernel
#define launch_shell_strain_material_kernel s2a_original_strain_launcher
#define S2A_ENTRY EvaluateElasticOriginal
#endif
#include "shell_geometry_kernel.cu"
#ifdef SHELL_ELASTIC_GAUSS
#include "shell_strain_material_kernel_gauss3.cu"
#else
#include "shell_strain_material_kernel.cu"
#endif

static_assert(std::is_same<Real,double>::value,"MYREAL8 must be set for every CUDA fixture TU");
static_assert(std::is_standard_layout<JohnsonCookParams>::value,"Donor material must be POD layout");
static_assert(sizeof(JohnsonCookParams)==200,"Pinned double-precision material layout changed");
static_assert(offsetof(JohnsonCookParams,IPLA)==168,"Pinned material integer field offset changed");
static_assert(offsetof(JohnsonCookParams,Z3)==184,"Pinned material trailing real offset changed");

namespace shell_spike {
namespace {
Vec3 Add(Vec3 a,Vec3 b){return {a.x+b.x,a.y+b.y,a.z+b.z};}
Vec3 Sub(Vec3 a,Vec3 b){return {a.x-b.x,a.y-b.y,a.z-b.z};}
Vec3 Mul(Vec3 a,double s){return {a.x*s,a.y*s,a.z*s};}
double Dot(Vec3 a,Vec3 b){return a.x*b.x+a.y*b.y+a.z*b.z;}
Vec3 Cross(Vec3 a,Vec3 b){return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};}
double Norm(Vec3 a){return std::hypot(a.x,a.y,a.z);}
bool Finite(Vec3 a){return std::isfinite(a.x)&&std::isfinite(a.y)&&std::isfinite(a.z);}

Attempt Invalid(Status status,const char* text,std::size_t bytes=0){return {status,text,bytes};}
Attempt Validate(const ElasticInput& in,const ElasticResult* committed) {
  if(!committed) return Invalid(Status::InvalidInput,"Null elastic committed result");
  if(!std::isfinite(in.half_x)||!std::isfinite(in.half_y)||
     in.half_x<1e-3||in.half_y<1e-3||in.half_x>1e3||in.half_y>1e3)
    return Invalid(Status::UnsupportedGeometry,"S2a half lengths must be in [1e-3,1e3]");
  if(!std::isfinite(in.dt)||in.dt<1e-8||in.dt>1||
     !std::isfinite(in.thickness)||in.thickness<1e-5||in.thickness>1||
     !Finite(in.center)||Norm(in.center)>1e6)
    return Invalid(Status::InvalidInput,"Invalid bounded S2a dt/thickness/center");
  for(Vec3 e:in.basis)
    if(!Finite(e)||std::abs(Norm(e)-1)>1e-12)
      return Invalid(Status::InvalidInput,"S2a basis must be finite orthonormal");
  if(std::abs(Dot(in.basis[0],in.basis[1]))>1e-12||
     Norm(Sub(Cross(in.basis[0],in.basis[1]),in.basis[2]))>1e-12)
    return Invalid(Status::InvalidInput,"S2a basis must be a proper orthonormal frame");
  double v=0,w=0;
  for(int i=0;i<4;++i) {
    if(!Finite(in.velocity[i])||!Finite(in.angular_velocity[i]))
      return Invalid(Status::InvalidInput,"Nonfinite S2a velocity input");
    v=std::max(v,Norm(in.velocity[i])); w=std::max(w,Norm(in.angular_velocity[i]));
  }
  // Conservative rectangle bound, independent of donor strain coefficients:
  // gradient <= 2*max(v)/min_half; rotations <= max(w), curvature <=2*w/min_half.
  // Extra factors bound all five stress components, engineering shear and the
  // donor's quadratic transverse-velocity correction. Accepted inputs are kept
  // far below the fixed yield stress; unsupported loading fails before CUDA.
  const double l=std::min(in.half_x,in.half_y);
  const double q=in.dt*v/l;
  const double bound=8*(q+q*q+in.dt*w+in.thickness*in.dt*w/l);
  const double stress_bound=8*kElasticYoung/(1-kElasticNu*kElasticNu)*bound;
  if(!std::isfinite(stress_bound)||stress_bound>kElasticYield*.1)
    return Invalid(Status::InvalidInput,"Loading exceeds conservative elastic-only stress bound");
  return {Status::Ok,{},0};
}

struct CudaFailure:std::runtime_error{using std::runtime_error::runtime_error;};
void Check(cudaError_t e,const char* operation){
  if(e!=cudaSuccess) throw CudaFailure(std::string(operation)+": "+cudaGetErrorString(e));
}
class Stream {
 public:
  Stream(){Check(cudaStreamCreateWithFlags(&value_,cudaStreamNonBlocking),"create S2a stream");}
  ~Stream(){if(value_)cudaStreamDestroy(value_);}
  Stream(const Stream&)=delete; Stream& operator=(const Stream&)=delete;
  operator cudaStream_t()const{return value_;}
 private:cudaStream_t value_=nullptr;
};
template<class T>class Device {
 public:
  explicit Device(std::size_t n){Check(cudaMalloc(reinterpret_cast<void**>(&p_),n*sizeof(T)),"allocate S2a slab");}
  ~Device(){if(p_)cudaFree(p_);}
  Device(const Device&)=delete;Device& operator=(const Device&)=delete;
  T* data()const{return p_;}
 private:T* p_=nullptr;
};

enum Field {
  Frame=0,Vel=9,Angular=21,Px1=33,Px2=34,Py1=35,Py2=36,Area=37,AreaInv=38,
  Vhx=39,Vhy=40,Z2=41,U=42,Sti=50,Stir=51,Young=52,Thk0=53,Off=54,Smstr=55,
  X=61,V=73,VR=85,Thk=97,Gstr=98,Energy=106,Epsd=108,
  Stress=109,Pla=124,Rate=127,Back=130,Dpla=139,Temp=142,
  Force=145,Moment=150,Sigy=153,Count=154
};
constexpr std::size_t kBytes=Count*sizeof(double)+4*sizeof(int);

Tensor WorldTensor(const std::array<Vec3,3>& f,double xx,double yy,double xy){
  const double u[]={f[0].x,f[0].y,f[0].z},v[]={f[1].x,f[1].y,f[1].z};
  Tensor t{};
  for(int i=0;i<3;++i)for(int j=0;j<3;++j)
    t[3*i+j]=xx*u[i]*u[j]+yy*v[i]*v[j]+xy*(u[i]*v[j]+v[i]*u[j]);
  return t;
}
}  // namespace

namespace detail {
Attempt S2A_ENTRY(const ElasticInput& in,ElasticResult* committed,const Options& options){
  Attempt valid=Validate(in,committed);if(valid.status!=Status::Ok)return valid;
  if(kBytes>options.max_device_bytes)
    return Invalid(Status::ResourceLimit,"S2a device budget exceeded",kBytes);
  try{
    std::array<double,Count> h{};
    const std::array<int,4> ids{{0,1,2,3}};
    const int sx[]={-1,1,1,-1},sy[]={-1,-1,1,1};
    auto put=[&](int field,int i,Vec3 v){h[field+3*i]=v.x;h[field+3*i+1]=v.y;h[field+3*i+2]=v.z;};
    for(int i=0;i<4;++i){
      put(X,i,Add(in.center,Add(Mul(in.basis[0],sx[i]*in.half_x),Mul(in.basis[1],sy[i]*in.half_y))));
      put(V,i,in.velocity[i]);put(VR,i,in.angular_velocity[i]);
    }
    h[Off]=1;h[Thk0]=in.thickness;h[Thk]=in.thickness;h[Young]=kElasticYoung;
    for(int ip=0;ip<3;++ip)h[Temp+ip]=kElasticTemperature;
    Stream stream;Device<double> storage(Count);Device<int> indices(4);
    auto d=[&](int field){return storage.data()+field;};
    auto c=[&](int i){return indices.data()+i;};
    Check(cudaMemcpyAsync(storage.data(),h.data(),sizeof(h),cudaMemcpyHostToDevice,stream),"upload S2a trial");
    Check(cudaMemcpyAsync(indices.data(),ids.data(),sizeof(ids),cudaMemcpyHostToDevice,stream),"upload S2a indices");
    shell_geometry_kernel<1><<<1,256,0,stream>>>(
      d(X),d(V),d(VR),c(0),c(1),c(2),c(3),d(Off),d(Smstr),
      d(Frame),d(Frame+1),d(Frame+2),d(Frame+3),d(Frame+4),d(Frame+5),d(Frame+6),d(Frame+7),d(Frame+8),
      d(Vel),d(Vel+1),d(Vel+2),d(Vel+3),d(Vel+4),d(Vel+5),d(Vel+6),d(Vel+7),d(Vel+8),d(Vel+9),d(Vel+10),d(Vel+11),
      d(Angular),d(Angular+1),d(Angular+2),d(Angular+3),d(Angular+4),d(Angular+5),d(Angular+6),d(Angular+7),d(Angular+8),d(Angular+9),d(Angular+10),d(Angular+11),
      d(Px1),d(Px2),d(Py1),d(Py2),d(Area),d(AreaInv),d(Vhx),d(Vhy),d(Z2),
      d(U),d(U+1),d(U+2),d(U+3),d(U+4),d(U+5),d(U+6),d(U+7),
      d(Sti),d(Stir),d(Young),d(Thk0),c(2),c(3),1,0.0,0.0);
    Check(cudaGetLastError(),"launch S2a K1");
    Check(cudaMemcpyAsync(h.data(),storage.data(),sizeof(h),cudaMemcpyDeviceToHost,stream),"download S2a K1");
    Check(cudaStreamSynchronize(stream),"synchronize S2a K1");
    if(!std::all_of(h.begin(),h.end(),[](double x){return std::isfinite(x);})||!(h[Area]>0))
      return Invalid(Status::InvalidOutput,"Nonfinite S2a geometry",kBytes);
    ElasticResult trial;
#ifdef SHELL_ELASTIC_GAUSS
    trial.rule=ThicknessRule::Gauss3;
#else
    trial.rule=ThicknessRule::OriginalMidpoint3;
#endif
    for(int j=0;j<3;++j){
      trial.frame[j]={h[Frame+3*j],h[Frame+3*j+1],h[Frame+3*j+2]};
      if(Norm(Sub(trial.frame[j],in.basis[j]))>1e-9)
        return Invalid(Status::InvalidOutput,"S2a rectangle frame differs from supplied basis",kBytes);
    }
    if(std::abs(h[Area]/(4*in.half_x*in.half_y)-1)>1e-9)
      return Invalid(Status::InvalidOutput,"S2a rectangle area mismatch",kBytes);

    JohnsonCookParams mat{};
    mat.E=kElasticYoung;mat.nu=kElasticNu;
    mat.G=mat.E/(2*(1+mat.nu));mat.A11=mat.E/(1-mat.nu*mat.nu);mat.A12=mat.nu*mat.A11;
    mat.CA=kElasticYield;mat.YMAX=kElasticYield;mat.CN=1;mat.EPDR=1;
    mat.M_EXP=1;mat.TREF=kElasticTemperature;mat.TMELT=kElasticTemperature;
    mat.ASRATE=1;mat.RHO=7800;mat.SSP=std::sqrt(mat.A11/mat.RHO);
    mat.SHF_COEF=kElasticShearFactor;mat.IPLA=0;mat.VP=1;mat.IFORM=0;mat.ICC=0;
    // CB, CC, FISOKIN, EPMX, RHOCP, Z3/Z4 remain explicit zero via value init.
    shell_strain_material_kernel<0,3,1><<<1,256,0,stream>>>(
      d(Frame),d(Frame+1),d(Frame+2),d(Frame+3),d(Frame+4),d(Frame+5),d(Frame+6),d(Frame+7),d(Frame+8),
      d(Vel),d(Vel+1),d(Vel+2),d(Vel+3),d(Vel+4),d(Vel+5),d(Vel+6),d(Vel+7),d(Vel+8),d(Vel+9),d(Vel+10),d(Vel+11),
      d(Angular),d(Angular+1),d(Angular+2),d(Angular+3),d(Angular+4),d(Angular+5),d(Angular+6),d(Angular+7),d(Angular+8),d(Angular+9),d(Angular+10),d(Angular+11),
      d(Px1),d(Px2),d(Py1),d(Py2),d(Area),d(AreaInv),
      d(U),d(U+1),d(U+2),d(U+3),d(U+4),d(U+5),d(U+6),d(U+7),
      d(Off),d(Thk),d(Thk0),d(Gstr),d(Energy),d(Epsd),
      d(Stress),d(Stress+3),d(Stress+6),d(Stress+9),d(Stress+12),
      d(Pla),d(Rate),d(Back),d(Back+3),d(Back+6),d(Dpla),d(Temp),
      d(Force),d(Force+1),d(Force+2),d(Force+3),d(Force+4),d(Moment),d(Moment+1),d(Moment+2),d(Sigy),
      mat,in.dt,1,0,100);
    Check(cudaGetLastError(),"launch S2a K2");
    Check(cudaMemcpyAsync(h.data(),storage.data(),sizeof(h),cudaMemcpyDeviceToHost,stream),"download S2a K2");
    Check(cudaStreamSynchronize(stream),"synchronize S2a K2");
    if(!std::all_of(h.begin(),h.end(),[](double x){return std::isfinite(x);})||!(h[Thk]>0))
      return Invalid(Status::InvalidOutput,"Nonfinite/nonphysical S2a state",kBytes);
    if(h[Off]!=2||h[Thk0]!=in.thickness||std::abs(h[Sigy]-kElasticYield)>1e-3)
      return Invalid(Status::InvalidOutput,"S2a activation/reference/yield state contract violated",kBytes);
    for(int ip=0;ip<3;++ip){
      auto& p=trial.points[ip];
      for(int j=0;j<5;++j)p.stress[j]=h[Stress+3*j+ip];
      p.plastic_strain=h[Pla+ip];p.plastic_increment=h[Dpla+ip];p.filtered_plastic_rate=h[Rate+ip];
      for(int j=0;j<3;++j)p.backstress[j]=h[Back+3*j+ip];
      p.temperature=h[Temp+ip];
      if(p.plastic_strain!=0||p.plastic_increment!=0||p.filtered_plastic_rate!=0||
         p.backstress!=std::array<double,3>{}||p.temperature!=kElasticTemperature)
        return Invalid(Status::InvalidOutput,"S2a elastic-only state contract violated",kBytes);
    }
    trial.area=h[Area];trial.thickness=h[Thk];trial.reference_thickness=h[Thk0];trial.off=h[Off];
    trial.mean_yield=h[Sigy];trial.element_rate=h[Epsd];trial.energy={h[Energy],h[Energy+1]};
    for(int i=0;i<8;++i)trial.generalized_increment[i]=h[Gstr+i];
    for(int i=0;i<5;++i){trial.normalized_forces[i]=h[Force+i];trial.forces[i]=h[Force+i]*in.thickness;}
    for(int i=0;i<3;++i){trial.normalized_moments[i]=h[Moment+i];trial.moments[i]=h[Moment+i]*in.thickness*in.thickness;}
    trial.membrane_world=WorldTensor(trial.frame,trial.forces[0],trial.forces[1],trial.forces[2]);
    trial.bending_world=WorldTensor(trial.frame,trial.moments[0],trial.moments[1],trial.moments[2]);
    trial.shear_world=Add(Mul(trial.frame[0],trial.forces[4]),Mul(trial.frame[1],trial.forces[3]));
    trial.device_bytes=kBytes;
    if(options.reject_after_assembly)
      return Invalid(Status::TrialRejected,"Injected S2a rejection after material integration",kBytes);
    *committed=std::move(trial);
    return {Status::Ok,{},kBytes};
  }catch(const CudaFailure& e){return {Status::CudaError,e.what(),kBytes};}
   catch(const std::bad_alloc&){return Invalid(Status::ResourceLimit,"S2a host allocation failed",kBytes);}
}
}  // namespace detail
}  // namespace shell_spike
