#include "PersistentShellStorage.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <type_traits>
#include <utility>

// Private numerical donor boundary. All three bodies are unchanged except the
// already qualified generated Gauss3 thickness rule in K2. No donor struct
// crosses the public TL nodal or persistent-fixture API.
#include "shell_geometry_kernel.cu"
#include "shell_strain_material_kernel_gauss3.cu"
#include "shell_force_assembly_kernel.cu"
static_assert(std::is_same<Real,double>::value,"Binary64 donor required");

namespace tl::qualification::shell::detail {
namespace {
void Check(cudaError_t e,const char* op) {
  if(e!=cudaSuccess) throw std::runtime_error(std::string(op)+": "+cudaGetErrorString(e));
}
HourglassParams Parameters(const Configuration& c) {
  const auto& p=c.stabilization;
  return {p.H1,p.H2,p.H3,p.SRH1,p.SRH2,p.SRH3,p.HVISC,p.HELAS,p.HVLIN};
}
JohnsonCookParams ElasticMaterial() {
  JohnsonCookParams m{};
  m.E=Young;m.nu=Poisson;m.G=m.E/(2*(1+m.nu));
  m.A11=m.E/(1-m.nu*m.nu);m.A12=m.nu*m.A11;
  m.CA=Yield;m.YMAX=Yield;m.CN=1;m.EPDR=1;
  m.M_EXP=1;m.TREF=Temperature;m.TMELT=Temperature;
  m.ASRATE=1;m.RHO=7800;m.SSP=std::sqrt(m.A11/m.RHO);
  m.SHF_COEF=ShearFactor;m.IPLA=0;m.VP=1;m.IFORM=0;m.ICC=0;
  return m;
}
__global__ void SnapshotThickness(double* step,double* squared,const double* current,int n) {
  const int i=blockIdx.x*blockDim.x+threadIdx.x;
  if(i<n){step[i]=current[i];squared[i]=current[i]*current[i];}
}
#include "PersistentShellLaunch.cuh"

template<int Mode,class D>
void LaunchMaterial(D d,int ne,const Configuration& c,double dt,cudaStream_t stream) {
  const auto mat=ElasticMaterial();
  shell_strain_material_kernel<0,3,Mode><<<1,256,0,stream>>>(
    d(Frame),d(Frame+1),d(Frame+2),d(Frame+3),d(Frame+4),d(Frame+5),d(Frame+6),d(Frame+7),d(Frame+8),
    d(Vel),d(Vel+1),d(Vel+2),d(Vel+3),d(Vel+4),d(Vel+5),d(Vel+6),d(Vel+7),d(Vel+8),d(Vel+9),d(Vel+10),d(Vel+11),
    d(Angular),d(Angular+1),d(Angular+2),d(Angular+3),d(Angular+4),d(Angular+5),d(Angular+6),d(Angular+7),d(Angular+8),d(Angular+9),d(Angular+10),d(Angular+11),
    d(Px1),d(Px2),d(Py1),d(Py2),d(Area),d(AreaInverse),
    d(U),d(U+1),d(U+2),d(U+3),d(U+4),d(U+5),d(U+6),d(U+7),
    d(Off),d(Thickness),d(StepThickness),d(Gstr),d(Energy),d(Epsd),
    d(Stress),d(Stress+3),d(Stress+6),d(Stress+9),d(Stress+12),
    d(Pla),d(Rate),d(Back),d(Back+3),d(Back+6),d(Dpla),d(Temp),
    d(Force),d(Force+1),d(Force+2),d(Force+3),d(Force+4),d(Moment),d(Moment+1),d(Moment+2),d(Sigy),
    mat,dt,ne,c.update_thickness?1:0,100);
}
}  // namespace

DeviceStorage::DeviceStorage(const Configuration& c)
  :ne_(static_cast<int>(c.element_count)),nn_(static_cast<int>(c.node_count)),
   count_(ElementPlanes*ne_+17*nn_),bytes_(2*count_*sizeof(double)+4*ne_*sizeof(int)) {
  try {
    Check(cudaStreamCreateWithFlags(&stream_,cudaStreamNonBlocking),"create persistent stream");
    Check(cudaMalloc(reinterpret_cast<void**>(&accepted_),count_*sizeof(double)),"allocate accepted slab");
    Check(cudaMalloc(reinterpret_cast<void**>(&trial_),count_*sizeof(double)),"allocate trial slab");
    Check(cudaMalloc(reinterpret_cast<void**>(&connectivity_),4*ne_*sizeof(int)),"allocate connectivity");
    std::array<int,4*MaxElements> ids{};
    for(int e=0;e<ne_;++e)for(int i=0;i<4;++i)ids[i*ne_+e]=c.elements[e].nodes[i];
    Check(cudaMemcpyAsync(connectivity_,ids.data(),4*ne_*sizeof(int),cudaMemcpyHostToDevice,stream_),"upload connectivity");
    Synchronize();
  } catch(...) {Release();throw;}
}
void DeviceStorage::Release() noexcept {
  if(stream_)cudaStreamSynchronize(stream_);
  if(connectivity_)cudaFree(connectivity_);
  if(trial_)cudaFree(trial_);
  if(accepted_)cudaFree(accepted_);
  if(stream_)cudaStreamDestroy(stream_);
}
DeviceStorage::~DeviceStorage(){Release();}
void DeviceStorage::Synchronize(){Check(cudaStreamSynchronize(stream_),"complete persistent phase");}
void DeviceStorage::UploadInitial(const std::vector<double>& host) {
  Check(cudaMemcpyAsync(trial_,host.data(),count_*sizeof(double),cudaMemcpyHostToDevice,stream_),"upload initial state");
}
void DeviceStorage::Prepare(fea::HostNodalKinematicsView in) {
  Check(cudaMemcpyAsync(trial_,accepted_,count_*sizeof(double),cudaMemcpyDeviceToDevice,stream_),"copy accepted to trial");
  for(auto pair:{std::make_pair(0,in.position_xyz),std::make_pair(3,in.velocity_xyz),std::make_pair(6,in.angular_velocity_xyz)})
    Check(cudaMemcpyAsync(node(pair.first),pair.second,3*nn_*sizeof(double),cudaMemcpyHostToDevice,stream_),"upload prescribed kinematics");
  // The global coordinator clears once. Every element in this batch accumulates
  // into these shared physical nodes. Preserve FOR/MOM and all point histories.
  Check(cudaMemsetAsync(node(9),0,8*nn_*sizeof(double),stream_),"clear nodal assembly");
}
void DeviceStorage::Download(std::vector<double>& host) {
  Check(cudaMemcpyAsync(host.data(),trial_,count_*sizeof(double),cudaMemcpyDeviceToHost,stream_),"read qualification state");
  Synchronize();
}
void DeviceStorage::Geometry(const Configuration& config,std::uint64_t epoch) {
  if(config.update_thickness){
    SnapshotThickness<<<1,32,0,stream_>>>(field(StepThickness),field(StepThicknessSquared),field(Thickness),ne_);
    Check(cudaGetLastError(),"snapshot accepted thickness");
  }
  const fea::DeviceNodalKinematicsView view{node(0),node(3),node(6),static_cast<std::size_t>(nn_),epoch};
  auto d=[&](int f){return field(f);};
  auto n=[&](int f)->const double* {
    return f==0?view.position_xyz:(f==3?view.velocity_xyz:view.angular_velocity_xyz);
  };
  auto c=[&](int i){return connectivity(i);};
  if(config.geometry==GeometryMode::FrozenReference)LaunchGeometry<1>(d,n,c,ne_,Parameters(config),stream_);
  else LaunchGeometry<2>(d,n,c,ne_,Parameters(config),stream_);
  Check(cudaGetLastError(),"launch persistent K1");
}
void DeviceStorage::Material(const Configuration& config,double dt) {
  auto d=[&](int f){return field(f);};
  if(config.geometry==GeometryMode::FrozenReference)LaunchMaterial<1>(d,ne_,config,dt,stream_);
  else LaunchMaterial<2>(d,ne_,config,dt,stream_);
  Check(cudaGetLastError(),"launch persistent K2");
}
void DeviceStorage::Assembly(const Configuration& config,double dt,std::uint64_t epoch) {
  const fea::DeviceNodalForceView view{node(9),node(10),node(11),node(12),node(13),node(14),static_cast<std::size_t>(nn_),epoch};
  auto d=[&](int f){return field(f);};
  auto n=[&](int f){
    const std::array<double*,8> output{view.force_x,view.force_y,view.force_z,view.couple_x,view.couple_y,view.couple_z,node(15),node(16)};
    return output[f-9];
  };
  auto c=[&](int i){return connectivity(i);};
  if(config.geometry==GeometryMode::FrozenReference)LaunchForces<1>(d,n,c,ne_,Parameters(config),dt,stream_);
  else LaunchForces<2>(d,n,c,ne_,Parameters(config),dt,stream_);
  Check(cudaGetLastError(),"launch persistent K3");
}
void DeviceStorage::Publish() noexcept {std::swap(accepted_,trial_);}
}  // namespace tl::qualification::shell::detail
