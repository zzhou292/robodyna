#pragma once

#include "PersistentShell.h"
#include <cuda_runtime.h>
#include <vector>

namespace tl::qualification::shell::detail {
// Packed SoA planes reproduce the qualified K1/K2/K3 contracts. Point arrays
// occupy three planes each. Nodal K1 input is interleaved; K3 output is SoA.
enum Field {
  Off=0,Smstr=1,Frame=7,Vel=16,Angular=28,Px1=40,Px2=41,Py1=42,Py2=43,
  Area=44,AreaInverse=45,Vhx=46,Vhy=47,Z2=48,U=49,Sti=57,Stir=58,
  Modulus=59,StepThickness=60,StepThicknessSquared=61,Sound=62,Rho=63,
  Nu=64,A11=65,ShearModulus=66,SectionShear=67,Force=68,Moment=73,
  Hour=76,Energy=81,Thickness=83,Gstr=84,Epsd=92,Stress=93,Pla=108,
  Rate=111,Back=114,Dpla=123,Temp=126,Sigy=129,ElementPlanes=130
};

// Runtime allocation/error handling is centralized here, independently of
// geometry/constitutive arithmetic and coordinator publication.
class DeviceStorage {
 public:
  explicit DeviceStorage(const Configuration&);
  ~DeviceStorage();
  DeviceStorage(const DeviceStorage&)=delete;
  DeviceStorage& operator=(const DeviceStorage&)=delete;
  void UploadInitial(const std::vector<double>&);
  void Prepare(fea::HostNodalKinematicsView);
  void Download(std::vector<double>&);
  void Synchronize();
  void Geometry(const Configuration&,std::uint64_t);
  void Material(const Configuration&,double);
  void Assembly(const Configuration&,double,std::uint64_t);
  void Publish() noexcept;
  std::size_t bytes() const noexcept {return bytes_;}
  std::size_t count() const noexcept {return count_;}
 private:
  void Release() noexcept;
  double* field(int f) const {return trial_+f*ne_;}
  double* node(int f) const {return trial_+ElementPlanes*ne_+f*nn_;}
  int* connectivity(int n) const {return connectivity_+n*ne_;}
  int ne_=0,nn_=0;
  std::size_t count_=0,bytes_=0;
  double *accepted_=nullptr,*trial_=nullptr;
  int* connectivity_=nullptr;
  cudaStream_t stream_=nullptr;
};

Report ValidateConfiguration(const Configuration&);
Report ValidateKinematics(const Configuration&,fea::HostNodalKinematicsView,bool initializing);
Report ValidateState(const Configuration&,const std::vector<double>&,bool material);
void Decode(const Configuration&,const std::vector<double>&,Snapshot&);
}  // namespace tl::qualification::shell::detail
