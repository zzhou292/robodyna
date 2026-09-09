#include "ShellFixture.h"

#include <cuda_runtime.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <new>
#include <stdexcept>
#include <utility>

// Unmodified, pinned OpenRadioss sources. Their copyright/license notices remain
// in the immutable donor files and adjacent LICENSE.md/source-manifest.json.
#include "shell_geometry_kernel.h"
#include "shell_force_assembly_kernel.h"
#include "shell_geometry_kernel.cu"
#include "shell_force_assembly_kernel.cu"

static_assert(std::is_same<Real, double>::value, "Fixture requires MYREAL8");

namespace shell_spike {
namespace {

Vec3 Sub(Vec3 a, Vec3 b) { return {a.x-b.x, a.y-b.y, a.z-b.z}; }
Vec3 Mul(Vec3 a, double s) { return {a.x*s, a.y*s, a.z*s}; }
double Dot(Vec3 a, Vec3 b) { return a.x*b.x+a.y*b.y+a.z*b.z; }
Vec3 Cross(Vec3 a, Vec3 b) {
  return {a.y*b.z-a.z*b.y, a.z*b.x-a.x*b.z, a.x*b.y-a.y*b.x};
}
double Norm(Vec3 a) { return std::hypot(a.x, a.y, a.z); }
bool Finite(Vec3 a) {
  return std::isfinite(a.x) && std::isfinite(a.y) && std::isfinite(a.z);
}
Vec3 Apply(const Tensor& a, Vec3 b) {
  return {a[0]*b.x+a[1]*b.y+a[2]*b.z,
          a[3]*b.x+a[4]*b.y+a[5]*b.z,
          a[6]*b.x+a[7]*b.y+a[8]*b.z};
}

Attempt Fail(Status status, const char* message, std::size_t bytes = 0) {
  return {status, message, bytes};
}

bool TangentTensor(const Tensor& a, Vec3 n) {
  double scale = 1;
  for (double v : a) {
    if (!std::isfinite(v)) return false;
    scale = std::max(scale, std::abs(v));
  }
  // Scale first, so validation does not overflow on finite large resultants.
  Tensor scaled{};
  for (int i=0;i<9;++i) scaled[i]=a[i]/scale;
  const double eps = 1e-10;
  return std::abs(scaled[1]-scaled[3]) <= eps &&
         std::abs(scaled[2]-scaled[6]) <= eps &&
         std::abs(scaled[5]-scaled[7]) <= eps &&
         Norm(Apply(scaled, n)) <= eps;
}

Attempt Validate(const Input& in, const Result* committed) {
  if (!committed) return Fail(Status::InvalidInput, "Null committed result");
  if (in.elements.empty()) return Fail(Status::EmptyInput, "No elements");
  if (in.elements.size()>16 || in.positions.size()>64)
    return Fail(Status::ResourceLimit, "Fixture allows at most 16 elements / 64 nodes");
  if (in.positions.size()<4 || in.velocity.size()!=in.positions.size() ||
      in.angular_velocity.size()!=in.positions.size())
    return Fail(Status::InvalidInput, "Position/velocity node counts differ");
  for (std::size_t i=0;i<in.positions.size();++i)
    if (!Finite(in.positions[i]) || !Finite(in.velocity[i]) ||
        !Finite(in.angular_velocity[i]))
      return Fail(Status::InvalidInput, "Nonfinite nodal input");
  for (const Element& e : in.elements) {
    if (!std::isfinite(e.thickness) || !(e.thickness>0) ||
        !std::isfinite(e.thickness*e.thickness) || !(e.thickness*e.thickness>0))
      return Fail(Status::InvalidInput, "Invalid thickness");
    for (int i=0;i<4;++i) {
      if (e.nodes[i]<0 || static_cast<std::size_t>(e.nodes[i])>=in.positions.size())
        return Fail(Status::InvalidInput, "Connectivity index out of range");
      for (int j=0;j<i;++j)
        if (e.nodes[i]==e.nodes[j])
          return Fail(Status::UnsupportedGeometry, "Native triangle/repeated node is not Q4");
    }
    std::array<Vec3,4> p{};
    for (int i=0;i<4;++i) p[i]=in.positions[e.nodes[i]];
    double scale=0;
    for (int i=0;i<4;++i) scale=std::max(scale, Norm(Sub(p[(i+1)%4],p[i])));
    // This qualification fixture deliberately avoids the donor's absolute
    // reciprocal cutoffs and extreme arithmetic scales; not a production limit.
    if (!std::isfinite(scale) || scale<1e-6 || scale>1e6)
      return Fail(Status::UnsupportedGeometry, "Fixture edge scale outside [1e-6,1e6]");
    Vec3 n=Cross(Mul(Sub(p[1],p[0]),1/scale), Mul(Sub(p[2],p[0]),1/scale));
    const double normal_length=Norm(n);
    if (!(normal_length>1e-10))
      return Fail(Status::UnsupportedGeometry, "Degenerate quad plane");
    n=Mul(n,1/normal_length);
    if (std::abs(Dot(Mul(Sub(p[3],p[0]),1/scale),n))>1e-10)
      return Fail(Status::UnsupportedGeometry, "Warped quad is outside planar fixture scope");
    for (int i=0;i<4;++i) {
      Vec3 a=Mul(Sub(p[(i+1)%4],p[i]),1/scale);
      Vec3 b=Mul(Sub(p[(i+2)%4],p[(i+1)%4]),1/scale);
      if (!(Dot(Cross(a,b),n)>1e-10))
        return Fail(Status::UnsupportedGeometry, "Quad must be strictly convex in perimeter order");
    }
    if (!TangentTensor(e.membrane,n) || !TangentTensor(e.bending,n) || !Finite(e.shear))
      return Fail(Status::InvalidInput, "Resultants must be finite symmetric tangent tensors");
    const double shear_scale=std::max({1.0,std::abs(e.shear.x),std::abs(e.shear.y),std::abs(e.shear.z)});
    if (std::abs(Dot(Mul(e.shear,1/shear_scale),n))>1e-10)
      return Fail(Status::InvalidInput, "Shear resultant must be tangent");
    for (double v : e.membrane)
      if (!std::isfinite(v/e.thickness))
        return Fail(Status::InvalidInput, "Membrane/thickness normalization overflow");
    for (double v : e.bending)
      if (!std::isfinite(v/(e.thickness*e.thickness)))
        return Fail(Status::InvalidInput, "Moment/thickness normalization overflow");
    if (!Finite(Mul(e.shear,1/e.thickness)))
      return Fail(Status::InvalidInput, "Shear/thickness normalization overflow");
  }
  return {Status::Ok, {}, 0};
}

struct CudaFailure : std::runtime_error { using std::runtime_error::runtime_error; };
void Check(cudaError_t e, const char* operation) {
  if (e!=cudaSuccess)
    throw CudaFailure(std::string(operation)+": "+cudaGetErrorString(e));
}

class Stream {
 public:
  Stream() { Check(cudaStreamCreateWithFlags(&value_,cudaStreamNonBlocking),"create stream"); }
  ~Stream() { if (value_) cudaStreamDestroy(value_); }
  Stream(const Stream&)=delete;
  Stream& operator=(const Stream&)=delete;
  operator cudaStream_t() const { return value_; }
 private:
  cudaStream_t value_=nullptr;
};

template <class T> class DeviceArray {
 public:
  explicit DeviceArray(std::size_t count) {
    Check(cudaMalloc(reinterpret_cast<void**>(&value_),count*sizeof(T)),"allocate fixture slab");
  }
  ~DeviceArray() { if (value_) cudaFree(value_); }
  DeviceArray(const DeviceArray&)=delete;
  DeviceArray& operator=(const DeviceArray&)=delete;
  T* data() const { return value_; }
 private:
  T* value_=nullptr;
};

// Element scalar planes. Multi-component fields occupy consecutive SoA planes.
enum Field {
  Off=0, Smstr=1, Frame=7, Vel=16, Angular=28, Px1=40, Px2=41,
  Py1=42, Py2=43, Area=44, AreaInverse=45, Vhx=46, Vhy=47, Z2=48,
  U=49, Sti=57, Stir=58, Young=59, Thickness=60, ThicknessSquared=61,
  Sound=62, Rho=63, Nu=64, A11=65, ShearModulus=66, ShearFactor=67,
  Force=68, Moment=73, Hour=76, Energy=81, ElementPlanes=83
};

}  // namespace

Attempt Evaluate(const Input& in, Result* committed, const Options& options) {
  Attempt valid=Validate(in,committed);
  if (valid.status!=Status::Ok) return valid;
  const int ne=static_cast<int>(in.elements.size());
  const int nn=static_cast<int>(in.positions.size());
  const std::size_t scalar_count=ElementPlanes*ne+17*nn;
  const std::size_t bytes=scalar_count*sizeof(double)+4*ne*sizeof(int);
  if (bytes>options.max_device_bytes)
    return Fail(Status::ResourceLimit,"Fixture device-byte cap exceeded",bytes);
  try {
    // Entire slab starts at zero: atomic outputs, OFF/SMSTR trial state,
    // hourglass histories, unused branches, and energy all have explicit values.
    std::vector<double> host(scalar_count,0.0);
    std::vector<int> connectivity(4*ne);
    auto h=[&](int field, int element)->double& { return host[field*ne+element]; };
    const int node_start=ElementPlanes*ne;
    auto put_aos=[&](int offset,const std::vector<Vec3>& values) {
      for (int i=0;i<nn;++i) {
        host[node_start+offset*nn+3*i]=values[i].x;
        host[node_start+offset*nn+3*i+1]=values[i].y;
        host[node_start+offset*nn+3*i+2]=values[i].z;
      }
    };
    put_aos(0,in.positions); put_aos(3,in.velocity); put_aos(6,in.angular_velocity);
    for (int i=0;i<ne;++i) {
      for (int j=0;j<4;++j) connectivity[j*ne+i]=in.elements[i].nodes[j];
      h(Off,i)=1;
      h(Thickness,i)=in.elements[i].thickness;
      h(ThicknessSquared,i)=in.elements[i].thickness*in.elements[i].thickness;
      // Finite, dimensionally consistent dummy elastic scalars. No material
      // update/stiffness assertion: HG and compute_sti are both disabled.
      h(Young,i)=1000; h(Nu,i)=0.25; h(Rho,i)=1;
      h(A11,i)=1000/(1-0.25*0.25); h(ShearModulus,i)=400;
      h(Sound,i)=std::sqrt(h(A11,i)); h(ShearFactor,i)=5.0/6.0;
    }
    Stream stream;
    DeviceArray<double> storage(scalar_count);
    DeviceArray<int> indices(connectivity.size());
    auto d=[&](int field) { return storage.data()+field*ne; };
    auto node=[&](int offset) { return storage.data()+node_start+offset*nn; };
    auto c=[&](int component) { return indices.data()+component*ne; };
    Check(cudaMemcpyAsync(storage.data(),host.data(),scalar_count*sizeof(double),cudaMemcpyHostToDevice,stream),"upload slab");
    Check(cudaMemcpyAsync(indices.data(),connectivity.data(),connectivity.size()*sizeof(int),cudaMemcpyHostToDevice,stream),"upload indices");

    // Call the unchanged donor kernel directly. Its host launcher exits the
    // process on failure; this checked wrapper instead preserves caller state.
    shell_geometry_kernel<1><<<1,256,0,stream>>>(
      node(0),node(3),node(6),c(0),c(1),c(2),c(3),d(Off),d(Smstr),
      d(Frame),d(Frame+1),d(Frame+2),d(Frame+3),d(Frame+4),d(Frame+5),d(Frame+6),d(Frame+7),d(Frame+8),
      d(Vel),d(Vel+1),d(Vel+2),d(Vel+3),d(Vel+4),d(Vel+5),d(Vel+6),d(Vel+7),d(Vel+8),d(Vel+9),d(Vel+10),d(Vel+11),
      d(Angular),d(Angular+1),d(Angular+2),d(Angular+3),d(Angular+4),d(Angular+5),d(Angular+6),d(Angular+7),d(Angular+8),d(Angular+9),d(Angular+10),d(Angular+11),
      d(Px1),d(Px2),d(Py1),d(Py2),d(Area),d(AreaInverse),d(Vhx),d(Vhy),d(Z2),
      d(U),d(U+1),d(U+2),d(U+3),d(U+4),d(U+5),d(U+6),d(U+7),
      d(Sti),d(Stir),d(Young),d(Thickness),c(2),c(3),ne,0.0,0.0);
    Check(cudaGetLastError(),"launch K1");
    Check(cudaMemcpyAsync(host.data(),storage.data(),scalar_count*sizeof(double),cudaMemcpyDeviceToHost,stream),"download K1");
    Check(cudaStreamSynchronize(stream),"synchronize K1");

    Result trial;
    trial.device_bytes=bytes;
    trial.frames.resize(ne); trial.areas.resize(ne);
    for (int i=0;i<ne;++i) {
      auto& f=trial.frames[i];
      for (int j=0;j<3;++j) f[j]={h(Frame+3*j,i),h(Frame+3*j+1,i),h(Frame+3*j+2,i)};
      if (!Finite(f[0]) || !Finite(f[1]) || !Finite(f[2]) ||
          std::abs(Norm(f[0])-1)>1e-10 || std::abs(Norm(f[1])-1)>1e-10 ||
          std::abs(Norm(f[2])-1)>1e-10 || std::abs(Dot(f[0],f[1]))>1e-10 ||
          Norm(Sub(Cross(f[0],f[1]),f[2]))>1e-10 ||
          !std::isfinite(h(Area,i)) || !(h(Area,i)>0))
        return Fail(Status::InvalidOutput,"K1 returned invalid frame/area",bytes);
      trial.areas[i]=h(Area,i);
      // Rotate the prescribed WORLD physical tensors into the actual K1 frame.
      // This is only an input adapter; test expectations use boundary traction,
      // never K1 derivatives or this frame construction as their force oracle.
      const Element& e=in.elements[i];
      h(Force,i)=Dot(f[0],Apply(e.membrane,f[0]))/e.thickness;
      h(Force+1,i)=Dot(f[1],Apply(e.membrane,f[1]))/e.thickness;
      h(Force+2,i)=Dot(f[0],Apply(e.membrane,f[1]))/e.thickness;
      h(Force+3,i)=Dot(e.shear,f[1])/e.thickness;
      h(Force+4,i)=Dot(e.shear,f[0])/e.thickness;
      const double t2=e.thickness*e.thickness;
      h(Moment,i)=Dot(f[0],Apply(e.bending,f[0]))/t2;
      h(Moment+1,i)=Dot(f[1],Apply(e.bending,f[1]))/t2;
      h(Moment+2,i)=Dot(f[0],Apply(e.bending,f[1]))/t2;
    }
    if (!std::all_of(host.begin(),host.end(),[](double v){return std::isfinite(v);}))
      return Fail(Status::InvalidOutput,"Nonfinite K1 or normalized resultants",bytes);
    // Upload only the eight prescribed resultant planes. Preserve K1 trial data.
    Check(cudaMemcpyAsync(d(Force),host.data()+Force*ne,8*ne*sizeof(double),cudaMemcpyHostToDevice,stream),"upload prescribed resultants");
    HourglassParams hg{};
    shell_force_assembly_kernel<1><<<1,256,0,stream>>>(
      c(0),c(1),c(2),c(3),
      d(Frame),d(Frame+1),d(Frame+2),d(Frame+3),d(Frame+4),d(Frame+5),d(Frame+6),d(Frame+7),d(Frame+8),
      d(Px1),d(Px2),d(Py1),d(Py2),d(Area),d(Vhx),d(Vhy),
      d(Vel),d(Vel+1),d(Vel+2),d(Vel+3),d(Vel+4),d(Vel+5),d(Vel+6),d(Vel+7),d(Vel+8),d(Vel+9),d(Vel+10),d(Vel+11),
      d(Angular),d(Angular+1),d(Angular+2),d(Angular+3),d(Angular+4),d(Angular+5),d(Angular+6),d(Angular+7),d(Angular+8),d(Angular+9),d(Angular+10),d(Angular+11),
      d(Off),d(Thickness),d(ThicknessSquared),
      d(Force),d(Force+1),d(Force+2),d(Force+3),d(Force+4),d(Moment),d(Moment+1),d(Moment+2),
      d(Hour),d(Sti),d(Stir),d(Sound),d(Rho),d(Young),d(Nu),d(A11),d(ShearModulus),d(ShearFactor),d(Energy),
      node(9),node(10),node(11),node(12),node(13),node(14),node(15),node(16),hg,0.01,3,ne,0,0);
    Check(cudaGetLastError(),"launch K3");
    Check(cudaMemcpyAsync(host.data(),storage.data(),scalar_count*sizeof(double),cudaMemcpyDeviceToHost,stream),"download K3");
    Check(cudaStreamSynchronize(stream),"synchronize K3");
    if (!std::all_of(host.begin(),host.end(),[](double v){return std::isfinite(v);}))
      return Fail(Status::InvalidOutput,"Nonfinite K3 output",bytes);
    trial.forces.resize(nn); trial.moments.resize(nn);
    for (int i=0;i<nn;++i) {
      trial.forces[i]={host[node_start+9*nn+i],host[node_start+10*nn+i],host[node_start+11*nn+i]};
      trial.moments[i]={host[node_start+12*nn+i],host[node_start+13*nn+i],host[node_start+14*nn+i]};
    }
    auto copy_field=[&](int field,int planes) {
      return std::vector<double>(host.begin()+field*ne,host.begin()+(field+planes)*ne);
    };
    trial.off=copy_field(Off,1); trial.reference_coordinates=copy_field(Smstr,6);
    trial.hourglass=copy_field(Hour,5); trial.energy=copy_field(Energy,2);
    if (options.reject_after_assembly)
      return Fail(Status::TrialRejected,"Injected fixture rejection after successful assembly",bytes);
    *committed=std::move(trial);
    return {Status::Ok,{},bytes};
  } catch (const CudaFailure& e) {
    return {Status::CudaError,e.what(),bytes};
  } catch (const std::bad_alloc&) {
    return Fail(Status::ResourceLimit,"Host fixture allocation failed",bytes);
  }
}

}  // namespace shell_spike
