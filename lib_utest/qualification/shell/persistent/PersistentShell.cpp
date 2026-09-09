#include "PersistentShellStorage.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <new>
#include <stdexcept>
#include <type_traits>
#include <utility>

namespace tl::qualification::shell {
using namespace detail;
struct PersistentShell::Impl {
  explicit Impl(const Configuration& c):configuration(c),device(c),host(device.count()){}
  Configuration configuration;
  DeviceStorage device;
  std::vector<double> host;
  Snapshot accepted{},candidate{};
  std::uint64_t attempt=0;
  bool trial_valid=false,usable=true;
};

PersistentShell::PersistentShell()=default;
PersistentShell::~PersistentShell()=default;
const Snapshot& PersistentShell::accepted()const noexcept{return impl_?impl_->accepted:empty_;}
const Snapshot* PersistentShell::trial()const noexcept{return impl_&&impl_->trial_valid?&impl_->candidate:nullptr;}
void PersistentShell::Discard()noexcept{if(impl_)impl_->trial_valid=false;}

Report PersistentShell::Initialize(const Configuration& c,fea::HostNodalKinematicsView in) {
  if(impl_)return {Status::InvalidInput,"Owner already initialized"};
  auto status=ValidateConfiguration(c);if(status.status!=Status::Ok)return status;
  status=ValidateKinematics(c,in,true);if(status.status!=Status::Ok)return status;
  try {
    auto next=std::make_unique<Impl>(c);
    auto& h=next->host;const int ne=static_cast<int>(c.element_count),nn=static_cast<int>(c.node_count);
    auto f=[&](int field,int e)->double&{return h[field*ne+e];};
    for(int e=0;e<ne;++e){
      const double t=c.elements[e].thickness;
      f(Off,e)=1;f(Modulus,e)=Young;f(StepThickness,e)=t;f(StepThicknessSquared,e)=t*t;f(Thickness,e)=t;
      f(Rho,e)=7800;f(Nu,e)=Poisson;f(A11,e)=Young/(1-Poisson*Poisson);
      f(ShearModulus,e)=Young/(2*(1+Poisson));f(Sound,e)=std::sqrt(f(A11,e)/f(Rho,e));
      f(SectionShear,e)=ShearFactor;f(Sigy,e)=Yield;
      for(int ip=0;ip<3;++ip)f(Temp+ip,e)=Temperature;
    }
    const int base=ElementPlanes*ne;
    std::copy_n(in.position_xyz,3*nn,h.data()+base);
    std::copy_n(in.velocity_xyz,3*nn,h.data()+base+3*nn);
    std::copy_n(in.angular_velocity_xyz,3*nn,h.data()+base+6*nn);
    next->device.UploadInitial(h);
    // Reference geometry is captured at X0, before the first deformed X1.
    // Initialization does not call K2/K3 or advance stresses/work/physical time.
    next->device.Geometry(c,0);next->device.Download(h);
    status=ValidateState(c,h,false);if(status.status!=Status::Ok)return status;
    Decode(c,h,next->accepted);next->accepted.device_bytes=next->device.bytes();
    next->device.Publish();impl_=std::move(next);
    return {Status::Ok,{}};
  }catch(const std::bad_alloc&){return {Status::ResourceLimit,"Host allocation failed"};}
   catch(const std::runtime_error& e){return {Status::DeviceFailure,e.what()};}
}

Report PersistentShell::Evaluate(const Request& r,TrialToken* token) {
  Discard();
  if(token)*token={};
  if(!impl_||!token)return {Status::InvalidInput,"Uninitialized owner or missing token"};
  auto& s=*impl_;
  if(!s.usable)return {Status::DeviceFailure,"Previous CUDA failure invalidated device owner"};
  if(s.attempt==std::numeric_limits<std::uint64_t>::max())return {Status::HistoryLimit,"Attempt counter exhausted"};
  ++s.attempt;
  const double dt=r.time_end-r.time_begin;
  if(!std::isfinite(r.time_begin)||!std::isfinite(r.time_end)||r.time_begin!=s.accepted.time||
     !std::isfinite(dt)||dt<1e-8||dt>1)
    return {Status::InvalidInput,"Request must continue the accepted completed interval"};
  if(s.accepted.epoch>=128)return {Status::HistoryLimit,"128 accepted increments reached"};
  if(r.reject_after!=RejectAfter::None&&r.reject_after!=RejectAfter::Geometry&&
     r.reject_after!=RejectAfter::Material&&r.reject_after!=RejectAfter::Assembly)
    return {Status::InvalidInput,"Unknown rejection phase"};
  auto status=ValidateKinematics(s.configuration,r.kinematics,false);
  if(status.status!=Status::Ok)return status;
  try {
    s.device.Prepare(r.kinematics);
    s.device.Geometry(s.configuration,s.accepted.epoch);s.device.Download(s.host);
    status=ValidateState(s.configuration,s.host,false);if(status.status!=Status::Ok)return status;
    if(r.reject_after==RejectAfter::Geometry)return {Status::Rejected,"Rejected after geometry"};

    s.device.Material(s.configuration,dt);s.device.Download(s.host);
    status=ValidateState(s.configuration,s.host,true);if(status.status!=Status::Ok)return status;
    std::array<double,MaxElements> material_total{};
    const int ne=static_cast<int>(s.configuration.element_count);
    for(int e=0;e<ne;++e)material_total[e]=s.host[Energy*ne+e]+s.host[(Energy+1)*ne+e];
    if(r.reject_after==RejectAfter::Material)return {Status::Rejected,"Rejected after material"};

    s.device.Assembly(s.configuration,dt,s.accepted.epoch);s.device.Download(s.host);
    status=ValidateState(s.configuration,s.host,true);if(status.status!=Status::Ok)return status;
    if(r.reject_after==RejectAfter::Assembly)return {Status::Rejected,"Rejected after assembly"};
    Snapshot candidate{};Decode(s.configuration,s.host,candidate);
    candidate.epoch=s.accepted.epoch+1;candidate.time=r.time_end;
    candidate.completed_dt=dt;candidate.device_bytes=s.device.bytes();
    for(int e=0;e<ne;++e){
      auto& out=candidate.elements[e];const auto& old=s.accepted.elements[e];
      out.material_work_increment=material_total[e]-old.work[0]-old.work[1];
      out.hourglass_work_increment=out.work[0]+out.work[1]-material_total[e];
    }
    s.candidate=candidate;s.trial_valid=true;
    token->owner=this;token->epoch=s.accepted.epoch;token->attempt=s.attempt;
    return {Status::Ok,{}};
  }catch(const std::bad_alloc&){return {Status::ResourceLimit,"Host diagnostic allocation failed"};}
   catch(const std::runtime_error& e){
     // Conservative terminal policy. Host accepted metadata survives; no claim
     // that a corrupted CUDA context can be retried safely in the same owner.
     s.usable=false;return {Status::DeviceFailure,e.what()};
   }
}

Status PersistentShell::Commit(const TrialToken& t)noexcept {
  if(!impl_)return Status::NoTrial;
  auto& s=*impl_;
  if(t.owner!=this||t.epoch!=s.accepted.epoch||t.attempt!=s.attempt)return Status::StaleTrial;
  if(!s.trial_valid)return Status::NoTrial;
  static_assert(std::is_nothrow_swappable<Snapshot>::value,"Atomic publication cannot allocate");
  s.device.Publish();using std::swap;swap(s.accepted,s.candidate);s.trial_valid=false;
  return Status::Ok;
}
}  // namespace tl::qualification::shell
