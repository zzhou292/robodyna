// SPDX-License-Identifier: AGPL-3.0-or-later
#include "ShellBatchPublicationValues.h"
#include "ShellBatchPublicationStorage.h"
#include "qeph/QephBatchStorage.h"
#include "t3/T3BatchStorage.h"
#include "qbat/QbatBatchIdentity.h"
#include "../../lib_utils/BoundedArena.h"
#include <cstring>

namespace tl::fea {
namespace shell_publication_detail {
using S=ShellPublicationStatus;
ShellPublicationReport Qbat(const qbat::BatchReport& report) noexcept {
  if(report.status==qbat::BatchStatus::Success) return {S::Success,"OK"};
  if(report.status==qbat::BatchStatus::DeviceFailure) return {S::DeviceFailure,report.message,report.nodal_status};
  if(report.status==qbat::BatchStatus::NodalFailure) return {S::NodalFailure,report.message,report.nodal_status};
  return {S::StaleTrial,report.message,report.nodal_status};
}
ShellPublicationReport Ok() noexcept { return {S::Success,"OK"}; }
ShellPublicationReport Nodal(const NodalReport& r) noexcept { return {S::NodalFailure,r.message,r.status}; }
ShellPublicationReport Connector(const type25::BatchReport& r) noexcept {
  using B=type25::BatchStatus;
  if(r.status==B::Success) return Ok();
  if(r.status==B::DeviceFailure||r.status==B::Unusable) return {S::DeviceFailure,r.message,r.nodal_status};
  if(r.status==B::NodalFailure) return {S::NodalFailure,r.message,r.nodal_status};
  return {S::StaleTrial,r.message,r.nodal_status};
}
bool SameDouble(double a,double b) noexcept { return std::memcmp(&a,&b,sizeof(a))==0; }
bool SameKinetic(const ShellBatchKinetic& a,const ShellBatchKinetic& b) noexcept {
  return SameDouble(a.translation,b.translation)&&SameDouble(a.rotation,b.rotation)&&
    SameDouble(a.physical_isotropic,b.physical_isotropic)&&SameDouble(a.added_isotropic,b.added_isotropic)&&
    SameDouble(a.connector_translation,b.connector_translation)&&SameDouble(a.connector_rotation,b.connector_rotation);
}
bool SameDiagnostics(const ShellBatchDiagnostics& a,const ShellBatchDiagnostics& b) noexcept {
  return a.valid==b.valid&&a.has_connector==b.has_connector&&
    a.has_qeph==b.has_qeph&&a.has_t3==b.has_t3&&a.has_qbat==b.has_qbat&&
    qbat::batch_detail::SameDiagnostics(a.qbat,b.qbat)&&
    type25::batch_detail::SameDiagnostics(a.connector,b.connector)&&qeph::batch_detail::SameDiagnostics(a.qeph,b.qeph)&&
    t3::batch_detail::SameDiagnostics(a.t3,b.t3)&&SameKinetic(a.base_kinetic,b.base_kinetic)&&SameKinetic(a.kinetic,b.kinetic);
}
ShellPublicationReport InitialMovingKinetic(FENodalState& owner,const ShellBatchBinding& binding,
    const NodalMassBinding* combined,const ShellBatchStartup& startup,
    const NodalStamp& expected,ShellBatchKinetic& output) {
  // Initial assembly views have expired after caller Discard. Authenticate
  // those source identities separately, then obtain fresh actual owner fields
  // through its accepted-only readback contract; never dereference old views.
  const auto count=binding.node_count();
  // The caller has preflighted this complete 13-double/node temporary payload.
  util::HostArena fields; util::BoundedArenaLayout layout(13*count*sizeof(double));
  util::ArenaRegion xr,vr,wr,qr;
  if(!layout.Append<double>(3*count,xr)||!layout.Append<double>(3*count,vr)||
     !layout.Append<double>(3*count,wr)||!layout.Append<double>(4*count,qr)||!fields.Initialize(layout.bytes()))
    return {S::ResourceLimit,"Initial kinetic readback staging allocation failed"};
  auto* x=fields.Construct<double>(xr); auto* v=fields.Construct<double>(vr);
  auto* w=fields.Construct<double>(wr); auto* orientation=fields.Construct<double>(qr);
  NodalStamp stamp;
  const auto copied=owner.CopyAccepted({x,v,count,orientation,w},&stamp);
  if(copied.status!=NodalStatus::Ok) return Nodal(copied);
  if(!trial_identity::SameStamp(stamp,expected)||stamp.epoch||stamp.time!=0||stamp.velocity_time!=0||stamp.node_count!=binding.node_count()||
     stamp.temporal_scheme!=NodalTemporalScheme::StaggeredHalfKickStart||stamp.velocity_phase!=NodalVelocityPhase::Collocated)
    return {S::StaleTrial,"Common initial kinetic requires the actual epoch-zero physical owner"};
  ShellBatchKinetic measured;
  for(std::size_t n=0;n<binding.node_count();++n) {
    const tl::math::Vec3 position{x[3*n],x[3*n+1],x[3*n+2]},velocity{v[3*n],v[3*n+1],v[3*n+2]},omega{w[3*n],w[3*n+1],w[3*n+2]};
    if(!shell_startup_detail::MatchesInitialNode(startup,position,binding.nodes()[n].position,velocity,omega,orientation+4*n))
      return {S::InvalidInput,"Actual common initial state differs from the bound motion declaration"};
    const auto mass=combined?combined->nodes()[n].coefficients.mass:binding.nodes()[n].native.mass;
    if(!shell_startup_detail::AddInitialTranslationKinetic(mass,velocity,measured.translation))
      return {S::NonfiniteResult,"Measured common initial kinetic energy overflows"};
    if(combined) {
      const auto connector_mass=combined->nodes()[n].coefficients.connector_mass;
      // The startup helper requires positive mass; ordinary nodes may carry
      // exactly zero connector mass and contribute a known zero partition.
      if(connector_mass>0&&!shell_startup_detail::AddInitialTranslationKinetic(
          connector_mass,velocity,measured.connector_translation))
        return {S::NonfiniteResult,"Measured connector initial kinetic energy overflows"};
    }
  }
  output=measured; return Ok();
}
void PopulateKineticModel(const ShellBatchBinding& binding,const NodalMassBinding* combined,Model& model) noexcept {
  for(std::size_t node=0;node<binding.node_count();++node) {
    const auto& shell=binding.nodes()[node].native;
    model.mass[node]=shell.mass;
    model.inertia[node]=shell.isotropic_inertia;
    model.physical[node]=shell.physical_inertia;
    model.added[node]=shell.added_inertia;
    if(combined) {
      const auto& coefficients=combined->nodes()[node].coefficients;
      model.mass[node]=coefficients.mass;
      model.inertia[node]=coefficients.isotropic_inertia;
      model.connector_mass[node]=coefficients.connector_mass;
      model.connector_inertia[node]=coefficients.connector_inertia;
    }
  }
}
} // namespace shell_publication_detail
} // namespace tl::fea
