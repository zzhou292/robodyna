#include "QephBatchStorage.h"
#include "QephStartup.h"
#include "QephHistory.h"
#include "QephBatchStartup.h"
#include "../ShellBatchJoinedModel.h"
#include "../ShellResidentStartupIndex.h"
#include "lib_src/solvers/NodalNativePhysicalCoefficients.h"
#include <cmath>
#include <new>

namespace tl::fea::qeph::batch_detail {
namespace {
bool SameVector(Vec3 a,Vec3 b) {
  return detail::SameHistoryBits(a.x,b.x)&&detail::SameHistoryBits(a.y,b.y)&&detail::SameHistoryBits(a.z,b.z);
}
bool SameReference(const ReferenceData& a,const ReferenceData& b) {
  if(!a.prepared||!b.prepared||a.area!=b.area) return false;
  for(unsigned i=0;i<9;++i) if(a.frame.v[i]!=b.frame.v[i]) return false;
  for(unsigned i=0;i<4;++i)
    if(a.derivative_x[i]!=b.derivative_x[i]||a.derivative_y[i]!=b.derivative_y[i]||
       !SameVector(a.local_position[i],b.local_position[i])||a.nodal_mass[i]!=b.nodal_mass[i]||
       a.physical_inertia[i]!=b.physical_inertia[i]||a.added_inertia[i]!=b.added_inertia[i]||
       a.isotropic_inertia[i]!=b.isotropic_inertia[i]) return false;
  return true;
}
}

BatchReport BuildModel(const QephBatchConfig& c,const QephBatchElement* input,Model& model,Slab& initial,const ShellBatchBinding* joined) {
  const auto& o=c.owner;
  if(!native_physical_coefficients::ValidScope(o.rigid_groups,o.node_count)||
     (!native_physical_coefficients::Empty(o.rigid_groups)&&(!joined||c.usage!=BatchUsage::CoupledForces)))
    return {BatchStatus::InvalidInput,"Rigid groups require complete scope and joined coupled shell publication"};
  if((!input&&!joined)||!o.owner_id||!o.has_rotations||o.epoch||o.time!=0||o.velocity_time!=0||
     o.reactions_valid||!std::isfinite(o.fixed_dt)||o.fixed_dt<=0||
     o.temporal_scheme!=NodalTemporalScheme::StaggeredHalfKickStart||
     o.velocity_phase!=NodalVelocityPhase::Collocated||!c.configuration_id||!c.qualification_id||
     (c.usage!=BatchUsage::PrescribedFields&&c.usage!=BatchUsage::CoupledForces))
    return {BatchStatus::InvalidInput,"Batch requires explicit usage and an epoch-zero staggered rotational owner"};
  if(!ValidStartup(c,joined!=nullptr))
    return {BatchStatus::InvalidInput,"Invalid or unsupported QEPH initial motion declaration"};
  Layout checked_layout;
  if(!ValidShellResidentLimits(c.storage_limits,c.element_count,o.node_count,c.max_device_bytes)||
     !checked_layout.Initialize(c.element_count,o.node_count,c.max_device_bytes))
    return {BatchStatus::ResourceLimit,"QEPH element/node/allocation capacity exceeded"};
  if(joined&&(!joined->prepared()||!joined->t3_count()||c.element_count!=joined->qeph_count()||
              o.node_count!=joined->node_count()))
    return {BatchStatus::InvalidInput,"Joined QEPH scope requires its exact count from the complete mixed collection"};
  model.config=c;
  std::unique_ptr<bool[]> seen(new(std::nothrow) bool[o.node_count]{});
  std::unique_ptr<std::uint32_t[]> ids(new(std::nothrow) std::uint32_t[o.node_count]{});
  if(!seen||!ids) return {BatchStatus::ResourceLimit,"QEPH active identity staging allocation failed"};
  const bool indexed=VehicleShellResidentLimits(c.storage_limits);
  shell_batch_detail::ResidentNodeIdentityIndex identity;
  if(indexed)identity.Prepare(4*c.element_count,[&](std::size_t i) { return joined?joined->qeph_reference(i/4).input.node_ids[i%4]:input[i/4].reference.input.node_ids[i%4]; });
  for(unsigned e=0;e<c.element_count;++e) {
    QephBatchElement selected;
    if(joined) {
      selected.reference=joined->qeph_reference(e);
      for(unsigned local=0;local<4;++local) selected.nodes[local]=joined->qeph_nodes(e)[local];
    }
    const auto& element=joined?selected:input[e]; ReferenceData checked;
    const auto status=InitializeReference(element.reference.input,checked);
    if(status!=Status::kSuccess||!SameReference(element.reference,checked))
      return {BatchStatus::ElementFailure,"Reference differs from its startup producer",e,UINT32_MAX,
              status==Status::kSuccess?Status::kInvalidReference:status};
    model.element[e]=element;
    for(unsigned local=0;local<4;++local) {
      const auto n=element.nodes[local];
      if(n>=o.node_count) return {BatchStatus::InvalidInput,"QEPH connectivity outside owner",e};
      for(unsigned other=0;other<local;++other)
        if(element.nodes[other]==n) return {BatchStatus::InvalidInput,"Repeated QEPH node",e};
      const auto id=element.reference.input.node_ids[local]; const auto x=element.reference.input.position[local];
      if(seen[n]&&(ids[n]!=id||!SameVector(model.initial_position[n],x)))
        return {BatchStatus::InvalidInput,"Shared reference node identity/position mismatch",e,static_cast<std::uint32_t>(n)};
      if(indexed) {
        if(!seen[n]&&identity.First(id)!=4*e+local)
          return {BatchStatus::InvalidInput,"Source node identity maps to multiple owner nodes",e};
      } else {
      for(unsigned other=0;other<o.node_count;++other)
        if(other!=n&&seen[other]&&ids[other]==id)
          return {BatchStatus::InvalidInput,"Source node identity maps to multiple owner nodes",e};
      }
      seen[n]=true; ids[n]=id; model.initial_position[n]=x;
      model.mass[n]+=checked.nodal_mass[local]; model.inertia[n]+=checked.isotropic_inertia[local];
      model.physical[n]+=checked.physical_inertia[local]; model.added[n]+=checked.added_inertia[local];
    }
    const auto history=InitializeHistory(element.reference,{0,0},initial.element[e].proposed_history);
    if(history!=Status::kSuccess) return {BatchStatus::ElementFailure,"Cannot initialize QEPH history",e,UINT32_MAX,history};
  }
  if(joined) shell_batch_detail::ApplyJoinedMass(*joined,model);
  for(unsigned n=0;n<o.node_count;++n)
    if((!joined&&!seen[n])||!detail::Positive(model.mass[n])||!detail::Positive(model.inertia[n])||
       !detail::Positive(model.physical[n])||!detail::Positive(model.added[n]))
      return {BatchStatus::InvalidMass,"Uncovered node or invalid assembled native mass/inertia",UINT32_MAX,n};
  if(c.startup.kind==BatchStartupKind::ReferenceUniformTranslation) {
    double kinetic=0;
    for(unsigned n=0;n<o.node_count;++n)
      if(!AddInitialTranslationKinetic(model.mass[n],c.startup.uniform_velocity,kinetic))
        return {BatchStatus::NonfiniteResult,"Declared initial translation kinetic energy overflows",UINT32_MAX,n};
  }
  return {BatchStatus::Success,"OK"};
}
BatchDiagnostics InitialDiagnostics(const QephBatchConfig& c,bool joined) {
  BatchDiagnostics d; d.owner_id=c.owner.owner_id; d.configuration_id=c.configuration_id;
  d.qualification_id=c.qualification_id; d.phase=BatchPhase::Accepted; d.usage=c.usage; d.valid=true;
  d.kinetic_available=!joined; return d;
}
} // namespace tl::fea::qeph::batch_detail
