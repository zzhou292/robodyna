#include "T3BatchStorage.h"
#include "T3Startup.h"
#include "T3History.h"
#include "../ShellBatchJoinedModel.h"
#include <cmath>

namespace tl::fea::t3::batch_detail {
namespace {
bool SameVector(Vec3 a,Vec3 b) {
  return detail::SameHistoryBits(a.x,b.x)&&detail::SameHistoryBits(a.y,b.y)&&detail::SameHistoryBits(a.z,b.z);
}
bool SameReference(const ReferenceData& a,const ReferenceData& b) {
  if(!a.prepared||!b.prepared) return false;
  for(unsigned i=0;i<9;++i) if(!detail::SameHistoryBits(a.frame.v[i],b.frame.v[i])) return false;
  for(unsigned i=0;i<3;++i) {
    if(!SameVector(a.local_position[i],b.local_position[i])) return false;
    const double x[]{a.angle_cosine[i],a.angle_weight[i],a.nodal_mass[i],a.physical_inertia[i],
      a.added_inertia[i],a.isotropic_inertia[i],a.startup_derivative[i]};
    const double y[]{b.angle_cosine[i],b.angle_weight[i],b.nodal_mass[i],b.physical_inertia[i],
      b.added_inertia[i],b.isotropic_inertia[i],b.startup_derivative[i]};
    for(unsigned c=0;c<7;++c) if(!detail::SameHistoryBits(x[c],y[c])) return false;
  }
  const double x[]{a.area,a.element_mass,a.element_isotropic_inertia,a.element_physical_inertia,
    a.element_added_inertia,a.characteristic_length};
  const double y[]{b.area,b.element_mass,b.element_isotropic_inertia,b.element_physical_inertia,
    b.element_added_inertia,b.characteristic_length};
  for(unsigned i=0;i<6;++i) if(!detail::SameHistoryBits(x[i],y[i])) return false;
  return true;
}
}

BatchReport BuildModel(const T3BatchConfig& c,const T3BatchElement* input,Model& output,Slab& startup,const ShellBatchBinding* joined) {
  const auto& o=c.owner;
  if(!input||!o.owner_id||!o.has_rotations||o.epoch||o.time!=0||o.velocity_time!=0||
     o.reactions_valid||!std::isfinite(o.fixed_dt)||o.fixed_dt<=0||
     o.temporal_scheme!=NodalTemporalScheme::StaggeredHalfKickStart||
     o.velocity_phase!=NodalVelocityPhase::Collocated||!c.configuration_id||!c.qualification_id||
     (c.usage!=BatchUsage::PrescribedFields&&c.usage!=BatchUsage::CoupledForces))
    return {BatchStatus::InvalidInput,"Batch requires explicit usage and an epoch-zero staggered rotational owner"};
  if(!c.element_count||c.element_count>MaxBatchElements||!o.node_count||o.node_count>MaxBatchNodes||
     !c.max_device_bytes||c.max_device_bytes>MaxBatchDeviceBytes||sizeof(Storage)>c.max_device_bytes)
    return {BatchStatus::ResourceLimit,"T3 element/node/allocation capacity exceeded"};
  if(joined&&(!joined->prepared()||c.element_count!=1||o.node_count!=joined->node_count()))
    return {BatchStatus::InvalidInput,"Joined scope requires one element from the complete union"};
  Model model{}; Slab initial{}; model.config=c;
  bool seen[MaxBatchNodes]{}; std::uint64_t ids[MaxBatchNodes]{};
  for(unsigned e=0;e<c.element_count;++e) {
    const auto& element=input[e]; ReferenceData checked;
    const auto status=InitializeReference(element.reference.input,checked);
    if(status!=Status::kSuccess||!SameReference(element.reference,checked))
      return {BatchStatus::ElementFailure,"Reference differs from its startup producer",e,UINT32_MAX,
              status==Status::kSuccess?Status::kInvalidReference:status};
    for(unsigned prior=0;prior<e;++prior) {
      bool same=true;
      for(unsigned i=0;i<3;++i) {
        bool found=false;
        for(unsigned j=0;j<3;++j) found|=element.nodes[i]==model.element[prior].nodes[j];
        same&=found;
      }
      if(same) return {BatchStatus::InvalidInput,"Duplicate T3 physical connectivity",e};
    }
    model.element[e]=element;
    for(unsigned local=0;local<3;++local) {
      const auto n=element.nodes[local];
      if(n>=o.node_count) return {BatchStatus::InvalidInput,"T3 connectivity outside owner",e};
      for(unsigned other=0;other<local;++other)
        if(element.nodes[other]==n) return {BatchStatus::InvalidInput,"Repeated T3 node",e};
      const auto id=element.reference.input.node_ids[local]; const auto x=element.reference.input.position[local];
      if(seen[n]&&(ids[n]!=id||!SameVector(model.initial_position[n],x)))
        return {BatchStatus::InvalidInput,"Shared reference node identity/position mismatch",e,static_cast<std::uint32_t>(n)};
      for(unsigned other=0;other<o.node_count;++other)
        if(other!=n&&seen[other]&&ids[other]==id)
          return {BatchStatus::InvalidInput,"Source node identity maps to multiple owner nodes",e};
      seen[n]=true; ids[n]=id; model.initial_position[n]=x;
      model.mass[n]+=checked.nodal_mass[local]; model.inertia[n]+=checked.isotropic_inertia[local];
      model.physical[n]+=checked.physical_inertia[local]; model.added[n]+=checked.added_inertia[local];
    }
    const auto history=InitializeHistory(element.reference,{0,0},initial.element[e].proposed_history);
    if(history!=Status::kSuccess) return {BatchStatus::ElementFailure,"Cannot initialize T3 history",e,UINT32_MAX,history};
  }
  if(joined) shell_batch_detail::ApplyJoinedMass(*joined,model);
  for(unsigned n=0;n<o.node_count;++n)
    if((!joined&&!seen[n])||!detail::Positive(model.mass[n])||!detail::Positive(model.inertia[n])||
       !detail::Positive(model.physical[n])||!detail::Positive(model.added[n]))
      return {BatchStatus::InvalidMass,"Uncovered node or invalid assembled native mass/inertia",UINT32_MAX,n};
  output=model; startup=initial;
  return {BatchStatus::Success,"OK"};
}
BatchDiagnostics InitialDiagnostics(const T3BatchConfig& c,bool joined) {
  BatchDiagnostics d; d.owner_id=c.owner.owner_id; d.configuration_id=c.configuration_id;
  d.qualification_id=c.qualification_id; d.phase=BatchPhase::Accepted; d.usage=c.usage; d.valid=true;
  d.kinetic_available=!joined; return d;
}
} // namespace tl::fea::t3::batch_detail
