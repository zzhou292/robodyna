// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Source.h"
#include "PhysicalMainSource.h"
#include "../activity_source/Types.h"
#include "../selection/lifecycle/Admission.h"
#include "../assembly/Endpoints.h"
#include "lib_src/elements/ShellBatchFailureBinding.h"
#include "lib_utils/BoundedArena.h"
#include <algorithm>
#include <climits>
#include <new>
#include <stdexcept>
namespace tlfea::contact::radioss_type25::runtime_detail {
namespace {
TransactionReport Fail(TransactionStatus s,const char* m,std::size_t row=SIZE_MAX){return {s,m,row};}
enum class ParentFamily { Qeph,T3,Qbat };
struct Parent {std::uint64_t id=0;ParentFamily family=ParentFamily::Qeph;std::size_t index=0;};
bool Scale(double value,double factor,double& output) {
  const double next=value*factor;
  if(!tl::math::Finite(value)||!tl::math::Finite(next)||(value!=0&&next==0))return false;
  output=next;return true;
}
}
TransactionReport PrepareSourceChecked(const TransactionConfig& config,const ContactSourceInput& source,
    const tl::fea::ShellPhysicalBinding& physical,TransactionLimits limits,bool require_fixed,SourceStaging& out,const startup::Snapshot* mixed) noexcept try {
  const auto& s=source.selection;const auto p=source.primary_main_count;
  const auto* domain=physical.domain();const auto* shells=physical.shells();const auto* ledger=physical.coefficients();
  units_detail::Factors units;
  if(!physical.prepared()||!domain||!shells||!ledger||!source.source_id||!source.topology_generation||!s.generation||
     !units_detail::Make(config.units,units)||source.native_workers!=1||!source.force_packet_size||
     source.force_packet_size>INT_MAX||!p||p>INT_MAX/2||
     (mixed?(s.main_count<p||s.main_count>2*p||source.primary_parent_ids!=nullptr):s.main_count!=2*p)||!s.secondary_count||
     s.node_count!=domain->node_count()||!s.nodes||!s.mains||!s.secondary||
     (!mixed&&!lifecycle::detail::Span(source.primary_parent_ids,p))||!lifecycle::detail::Span(source.primary_curvature,p))
    return Fail(TransactionStatus::InvalidInput,"Incomplete native shell contact source");
  if(s.node_count>UINT32_MAX||s.secondary_count>=INT_MAX||s.node_count>limits.inventory.max_nodes||
     s.secondary_count>limits.inventory.max_secondaries||p>limits.inventory.max_mains||
     limits.optimized_candidates>INT_MAX/5||limits.sliding_entries>=INT_MAX||limits.inventory.max_pairs>=INT_MAX)
    return Fail(TransactionStatus::ResourceLimit,"Native shell source exceeds explicit count limits");
  // Typed span/alignment/overflow admission precedes every borrowed metadata
  // read, including preparation of the physical identity and virgin history.
  if(s.normal_count>4*s.main_count||s.normal_to_main.offset_count!=s.normal_count+1||
     s.normal_to_main.entry_count>4*s.main_count||
     s.removed_main_by_secondary.offset_count!=s.secondary_count+1||
     s.removed_main_by_secondary.entry_count>limits.inventory.max_removals)
    return Fail(TransactionStatus::ResourceLimit,"Native source CSR counts exceed topology limits");
  if(!lifecycle::detail::Span(s.nodes,s.node_count)||!lifecycle::detail::Span(s.mains,s.main_count)||
     !lifecycle::detail::Span(s.secondary,s.secondary_count)||!lifecycle::detail::Span(s.normals,s.normal_count)||
     !lifecycle::detail::Span(s.normal_to_main.offsets,s.normal_to_main.offset_count)||
     !lifecycle::detail::Span(s.normal_to_main.entries,s.normal_to_main.entry_count)||
     !lifecycle::detail::Span(s.removed_main_by_secondary.offsets,s.removed_main_by_secondary.offset_count)||
     !lifecycle::detail::Span(s.removed_main_by_secondary.entries,s.removed_main_by_secondary.entry_count))
    return Fail(TransactionStatus::InvalidInput,"Native source metadata span is invalid");
  if(!normal_detail::Nonnegative(source.margin)||!tl::math::Finite(source.gap_load)||!normal_detail::Nonnegative(source.drad)||
     !friction_detail::Supported(config.friction)||!friction_detail::Valid(config.friction_coefficients)||
     !assembly::detail::Supported(config.assembly)||config.normal.engine.kdtint!=config.assembly.engine.kdtint||
     config.normal.engine.idtmins!=config.assembly.engine.idtmins||config.normal.engine.idtmins_int!=config.assembly.engine.idtmins_int)
    return Fail(TransactionStatus::UnsupportedProfile,"Unsupported native response or search controls");
  // OptimizedCandidate implements the qualified DRAD=0 / DGAPLOAD=0
  // source branch. These controls must not be admitted and silently ignored.
  if(source.drad!=0||source.gap_load!=0)
    return Fail(TransactionStatus::UnsupportedProfile,"Native OPTCD requires zero DRAD and DGAPLOAD");
  if(config.response_mass!=ResponseMassPolicy::StaticPhysicalLedger&&
     config.response_mass!=ResponseMassPolicy::AcceptedOwnerCoefficients)
    return Fail(TransactionStatus::UnsupportedProfile,"Unknown native response mass policy");
  const bool static_mass=config.response_mass==ResponseMassPolicy::StaticPhysicalLedger;
  NativeNormalResult probe;
  if(EvaluateNativeNormal(config.normal,{}, {},&probe)!=NormalStatus::Ok)
    return Fail(TransactionStatus::UnsupportedProfile,"Unsupported native normal response");
  const bool complete=config.physical_source==PhysicalSourceProfile::CompleteBoundLedger;
  if(config.physical_source!=PhysicalSourceProfile::QephT3Only&&!complete)
    return Fail(TransactionStatus::UnsupportedProfile,"Unknown native physical source profile");
  if(complete&&static_mass)
    return Fail(TransactionStatus::UnsupportedProfile,"Complete physical contact requires actual accepted-owner mass");
  if(config.lifecycle.main_coefficient_domain!=(mixed?MainCoefficientDomain::NativeSigned:MainCoefficientDomain::Nonnegative))
    return Fail(TransactionStatus::UnsupportedProfile,"Contact main coefficient domain differs from explicit source profile");
  const auto& coverage=ledger->scope();
  if(coverage.uncovered_nodes||coverage.qeph_parents!=shells->qeph_count()||
     coverage.t3_parents!=shells->t3_count()||coverage.qbat_parents!=shells->qbat_count())
    return Fail(complete?TransactionStatus::SourceMismatch:TransactionStatus::UnsupportedProfile,
      complete?"Contact requires the exact complete physical shell ledger":
      "First native shell profiles require the complete QEPH/T3 mass ledger");
  if(!complete&&(coverage.qbat_parents||coverage.type25_connections||coverage.type13_connections||
     coverage.element_mass_records||coverage.solid18_parents||coverage.solid24_parents||coverage.solid6z_parents||
     coverage.solid18_law44_parents||coverage.solid18_law90_parents||coverage.beam18_parents))
    return Fail(TransactionStatus::UnsupportedProfile,"First native shell profiles require the complete QEPH/T3 mass ledger");
  // Transaction::InitializeSource authenticates this binding and all actual
  // mechanical participants with the common publisher before entering here.
  // Extra contributor families affect their real shared nodal coefficients;
  // this profile creates no substitute mass or constitutive model.
  const bool active_prefix=config.activity==ContactActivityPolicy::AllActivePrefix;
  const bool shell_removal=config.activity==ContactActivityPolicy::ShellRemoval;
  const bool declared_activity=active_prefix||shell_removal;
  if(config.activity!=ContactActivityPolicy::NoDeclaredFailure&&!declared_activity)
    return Fail(TransactionStatus::UnsupportedProfile,"Unknown contact activity policy");
  if(shell_removal?!lifecycle::detail::Span(source.activity_controls,1):
      (source.activity_controls!=nullptr||source.activity_type45!=nullptr))
    return Fail(TransactionStatus::SourceMismatch,"Activity source declaration differs from selected policy");
  if(declared_activity&&(!complete||static_mass||source.contact_thickness_update!=0))
    return Fail(TransactionStatus::UnsupportedProfile,
      "Declared activity requires a complete accepted-owner source with resolved ITHK0");
  const auto* failure=physical.failure();
  if(!failure)return Fail(TransactionStatus::SourceMismatch,"Physical failure declaration is missing");
  for(std::size_t i=0;i<failure->parent_count();++i)
    if(!failure->parent(i)||(!declared_activity&&failure->parent(i)->policy!=tl::fea::ShellFailurePolicy::None))
      return Fail(TransactionStatus::UnsupportedProfile,"Contact activity changes are not admitted",i);
  PhysicalMainValidation main_validation;
  if(mixed) {
    if(require_fixed||!complete||!declared_activity)
      return Fail(TransactionStatus::UnsupportedProfile,"Mixed contact requires a complete declared-activity physical profile");
    main_validation=ValidateMixedPhysicalMains(physical,*mixed,limits.max_host_bytes);
    if(main_validation.report.status!=TransactionStatus::Ok)return main_validation.report;
  }
  // Bound all startup vectors before reading metadata or allocating them. The
  // complete layout accounts simultaneous validation and removal transposition.
  tl::util::BoundedArenaLayout host(limits.max_host_bytes);tl::util::ArenaRegion ignored;
#define CHARGE(type,count) if(!host.Append<type>(count,ignored))return Fail(TransactionStatus::ResourceLimit,"Source startup byte cap exceeded")
  CHARGE(std::uint64_t,s.node_count);CHARGE(int,s.node_count);CHARGE(std::uint32_t,s.secondary_count+4*p);
  CHARGE(candidates::Main,p);CHARGE(NativeGeometryHistory,s.secondary_count);CHARGE(Vector,2*s.node_count);
  CHARGE(double,(static_mass?s.node_count:0)+2*s.secondary_count+3*p);CHARGE(std::uint64_t,2*(p+1));
  CHARGE(std::uint32_t,s.removed_main_by_secondary.entry_count);CHARGE(Parent,mixed?0:shells->qeph_count()+shells->t3_count()+(declared_activity?shells->qbat_count():0));
  CHARGE(std::uint64_t,mixed?0:p);CHARGE(std::uint32_t,s.secondary_count+1);
  CHARGE(std::byte,main_validation.startup_host_bytes);
#undef CHARGE
  SourceStaging next;next.bytes=host.bytes();next.ids.resize(s.node_count);next.codes.resize(s.node_count);
  next.positions.resize(s.node_count);if(static_mass)next.native_mass.resize(s.node_count);
  std::vector<Vector> zero_velocity(s.node_count);
  for(std::size_t node=0;node<s.node_count;++node) {
    if(s.nodes[node].source_id!=domain->nodes()[node].source_id||!SupportedConstraint(s.nodes[node].constraint,s.nodes[node].skew))
      return Fail(TransactionStatus::SourceMismatch,"Contact node identity/constraint differs from physical source",node);
    next.ids[node]=s.nodes[node].source_id;next.codes[node]=s.nodes[node].constraint;next.positions[node]=domain->nodes()[node].position;
    const double mass=ledger->nodes()[node].coefficients.mass;
    if(!(mass>0)||!tl::math::Finite(mass)||
       (static_mass&&(!tl::math::Finite(mass/units.mass)||!(mass/units.mass>0))))
      return Fail(TransactionStatus::SourceMismatch,"Physical raw contact mass is invalid",node);
    if(static_mass)next.native_mass[node]=mass/units.mass;
  }
  next.history.resize(s.secondary_count);next.secondary_nodes.resize(s.secondary_count);
  next.secondary_stiffness.resize(s.secondary_count);next.secondary_gaps.resize(s.secondary_count);
  for(std::size_t row=0;row<s.secondary_count;++row) {
    if(s.secondary[row].node>=s.node_count)return Fail(TransactionStatus::InvalidInput,"Invalid secondary node",row);
    next.history[row]={s.nodes[s.secondary[row].node].source_id,s.generation,{}};
    next.secondary_nodes[row]=s.secondary[row].node;
    if(!Scale(s.secondary[row].coefficient,units.stiffness,next.secondary_stiffness[row])||
       !Scale(s.secondary[row].gap,units.length,next.secondary_gaps[row]))
      return Fail(TransactionStatus::InvalidInput,"Secondary SI conversion is not representable",row);
  }
  std::vector<std::uint32_t> empty_offsets(s.secondary_count+1,0);
  lifecycle::Input input;input.profile=config.lifecycle;input.source=s;
  input.current={VectorView{reinterpret_cast<const double*>(next.positions.data()),std::uint32_t(s.node_count),3,1},
      VectorView{reinterpret_cast<const double*>(zero_velocity.data()),std::uint32_t(s.node_count),3,1},lifecycle::KinematicsUnits::Si,config.units};
  input.accepted_rows=next.history.data();input.accepted_row_count=s.secondary_count;
  input.spatial_by_secondary={empty_offsets.data(),empty_offsets.size(),nullptr,0};
  const auto admitted=lifecycle::detail::Validate(input);
  if(admitted!=selection::Status::Ok)return {TransactionStatus::InvalidInput,"Native lifecycle source admission failed",SIZE_MAX,SIZE_MAX,admitted};
  if(!mixed) {
    std::vector<Parent> parents;
    parents.reserve(shells->qeph_count()+shells->t3_count()+(declared_activity?shells->qbat_count():0));
    for(std::size_t i=0;i<shells->qeph_count();++i)parents.push_back({shells->qeph_source_id(i),ParentFamily::Qeph,i});
    for(std::size_t i=0;i<shells->t3_count();++i)parents.push_back({shells->t3_source_id(i),ParentFamily::T3,i});
    if(declared_activity)for(std::size_t i=0;i<shells->qbat_count();++i)
      parents.push_back({shells->qbat_source_id(i),ParentFamily::Qbat,i});
    std::sort(parents.begin(),parents.end(),[](auto a,auto b){return a.id<b.id;});
    std::vector<std::uint64_t> selected(source.primary_parent_ids,source.primary_parent_ids+p);
    std::sort(selected.begin(),selected.end());
    for(std::size_t i=1;i<p;++i)if(selected[i]==selected[i-1])return Fail(TransactionStatus::SourceMismatch,"Repeated physical primary source",i);
    for(std::size_t i=0;i<p;++i) {
      const auto id=source.primary_parent_ids[i];const auto it=std::lower_bound(parents.begin(),parents.end(),id,[](auto a,auto b){return a.id<b;});
      if(it==parents.end()||it->id!=id)return Fail(TransactionStatus::SourceMismatch,"Primary is not a physical shell",i);
      const auto& main=s.mains[i];const auto& opposite=s.mains[p+i];
      if(main.global_id!=int(i+1)||opposite.global_id!=int(p+i+1)||main.segment_type!=int(p+i+1)||opposite.segment_type!=-int(i+1))
        return Fail(TransactionStatus::SourceMismatch,"Primary/opposite native role map is not the complete ordinary prefix",i);
      const bool triangle=it->family==ParentFamily::T3;
      for(unsigned slot=0;slot<4;++slot) {
        const auto local=triangle?shells->t3_nodes(it->index)[slot<3?slot:2]:
          it->family==ParentFamily::Qbat?shells->qbat_nodes(it->index)[slot]:shells->qeph_nodes(it->index)[slot];
        const auto source_node=shells->active_nodes()[local].source_id;
        if(next.ids[main.nodes[slot]]!=source_node||(require_fixed&&s.nodes[main.nodes[slot]].constraint!=7))
          return Fail(TransactionStatus::SourceMismatch,"Primary ordered connectivity/fixed domain differs from physical shell",i);
        const unsigned reverse=slot==0?1:slot==1?0:triangle?2:slot==2?3:2;
        if(opposite.nodes[slot]!=main.nodes[reverse])return Fail(TransactionStatus::SourceMismatch,"Opposite connectivity is not native SH2SURF order",i);
      }
    }
  }
  next.primary.resize(p);next.main_nodes.resize(4*p);next.main_stiffness.resize(p);next.main_gaps.resize(p);next.main_curvature.resize(p);
  for(std::size_t i=0;i<p;++i) {
    const auto& main=s.mains[i];
    for(unsigned slot=0;slot<4;++slot) {
      next.primary[i].nodes[slot]=main.nodes[slot];next.main_nodes[4*i+slot]=main.nodes[slot];
    }
    if(!normal_detail::Nonnegative(source.primary_curvature[i]))return Fail(TransactionStatus::InvalidInput,"Invalid main curvature",i);
    next.primary[i].source_id=std::uint64_t(main.global_id);next.primary[i].segment_type=main.segment_type;
    if(!Scale(main.coefficient,units.stiffness,next.main_stiffness[i])||
       !Scale(main.maximum_gap,units.length,next.main_gaps[i])||
       !Scale(source.primary_curvature[i],units.length,next.main_curvature[i]))
      return Fail(TransactionStatus::InvalidInput,"Main SI conversion is not representable",i);
  }
  // Transpose genuine per-secondary main-removal lists into the candidate
  // owner's main-to-physical-node representation. Duplicate source occurrences
  // are preserved; no absent list is synthesized from NOINT or shell thickness.
  next.removal_offsets.assign(p+1,0);
  const auto csr=s.removed_main_by_secondary;
  for(std::size_t row=0;row<s.secondary_count;++row)for(auto j=csr.offsets[row];j<csr.offsets[row+1];++j)
    if(csr.entries[j]<=p)++next.removal_offsets[csr.entries[j]];
  for(std::size_t i=0;i<p;++i)next.removal_offsets[i+1]+=next.removal_offsets[i];
  next.removal_nodes.resize(next.removal_offsets[p]);auto cursor=next.removal_offsets;
  for(std::size_t row=0;row<s.secondary_count;++row)for(auto j=csr.offsets[row];j<csr.offsets[row+1];++j)
    if(csr.entries[j]<=p)next.removal_nodes[cursor[csr.entries[j]-1]++]=s.secondary[row].node;
  next.inventory.stamp={source.source_id,source.topology_generation};next.inventory.units=config.units;
  next.inventory.input_units=candidates::InputUnits::Si;next.inventory.physical_nodes=s.node_count;
  next.inventory.secondaries=s.secondary_count;next.inventory.mains=p;next.inventory.removals=next.removal_nodes.size();
  next.inventory.node_ids=next.ids.data();next.inventory.constraint_codes=next.codes.data();
  next.inventory.secondary_nodes=next.secondary_nodes.data();next.inventory.main=next.primary.data();
  next.inventory.removal_offsets=next.removal_offsets.data();next.inventory.removal_nodes=next.removal_nodes.data();
  next.inventory.primary_main_count=int(p);
  next.inventory.main_coefficient_domain=config.lifecycle.main_coefficient_domain;
  next.maintenance.stamp={source.source_id,source.topology_generation,s.generation};next.maintenance.units=config.units;
  next.maintenance.input_units=search::InputUnits::Si;next.maintenance.physical_nodes=s.node_count;
  next.maintenance.secondary_nodes=next.secondary_nodes.data();next.maintenance.secondaries=s.secondary_count;
  next.maintenance.main_nodes=next.main_nodes.data();next.maintenance.mains=4*p;
  next.maintenance.main_segments=p;next.maintenance.margin=source.margin;
  next.maintenance.activity_policy=config.activity==ContactActivityPolicy::ShellRemoval?
      search::ActivityPolicy::MonotoneRetirement:search::ActivityPolicy::Immutable;
  out=std::move(next);return {TransactionStatus::Ok,"OK"};
} catch(const std::bad_alloc&){return Fail(TransactionStatus::ResourceLimit,"Source startup allocation failed");}
  catch(const std::length_error&){return Fail(TransactionStatus::ResourceLimit,"Source startup length overflow");}
TransactionReport PrepareSource(const TransactionConfig& config,const FixedMainSource& source,
    const tl::fea::ShellPhysicalBinding& physical,TransactionLimits limits,SourceStaging& out) noexcept {
  return PrepareSourceChecked(config,source,physical,limits,true,out);
}
} // namespace tlfea::contact::radioss_type25::runtime_detail
