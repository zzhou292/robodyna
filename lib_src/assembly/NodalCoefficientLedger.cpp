// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NodalCoefficientInternal.h"
#include <new>
#include <stdexcept>

namespace tl::fea {
struct NodalCoefficientLedger::Impl {
  Impl(NodalCoefficientSources input,const ElementMassContributions* mass,
      const SolidNodeContributions* solid):shells(*input.shells),
      springs(input.type25?*input.type25:tl::fea::type25::Model{}),
      beams(input.type13?*input.type13:Type13NodeContributions{}),
      masses(mass?*mass:ElementMassContributions{}),
      solids(solid?*solid:SolidNodeContributions{}) {}
  ShellNodeMap shells;
  tl::fea::type25::Model springs;
  Type13NodeContributions beams;
  ElementMassContributions masses;
  SolidNodeContributions solids;
  util::HostArena arena;
  NodalCoefficientNode* nodes=nullptr;
  NodalCoefficientTotals totals{};
  NodalCoefficientScope scope{};
  CoefficientOrder order=CoefficientOrder::PreparedSI_Q_T_B_Type25_Type13_V1;
  std::size_t retained=0,startup=0;
};
CoefficientReport NodalCoefficientLedger::Initialize(NodalCoefficientSources input,
    CoefficientLimits limits) noexcept {
  return InitializeImpl({input,nullptr},CoefficientOrder::PreparedSI_Q_T_B_Type25_Type13_V1,limits);
}
CoefficientReport NodalCoefficientLedger::InitializeWithElementMass(
    NodalCoefficientSourcesWithElementMass input,CoefficientLimits limits) noexcept {
  return InitializeImpl({input.structural,input.element_mass,nullptr},
      CoefficientOrder::PreparedSI_Q_T_B_Type25_Type13_ElementMass_V2,limits);
}
CoefficientReport NodalCoefficientLedger::InitializeWithSolids(
    NodalCoefficientSourcesWithSolids input,CoefficientLimits limits) noexcept {
  return InitializeImpl(input,CoefficientOrder::PreparedSI_Q_T_B_Type25_Type13_ElementMass_Solid18_24_6z_V3,limits);
}
CoefficientReport NodalCoefficientLedger::InitializeImpl(NodalCoefficientSourcesWithSolids sources,
    CoefficientOrder order,CoefficientLimits limits) noexcept try {
  using namespace coefficient_detail;
  if(impl_) return {S::AlreadyInitialized,"Coefficient ledger is immutable"};
  const auto input=sources.structural;
  const auto* masses=sources.element_mass;
  const auto* solids=sources.solids;
  Budget budget(limits.max_host_bytes);
  auto r=Preflight(input,masses,solids,limits,sizeof(Impl),budget);
  if(!r) return r;
  r=Identities(input,masses,solids);
  if(!r) return r;
  auto next=std::make_shared<Impl>(input,masses,solids);
  if(!next->arena.Initialize(budget.arena.bytes())||
      !(next->nodes=next->arena.Construct<NodalCoefficientNode>(budget.nodes)))
    return {S::ResourceLimit,"Coefficient node arena allocation failed"};
  const auto& binding=*input.shells->shells();
  auto& scope=next->scope;
  scope.qeph_parents=binding.qeph_count();
  scope.t3_parents=binding.t3_count();
  scope.qbat_parents=binding.qbat_count();
  scope.type25_connections=input.type25?input.type25->connection_count():0;
  scope.type13_connections=input.type13?input.type13->model()->connection_count():0;
  scope.element_mass_records=masses?masses->records().size():0;
  scope.solid18_parents=solids?solids->parent_count(SolidCoefficientFamily::Solid18):0;
  scope.solid24_parents=solids?solids->parent_count(SolidCoefficientFamily::Solid24):0;
  scope.solid6z_parents=solids?solids->parent_count(SolidCoefficientFamily::Solid6z):0;
  r=Shells(next->shells,next->nodes);
  if(!r) return r;
  r=Springs(input,next->nodes);
  if(!r) return r;
  r=ElementMasses(masses,next->nodes);
  if(!r) return r;
  r=Solids(solids,next->nodes);
  if(!r) return r;
  r=Totals(next->nodes,next->shells.owner_node_count(),next->totals,scope);
  if(!r) return r;
  next->retained=budget.retained;
  next->startup=budget.startup;
  next->order=order;
  impl_=std::move(next);
  return {};
} catch(const std::bad_alloc&) {
  return {CoefficientStatus::ResourceLimit,"Coefficient allocation failed"};
} catch(const std::length_error&) {
  return {CoefficientStatus::ResourceLimit,"Coefficient allocation size overflow"};
}
const ShellNodeMap* NodalCoefficientLedger::shells() const noexcept {return impl_?&impl_->shells:nullptr;}
const type25::Model* NodalCoefficientLedger::type25() const noexcept {
  return impl_&&impl_->springs.prepared()?&impl_->springs:nullptr;
}
const Type13NodeContributions* NodalCoefficientLedger::type13() const noexcept {
  return impl_&&impl_->beams.prepared()?&impl_->beams:nullptr;
}
const ElementMassContributions* NodalCoefficientLedger::element_mass() const noexcept {
  return impl_&&impl_->masses.prepared()?&impl_->masses:nullptr;
}
const SolidNodeContributions* NodalCoefficientLedger::solids() const noexcept {
  return impl_&&impl_->solids.prepared()?&impl_->solids:nullptr;
}
CoefficientOrder NodalCoefficientLedger::order() const noexcept {
  return impl_?impl_->order:CoefficientOrder::PreparedSI_Q_T_B_Type25_Type13_V1;
}
const NodalNodeDomain* NodalCoefficientLedger::domain() const noexcept {return impl_?impl_->shells.domain():nullptr;}
tl::util::ConstView<NodalCoefficientNode> NodalCoefficientLedger::nodes() const noexcept {
  static const NodalCoefficientNode empty;
  return {impl_?impl_->nodes:&empty,impl_?impl_->shells.owner_node_count():0};
}
const NodalCoefficientTotals& NodalCoefficientLedger::totals() const noexcept {
  static const NodalCoefficientTotals empty;
  return impl_?impl_->totals:empty;
}
const NodalCoefficientScope& NodalCoefficientLedger::scope() const noexcept {
  static const NodalCoefficientScope empty;
  return impl_?impl_->scope:empty;
}
bool NodalCoefficientLedger::Matches(NodalCoefficientSources input) const noexcept {
  return impl_&&order()==CoefficientOrder::PreparedSI_Q_T_B_Type25_Type13_V1&&
      input.shells&&impl_->shells.Matches(*input.shells)&&
      bool(input.type25)==bool(type25())&&(!input.type25||impl_->springs.Matches(*input.type25))&&
      bool(input.type13)==bool(type13())&&(!input.type13||impl_->beams.Matches(*input.type13));
}
bool NodalCoefficientLedger::MatchesWithElementMass(NodalCoefficientSourcesWithElementMass sources) const noexcept {
  const auto input=sources.structural;
  return impl_&&order()==CoefficientOrder::PreparedSI_Q_T_B_Type25_Type13_ElementMass_V2&&
      input.shells&&impl_->shells.Matches(*input.shells)&&
      bool(input.type25)==bool(type25())&&(!input.type25||impl_->springs.Matches(*input.type25))&&
      bool(input.type13)==bool(type13())&&(!input.type13||impl_->beams.Matches(*input.type13))&&
      bool(sources.element_mass)==bool(element_mass())&&
      (!sources.element_mass||impl_->masses.Matches(*sources.element_mass));
}
bool NodalCoefficientLedger::Matches(const NodalCoefficientLedger& other) const noexcept {
  if(!impl_||!other.impl_) return false;
  if(impl_==other.impl_) return true;
  const NodalCoefficientSources sources{other.shells(),other.type25(),other.type13()};
  switch(other.order()) {
    case CoefficientOrder::PreparedSI_Q_T_B_Type25_Type13_V1: return Matches(sources);
    case CoefficientOrder::PreparedSI_Q_T_B_Type25_Type13_ElementMass_V2:
      return MatchesWithElementMass({sources,other.element_mass()});
    case CoefficientOrder::PreparedSI_Q_T_B_Type25_Type13_ElementMass_Solid18_24_6z_V3:
      return MatchesWithSolids({sources,other.element_mass(),other.solids()});
  }
  return false;
}
bool NodalCoefficientLedger::MatchesWithSolids(NodalCoefficientSourcesWithSolids sources) const noexcept {
  const auto input=sources.structural;
  return impl_&&order()==CoefficientOrder::PreparedSI_Q_T_B_Type25_Type13_ElementMass_Solid18_24_6z_V3&&
      input.shells&&impl_->shells.Matches(*input.shells)&&
      bool(input.type25)==bool(type25())&&(!input.type25||impl_->springs.Matches(*input.type25))&&
      bool(input.type13)==bool(type13())&&(!input.type13||impl_->beams.Matches(*input.type13))&&
      bool(sources.element_mass)==bool(element_mass())&&
      (!sources.element_mass||impl_->masses.Matches(*sources.element_mass))&&
      bool(sources.solids)==bool(solids())&&(!sources.solids||impl_->solids.Matches(*sources.solids));
}
std::size_t NodalCoefficientLedger::owned_payload_bytes() const noexcept {return impl_?impl_->retained:sizeof(*this);}
std::size_t NodalCoefficientLedger::startup_payload_bytes() const noexcept {return impl_?impl_->startup:sizeof(*this);}
} // namespace tl::fea
