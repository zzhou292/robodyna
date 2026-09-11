// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NodalCoefficientInternal.h"
#include <new>
#include <stdexcept>

namespace tl::fea {
struct NodalCoefficientLedger::Impl {
  explicit Impl(NodalCoefficientSources input):shells(*input.shells),
      springs(input.type25?*input.type25:tl::fea::type25::Model{}),
      beams(input.type13?*input.type13:Type13NodeContributions{}) {}
  ShellNodeMap shells;
  tl::fea::type25::Model springs;
  Type13NodeContributions beams;
  util::HostArena arena;
  NodalCoefficientNode* nodes=nullptr;
  NodalCoefficientTotals totals{};
  NodalCoefficientScope scope{};
  std::size_t retained=0,startup=0;
};
CoefficientReport NodalCoefficientLedger::Initialize(NodalCoefficientSources input,
    CoefficientLimits limits) noexcept try {
  using namespace coefficient_detail;
  if(impl_) return {S::AlreadyInitialized,"Coefficient ledger is immutable"};
  Budget budget(limits.max_host_bytes);
  auto r=Preflight(input,limits,sizeof(Impl),budget);
  if(!r) return r;
  r=Identities(input);
  if(!r) return r;
  auto next=std::make_shared<Impl>(input);
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
  r=Shells(next->shells,next->nodes);
  if(!r) return r;
  r=Springs(input,next->nodes);
  if(!r) return r;
  r=Totals(next->nodes,next->shells.owner_node_count(),next->totals,scope);
  if(!r) return r;
  next->retained=budget.retained;
  next->startup=budget.startup;
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
  return impl_&&input.shells&&impl_->shells.Matches(*input.shells)&&
      bool(input.type25)==bool(type25())&&(!input.type25||impl_->springs.Matches(*input.type25))&&
      bool(input.type13)==bool(type13())&&(!input.type13||impl_->beams.Matches(*input.type13));
}
bool NodalCoefficientLedger::Matches(const NodalCoefficientLedger& other) const noexcept {
  return impl_&&other.impl_&&(impl_==other.impl_||Matches({other.shells(),other.type25(),other.type13()}));
}
std::size_t NodalCoefficientLedger::owned_payload_bytes() const noexcept {return impl_?impl_->retained:sizeof(*this);}
std::size_t NodalCoefficientLedger::startup_payload_bytes() const noexcept {return impl_?impl_->startup:sizeof(*this);}
} // namespace tl::fea
