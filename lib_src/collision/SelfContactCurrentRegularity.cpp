// SPDX-License-Identifier: AGPL-3.0-or-later
#include "self_contact_current_regularity/Storage.h"

#include "lib_src/solvers/NodalTrialIdentity.h"

#include <limits>
#include <new>
#include <utility>

namespace tlfea::contact {
namespace {

using S = SelfContactCurrentRegularityStatus;

SelfContactCurrentRegularityReport Fail(
    S status, const char* message, std::size_t parents = 0,
    std::size_t facets = 0, std::size_t parent = SIZE_MAX) noexcept {
  return {status, parent, SIZE_MAX, parents, facets, message};
}

bool ValidRange(const void* pointer, std::size_t count,
                std::size_t width) noexcept {
  if (!count) return pointer == nullptr;
  if (!pointer || count > SIZE_MAX/width) return false;
  const auto address = reinterpret_cast<std::uintptr_t>(pointer);
  return count*width <= UINTPTR_MAX-address;
}

bool ViewRange(VectorView view, std::size_t expected_nodes,
               std::size_t* bytes) noexcept {
  if (!bytes || !view.valid() ||
      view.node_count != expected_nodes)
    return false;
  const std::uint64_t last =
      (std::uint64_t{view.node_count}-1)*view.node_stride +
      2*view.component_stride;
  if (last >= SIZE_MAX/sizeof(double)) return false;
  *bytes = (static_cast<std::size_t>(last)+1)*sizeof(double);
  return true;
}

template <class ImplType>
bool StorageDisjoint(const SelfContactCurrentRegularity* owner,
                     const ImplType& impl, const void* pointer,
                     std::size_t bytes) noexcept {
  using tl::fea::trial_identity::Disjoint;
  return Disjoint(pointer, bytes, owner, sizeof(*owner)) &&
      Disjoint(pointer, bytes, &impl, sizeof(impl)) &&
      Disjoint(pointer, bytes, impl.arena.data(), impl.arena.bytes()) &&
      impl.binding.OutputDisjoint(pointer, bytes);
}

bool Same(VectorView a, VectorView b) noexcept {
  return a.data == b.data && a.node_count == b.node_count &&
      a.node_stride == b.node_stride &&
      a.component_stride == b.component_stride;
}

bool Same(SelfContactActivityView a,
          SelfContactActivityView b) noexcept {
  return a.base == b.base && a.current == b.current &&
      a.parent_count == b.parent_count;
}

}  // namespace

SelfContactCurrentRegularity::SelfContactCurrentRegularity() noexcept =
    default;
SelfContactCurrentRegularity::~SelfContactCurrentRegularity() = default;
SelfContactCurrentRegularity::SelfContactCurrentRegularity(
    SelfContactCurrentRegularity&&) noexcept = default;
SelfContactCurrentRegularity& SelfContactCurrentRegularity::operator=(
    SelfContactCurrentRegularity&&) noexcept = default;

SelfContactCurrentRegularityPreflight
SelfContactCurrentRegularity::Preflight(
    const SelfContactActiveUseBinding& binding,
    SelfContactCurrentRegularityLimits limits) noexcept {
  current_regularity::Layout layout;
  auto report = current_regularity::MakeLayout(
      binding, limits, sizeof(Impl), layout);
  if (report.status == S::Ok)
    report = current_regularity::ValidateSource(
        binding, layout.forecast);
  return {report, layout.forecast};
}

SelfContactCurrentRegularityReport
SelfContactCurrentRegularity::Initialize(
    const SelfContactActiveUseBinding& binding,
    SelfContactCurrentRegularityLimits limits) noexcept try {
  if (impl_)
    return Fail(S::AlreadyInitialized,
        "Current-regularity binding is immutable",
        impl_->forecast.parents, impl_->forecast.facets);
  if (!binding.OutputDisjoint(this, sizeof(*this)))
    return Fail(S::InvalidInput,
        "Destination aliases active-use/facet/S0 source or source is absent");
  current_regularity::Layout layout;
  auto report = current_regularity::MakeLayout(
      binding, limits, sizeof(Impl), layout);
  if (report.status != S::Ok) return report;
  report = current_regularity::ValidateSource(binding, layout.forecast);
  if (report.status != S::Ok) return report;

  auto next = std::make_unique<Impl>(binding);
  if (!next->arena.Initialize(layout.forecast.arena_bytes))
    return Fail(S::ResourceLimit,
        "Current result/facet staging allocation failed",
        layout.forecast.parents, layout.forecast.facets);
  auto& storage = next->storage;
  storage.first_results =
      next->arena.Construct<SelfContactCurrentParentResult>(
          layout.first_results);
  storage.second_results =
      next->arena.Construct<SelfContactCurrentParentResult>(
          layout.second_results);
  storage.facet_staging =
      next->arena.Construct<CurrentFixedTriangle>(
          layout.facet_staging);
  if (!storage.first_results || !storage.second_results ||
      !storage.facet_staging)
    return Fail(S::ResourceLimit,
        "Typed current result/facet staging construction failed",
        layout.forecast.parents, layout.forecast.facets);
  next->publication = storage.first_results;
  next->staging = storage.second_results;
  next->forecast = layout.forecast;
  impl_ = std::move(next);
  return {S::Ok, SIZE_MAX, SIZE_MAX,
          impl_->forecast.parents, impl_->forecast.facets, "OK"};
} catch (const std::bad_alloc&) {
  return Fail(S::ResourceLimit,
      "Current-regularity startup allocation failed");
}

SelfContactCurrentRegularityReport
SelfContactCurrentRegularity::Certify(
    VectorView positions, SelfContactActivityView activity,
    SelfContactCurrentRegularityReceipt* receipt) noexcept {
  if (!impl_)
    return Fail(S::NotInitialized,
        "Current-regularity binding is not initialized");
  const auto parents = impl_->forecast.parents;
  const auto facets = impl_->forecast.facets;
  std::size_t position_bytes = 0;
  if (!receipt ||
      !ViewRange(positions, impl_->binding.forecast().node_roles,
                 &position_bytes) ||
      !ValidRange(activity.base, activity.parent_count,
                  sizeof(std::uint8_t)) ||
      !ValidRange(activity.current, activity.parent_count,
                  sizeof(std::uint8_t)) ||
      activity.parent_count != parents)
    return Fail(S::InvalidInput,
        "Current positions/activity/receipt shape is invalid",
        parents, facets);
  const std::size_t activity_bytes =
      activity.parent_count*sizeof(std::uint8_t);
  using tl::fea::trial_identity::Disjoint;
  const bool activity_same = activity.base == activity.current;
  if (!StorageDisjoint(this, *impl_, receipt, sizeof(*receipt)) ||
      !StorageDisjoint(this, *impl_, positions.data, position_bytes) ||
      !StorageDisjoint(this, *impl_,
          activity.base, activity_bytes) ||
      !StorageDisjoint(this, *impl_,
          activity.current, activity_bytes) ||
      !Disjoint(receipt, sizeof(*receipt),
          positions.data, position_bytes) ||
      !Disjoint(receipt, sizeof(*receipt),
          activity.base, activity_bytes) ||
      !Disjoint(receipt, sizeof(*receipt),
          activity.current, activity_bytes) ||
      !Disjoint(positions.data, position_bytes,
          activity.base, activity_bytes) ||
      !Disjoint(positions.data, position_bytes,
          activity.current, activity_bytes) ||
      (!activity_same &&
       !Disjoint(activity.base, activity_bytes,
                 activity.current, activity_bytes)))
    return Fail(S::InvalidInput,
        "Current query inputs/output alias retained source or staging",
        parents, facets);
  if (impl_->generation ==
      std::numeric_limits<std::uint64_t>::max())
    return Fail(S::ResourceLimit,
        "Current query generation is exhausted", parents, facets);

  SelfContactCurrentRegularitySummary summary;
  auto report = current_regularity::RunQuery(
      impl_->binding, impl_->forecast, positions, activity,
      impl_->staging, impl_->storage.facet_staging, &summary);
  if (report.status != S::Ok) return report;

  const std::uint64_t generation = impl_->generation + 1;
  summary.generation = generation;
  std::swap(impl_->publication, impl_->staging);
  impl_->summary = summary;
  impl_->positions_identity = positions;
  impl_->activity_identity = activity;
  impl_->generation = generation;
  impl_->complete = true;

  SelfContactCurrentRegularityReceipt next;
  next.owner_identity_ = impl_.get();
  next.binding_identity_ = &impl_->binding;
  next.generation_ = generation;
  next.positions_ = positions;
  next.activity_ = activity;
  *receipt = next;
  return report;
}

SelfContactCurrentRegularityReport
SelfContactCurrentRegularity::ExcludeCertifiedOwnParent(
    const SelfContactPairClassification& pair,
    const SelfContactCurrentRegularityReceipt& receipt,
    SelfContactPairClassification* output) const noexcept {
  if (!impl_)
    return Fail(S::NotInitialized,
        "Current-regularity binding is not initialized");
  const auto parents = impl_->forecast.parents;
  const auto facets = impl_->forecast.facets;
  using tl::fea::trial_identity::Disjoint;
  if (!output ||
      !StorageDisjoint(this, *impl_, output, sizeof(*output)) ||
      !Disjoint(output, sizeof(*output), &pair, sizeof(pair)) ||
      !Disjoint(output, sizeof(*output), &receipt, sizeof(receipt)))
    return Fail(S::InvalidInput,
        "Own-parent decision output aliases input/source/staging",
        parents, facets);
  if (!impl_->complete ||
      receipt.owner_identity_ != impl_.get() ||
      receipt.binding_identity_ != &impl_->binding ||
      receipt.generation_ != impl_->generation ||
      !Same(receipt.positions_, impl_->positions_identity) ||
      !Same(receipt.activity_, impl_->activity_identity))
    return Fail(S::ReceiptMismatch,
        "Regularity receipt is foreign, stale or has wrong query identity",
        parents, facets);
  if (!impl_->binding.Authenticates(pair) ||
      pair.activity_base_identity != receipt.activity_.base ||
      pair.activity_current_identity != receipt.activity_.current ||
      pair.activity_parent_count != receipt.activity_.parent_count)
    return Fail(S::ReceiptMismatch,
        "Active-use pair does not share the receipt binding/activity identity",
        parents, facets);
  if (pair.status !=
          SelfContactPairStatus::SameParentNeedsCurrentRegularity ||
      pair.parent[0] != pair.parent[1] ||
      pair.parent[0] >= parents || !pair.active[0] || !pair.active[1])
    return Fail(S::InvalidPair,
        "Pair is not an active unresolved exact own-parent pair",
        parents, facets);
  const auto parent = pair.parent[0];
  const auto& certified = impl_->publication[parent];
  if (!certified.geometry_evaluated ||
      certified.binding_parent != parent ||
      certified.state != SelfContactCurrentParentState::Active ||
      certified.chart ==
          SelfContactCurrentChartStatus::SkippedLongInactive ||
      certified.facets_evaluated != certified.facet_count)
    return Fail(S::ReceiptMismatch,
        "Pair parent is not certified regular and active in this receipt",
        parents, facets, parent);

  auto next = pair;
  next.status = SelfContactPairStatus::ExcludedRegularOwnParent;
  next.excluded = true;
  next.candidate_directed_area_m2 = {};
  next.admitted_force_area_m2 = {};
  *output = next;
  return {S::Ok, parent, SIZE_MAX, parents, facets, "OK"};
}

const SelfContactActiveUseBinding*
SelfContactCurrentRegularity::binding() const noexcept {
  return impl_ ? &impl_->binding : nullptr;
}

SelfContactCurrentRegularityForecast
SelfContactCurrentRegularity::forecast() const noexcept {
  return impl_ ? impl_->forecast :
      SelfContactCurrentRegularityForecast{};
}

SelfContactCurrentRegularityView
SelfContactCurrentRegularity::results() const noexcept {
  if (!impl_ || !impl_->complete) return {};
  return {impl_->publication, impl_->forecast.parents,
          true, impl_->summary};
}

}  // namespace tlfea::contact
