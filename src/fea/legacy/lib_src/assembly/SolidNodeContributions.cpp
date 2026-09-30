// SPDX-License-Identifier: AGPL-3.0-or-later
#include "SolidContributionChecks.h"
#include "../../lib_utils/BoundedArena.h"
#include "../../lib_utils/SourceIdentityIndex.h"
#include <new>
#include <stdexcept>

namespace tl::fea {
struct SolidNodeContributions::Impl {
  explicit Impl(const NodalNodeDomain& d) : domain(d) {}
  NodalNodeDomain domain;
  util::HostArena arena;
  SolidCoefficientParent* rows = nullptr;
  std::size_t count[5]{}, retained = 0, startup = 0;
  SolidCoefficientProfile profile = SolidCoefficientProfile::OriginalThreeFamilies;
  std::size_t Count() const noexcept {
    return count[0]+count[1]+count[2]+count[3]+count[4];
  }
};
NodalDomainReport SolidNodeContributions::Initialize(const NodalNodeDomain& domain,
    SolidCoefficientInput input, SolidCoefficientLimits limits) noexcept try {
  using S = NodalDomainStatus;
  using F = SolidCoefficientFamily;
  if (impl_) return {S::AlreadyInitialized, "Solid coefficient snapshot is immutable"};
  if (!domain.prepared() || input.source_instance_id != domain.source_instance_id())
    return {S::InvalidInput, "Prepared matching source domain is required"};
  using Profile = SolidCoefficientProfile;
  if ((input.profile == Profile::OriginalThreeFamilies &&
       (input.law44 || input.law44_count || input.law90 || input.law90_count)) ||
      (input.profile == Profile::ExtendedLaw44Law90 && !input.law44_count && !input.law90_count) ||
      (input.profile != Profile::OriginalThreeFamilies && input.profile != Profile::ExtendedLaw44Law90))
    return {S::InvalidInput, "Solid coefficient extension requires its explicit profile"};
  const SolidCoefficientLimits hard;
  if (!limits.max_parents || limits.max_parents > hard.max_parents ||
      !limits.max_nodes || limits.max_nodes > hard.max_nodes ||
      !limits.max_host_bytes || limits.max_host_bytes > hard.max_host_bytes ||
      domain.node_count() > limits.max_nodes || input.solid18_count > limits.max_parents ||
      input.solid24_count > limits.max_parents || input.solid6z_count > limits.max_parents ||
      input.law44_count > limits.max_parents || input.law90_count > limits.max_parents)
    return {S::ResourceLimit, "Solid coefficient counts or caps exceed scope"};
  const auto count = input.solid18_count + input.solid24_count + input.solid6z_count +
                     input.law44_count + input.law90_count;
  if (count > limits.max_parents) return {S::ResourceLimit, "Combined solid count exceeds cap"};
  if (!count) return {S::InvalidInput, "At least one solid reference is required"};
  util::BoundedArenaLayout arena(limits.max_host_bytes), retained(limits.max_host_bytes),
      startup(limits.max_host_bytes);
  util::ArenaRegion rows, ignored;
  const auto domain_bytes = domain.owned_payload_bytes();
  if (domain_bytes < sizeof(NodalNodeDomain) ||
      !arena.Append<SolidCoefficientParent>(count, rows) ||
      !retained.Append<unsigned char>(sizeof(*this) + sizeof(Impl) + 64, ignored) ||
      !retained.Append<unsigned char>(arena.bytes(), ignored) ||
      !retained.Append<unsigned char>(domain_bytes - sizeof(NodalNodeDomain), ignored) ||
      !startup.Append<unsigned char>(retained.bytes(), ignored) ||
      !startup.Append<unsigned char>(util::SourceIdentityIndex<0>::Bytes(count), ignored))
    return {S::ResourceLimit, "Solid coefficients, retained domain and index exceed cap"};
  auto range = [](const auto* p, std::size_t n) {
    return n ? nodal_domain_detail::ValidRange(p, n) : !p;
  };
  if (!range(input.solid18, input.solid18_count) || !range(input.solid24, input.solid24_count) ||
      !range(input.solid6z, input.solid6z_count) || !range(input.law44, input.law44_count) ||
      !range(input.law90, input.law90_count))
    return {S::InvalidInput, "Solid reference pointer/count pair is invalid"};
  auto id = [&](std::size_t i) {
    return solid_coefficient_detail::Visit(input, i, [](const auto& ref, F, unsigned) {
      return ref.input().source_element_id;
    });
  };
  util::SourceIdentityIndex<0> identities;
  identities.Prepare(count, id);
  auto next = std::make_shared<Impl>(domain);
  if (!next->arena.Initialize(arena.bytes()) ||
      !(next->rows = next->arena.Construct<SolidCoefficientParent>(rows)))
    return {S::ResourceLimit, "Solid coefficient arena allocation failed"};
  for (std::size_t i = 0; i < count; ++i) {
    if (!id(i)) return {S::InvalidInput, "Solid EID must be positive", i};
    if (identities.First(id(i)) != i)
      return {S::DuplicateIdentity, "Repeated original solid EID", i};
    const auto report = solid_coefficient_detail::Visit(input, i,
        [&](const auto& ref, F family, unsigned arity) {
          return solid_coefficient_detail::Map(ref, domain, family, arity, next->rows[i], i);
        });
    if (!report) return report;
  }
  next->count[0] = input.solid18_count;
  next->count[1] = input.solid24_count;
  next->count[2] = input.solid6z_count;
  next->count[3] = input.law44_count;
  next->count[4] = input.law90_count;
  next->profile = input.profile;
  next->retained = retained.bytes();
  next->startup = startup.bytes();
  impl_ = std::move(next);
  return {};
} catch (const std::bad_alloc&) {
  return {NodalDomainStatus::ResourceLimit, "Solid coefficient allocation failed"};
} catch (const std::length_error&) {
  return {NodalDomainStatus::ResourceLimit, "Solid coefficient allocation size overflow"};
}
const NodalNodeDomain* SolidNodeContributions::domain() const noexcept {
  return impl_ ? &impl_->domain : nullptr;
}
SolidCoefficientProfile SolidNodeContributions::profile() const noexcept {
  return impl_ ? impl_->profile : SolidCoefficientProfile::OriginalThreeFamilies;
}
tl::util::ConstView<SolidCoefficientParent> SolidNodeContributions::parents() const noexcept {
  static const SolidCoefficientParent empty;
  return {impl_ ? impl_->rows : &empty,
      impl_ ? impl_->Count() : 0};
}
std::size_t SolidNodeContributions::parent_count(SolidCoefficientFamily family) const noexcept {
  const auto i = static_cast<unsigned>(family);
  return impl_ && i < 5 ? impl_->count[i] : 0;
}
bool SolidNodeContributions::Matches(const SolidNodeContributions& other) const noexcept {
  if (!impl_ || !other.impl_) return false;
  if (impl_ == other.impl_) return true;
  if (profile() != other.profile() || !impl_->domain.Matches(other.impl_->domain) ||
      parents().size() != other.parents().size()) return false;
  for (std::size_t i = 0; i < parents().size(); ++i)
    if (!solid_coefficient_detail::Same(impl_->rows[i], other.impl_->rows[i])) return false;
  return true;
}
std::size_t SolidNodeContributions::owned_payload_bytes() const noexcept {
  return impl_ ? impl_->retained : sizeof(*this);
}
std::size_t SolidNodeContributions::startup_payload_bytes() const noexcept {
  return impl_ ? impl_->startup : sizeof(*this);
}
}  // namespace tl::fea
