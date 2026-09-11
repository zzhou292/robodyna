#include "../ShellBatchFailureBinding.h"
#include "../sections/ShellLayeredTab1ForceAdapter.h"
#include "lib_utils/BoundedArena.h"
#include <cstring>
#include <new>
#include <stdexcept>

namespace tl::fea {
struct ShellBatchFailureBinding::Data {
  explicit Data(const ShellBatchPlasticityBinding& value) : catalog(value) {}
  ShellBatchPlasticityBinding catalog;
  util::BoundedStartupArray<ShellFailureParentInput, 0> rows;
  util::BoundedStartupArray<std::size_t, 0> qeph, t3;
  std::size_t count = 0, bytes = 0;
};
namespace {
using Status = ShellPlasticityBindingStatus;
bool Same(const ShellPlasticityParentInput& a, const ShellPlasticityParentInput& b) noexcept {
  return a.family == b.family && a.family_index == b.family_index &&
      a.source_parent_id == b.source_parent_id && a.source_part_id == b.source_part_id &&
      a.material_id == b.material_id && a.section_id == b.section_id;
}
bool Same(double a, double b) noexcept {
  return std::memcmp(&a, &b, sizeof a) == 0;
}
bool Same(const sections::ShellLayeredTab1Parameters& a,
          const sections::ShellLayeredTab1Parameters& b) noexcept {
  if (a.parent_policy != b.parent_policy || !Same(a.table.failure_strain, b.table.failure_strain)) {
    return false;
  }
  for (unsigned i = 0; i < 3; ++i) {
    if (!Same(a.table.triaxiality[i], b.table.triaxiality[i])) return false;
  }
  return true;
}
bool AdmittedPolicy(const ShellFailureParentInput& row, ShellSectionLaw law,
                    const ShellBatchPlasticityBinding& catalog) noexcept {
  const bool no_tab1 = Same(row.tab1, sections::ShellLayeredTab1Parameters{});
  if (law == ShellSectionLaw::Law44Nip1) {
    return row.policy == ShellFailurePolicy::ConstantAllPoints && no_tab1 &&
        tl::math::Finite(row.constant.failure_strain) && row.constant.failure_strain > 0;
  }
  if (row.policy == ShellFailurePolicy::None) {
    return Same(row.constant.failure_strain, 0.) && no_tab1;
  }
  if (law != ShellSectionLaw::LayeredLaw44Nip3) return false;
  if (row.policy == ShellFailurePolicy::ConstantAllPoints) {
    return no_tab1 && tl::math::Finite(row.constant.failure_strain) && row.constant.failure_strain > 0;
  }
  if (row.policy != ShellFailurePolicy::Tab1AnyPoint || !Same(row.constant.failure_strain, 0.)) return false;
  sections::PointParameters material;
  return catalog.Parameters(row.source.family, row.source.family_index, &material) &&
      sections::ValidLayeredTab1ForceParameters(material, row.tab1);
}
} // namespace

ShellPlasticityBindingReport ShellBatchFailureBinding::Initialize(
    const ShellBatchPlasticityBinding& catalog, const ShellFailureParentInput* input,
    std::size_t count, const ShellBatchFailureLimits& limits) noexcept try {
  if (data_) {
    return {Status::AlreadyInitialized, NoShellBindingNode, ShellBindingFamily::None,
            "Failure binding is immutable"};
  }
  if (!catalog.heterogeneous_sections() || count != catalog.parent_count() || !count) {
    return {Status::InvalidInput, NoShellBindingNode, ShellBindingFamily::None,
            "Failure requires the complete explicit mixed catalog"};
  }
  if (!limits.max_parents || limits.max_parents > 524288 || count > limits.max_parents ||
      !limits.max_host_bytes || limits.max_host_bytes > 512ULL * 1024 * 1024) {
    return {Status::ResourceLimit, NoShellBindingNode, ShellBindingFamily::None,
            "Failure declaration count/host cap exceeded"};
  }
  ShellSectionCounts q, t;
  catalog.Counts(ShellBindingFamily::Qeph, &q);
  catalog.Counts(ShellBindingFamily::T3, &t);
  const auto nq = q.law1 + q.law44, nt = t.law1 + t.law44;
  util::BoundedArenaLayout budget(limits.max_host_bytes);
  util::ArenaRegion ignored;
  // catalog.host_bytes includes its inline object; Data embeds it exactly once.
  if (!budget.Append<unsigned char>(sizeof(Data) - sizeof(ShellBatchPlasticityBinding), ignored) ||
      !budget.Append<unsigned char>(catalog.host_bytes(), ignored) ||
      !budget.Append<ShellFailureParentInput>(count, ignored) ||
      !budget.Append<std::size_t>(count, ignored) ||
      !budget.Append<unsigned char>(4 * 64, ignored)) {
    return {Status::ResourceLimit, NoShellBindingNode, ShellBindingFamily::None,
            "Failure declaration payload exceeds host cap"};
  }
  if (!input) {
    return {Status::InvalidInput, NoShellBindingNode, ShellBindingFamily::None,
            "Missing failure declarations"};
  }
  auto next = std::make_shared<Data>(catalog);
  next->rows.Resize(count);
  next->qeph.Resize(nq);
  next->t3.Resize(nt);
  bool any_failure = false;
  for (std::size_t i = 0; i < count; ++i) {
    const auto& row = input[i];
    const auto* source = catalog.parent(i);
    if (!source || !Same(row.source, *source)) {
      return {Status::IdentityMismatch, i, row.source.family,
              "Failure parent differs from complete catalog source order"};
    }
    ShellSectionLaw law;
    catalog.Law(source->family, source->family_index, &law);
    if (!AdmittedPolicy(row, law, catalog)) {
      return {Status::InvalidMaterial, i, source->family,
              "Failure policy/parameters differ from the qualified complete material catalog"};
    }
    any_failure = any_failure || row.policy != ShellFailurePolicy::None;
    next->rows[i] = row;
    auto& family_index = source->family == ShellBindingFamily::Qeph ? next->qeph : next->t3;
    family_index[source->family_index] = i;
  }
  if (!any_failure) {
    return {Status::InvalidInput, NoShellBindingNode, ShellBindingFamily::None,
            "An all-None declaration uses the ordinary mixed path"};
  }
  next->count = count;
  next->bytes = budget.bytes();
  data_ = std::move(next);
  return {};
} catch (const std::bad_alloc&) {
  return {Status::ResourceLimit};
} catch (const std::length_error&) {
  return {Status::ResourceLimit};
}

const ShellBatchPlasticityBinding* ShellBatchFailureBinding::catalog() const noexcept {
  return data_ ? &data_->catalog : nullptr;
}
std::size_t ShellBatchFailureBinding::parent_count() const noexcept {
  return data_ ? data_->count : 0;
}
std::size_t ShellBatchFailureBinding::host_bytes() const noexcept {
  return data_ ? sizeof(*this) + data_->bytes : sizeof(*this);
}
const ShellFailureParentInput* ShellBatchFailureBinding::parent(std::size_t i) const noexcept {
  return data_ && i < data_->count ? &data_->rows[i] : nullptr;
}
const ShellFailureParentInput* ShellBatchFailureBinding::parent(
    ShellBindingFamily family, std::size_t i) const noexcept {
  if (!data_ || (family != ShellBindingFamily::Qeph && family != ShellBindingFamily::T3)) return nullptr;
  const auto& index = family == ShellBindingFamily::Qeph ? data_->qeph : data_->t3;
  return i < index.size() ? &data_->rows[index[i]] : nullptr;
}
bool ShellBatchFailureBinding::Matches(const ShellBatchPlasticityBinding& value) const noexcept {
  return data_ && data_->catalog.SameScope(value);
}
bool ShellBatchFailureBinding::SameScope(const ShellBatchFailureBinding& other) const noexcept {
  if (!data_ || !other.data_ || !Matches(other.data_->catalog) || data_->count != other.data_->count) {
    return false;
  }
  for (std::size_t i = 0; i < data_->count; ++i) {
    const auto& a = data_->rows[i];
    const auto& b = other.data_->rows[i];
    if (!Same(a.source, b.source) || a.policy != b.policy ||
        !Same(a.constant.failure_strain, b.constant.failure_strain) || !Same(a.tab1, b.tab1)) {
      return false;
    }
  }
  return true;
}
} // namespace tl::fea
