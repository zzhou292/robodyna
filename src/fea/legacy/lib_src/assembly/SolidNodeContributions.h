// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "NodalNodeDomain.h"
#include "../elements/solid18/Solid18Types.h"
#include "../elements/solid24/Solid24Types.h"
#include "../elements/solid6z/Solid6zTypes.h"
#include "../elements/solid18/law44/Types.h"
#include "../elements/solid18/total_strain/ReferenceTypes.h"

namespace tl::fea {
enum class SolidCoefficientFamily { Solid18, Solid24, Solid6z, Solid18Law44, Solid18Law90 };
enum class SolidCoefficientProfile { OriginalThreeFamilies, ExtendedLaw44Law90 };
struct SolidCoefficientInput {
  std::uint64_t source_instance_id = 0;
  const solid18::Reference* solid18 = nullptr;
  std::size_t solid18_count = 0;
  const solid24::Reference* solid24 = nullptr;
  std::size_t solid24_count = 0;
  const solid6z::Reference* solid6z = nullptr;
  std::size_t solid6z_count = 0;
  // Explicit extension; the legacy aggregate prefix and family order stay fixed.
  const solid18::law44::Reference* law44 = nullptr;
  std::size_t law44_count = 0;
  const solid18::total_strain::Reference* law90 = nullptr;
  std::size_t law90_count = 0;
  SolidCoefficientProfile profile = SolidCoefficientProfile::OriginalThreeFamilies;
};
struct SolidCoefficientParent {
  SolidCoefficientFamily family = SolidCoefficientFamily::Solid18;
  std::uint64_t source_element_id = 0, source_part_id = 0;
  std::uint64_t source_section_id = 0, source_material_id = 0;
  unsigned node_count = 0;
  std::uint64_t source_node_id[8]{};
  std::size_t domain_node[8]{};
  double mass_kg[8]{};
  // Selected native SMASS3/S6MASS3 writes no nodal rotational inertia.
  static constexpr double isotropic_inertia_kg_m2() noexcept { return 0; }
};
struct SolidCoefficientLimits {
  std::size_t max_parents = 16384, max_nodes = 524288;
  std::size_t max_host_bytes = 256 * 1024 * 1024;
};

// Closed coefficient snapshot from prepared typed references. Parent order is
// solid18, solid24, solid6z, preserving each input order and original source
// slots (not native reorientation). No raw caller-supplied coefficient entry.
// The immutable domain and exact native masses are retained, not borrowed refs.
// The extension appends LAW44 and LAW90; a repeated H8 keeps all eight slots.
// Extended snapshots require a later explicit ledger/owner profile admission.
// Matches authenticates coefficient identity only; mechanics models must retain
// their complete references/material profiles separately. No force/DOF admission.
class SolidNodeContributions {
 public:
  SolidNodeContributions() = default;
  SolidNodeContributions(const SolidNodeContributions&) noexcept = default;
  SolidNodeContributions(SolidNodeContributions&& other) noexcept
      : SolidNodeContributions(static_cast<const SolidNodeContributions&>(other)) {}
  SolidNodeContributions& operator=(const SolidNodeContributions&) = delete;
  // Report.node is the combined input parent index on a row error.
  NodalDomainReport Initialize(const NodalNodeDomain&, SolidCoefficientInput,
                              SolidCoefficientLimits = {}) noexcept;
  bool prepared() const noexcept { return bool(impl_); }
  const NodalNodeDomain* domain() const noexcept;
  SolidCoefficientProfile profile() const noexcept;
  tl::util::ConstView<SolidCoefficientParent> parents() const noexcept;
  std::size_t parent_count(SolidCoefficientFamily) const noexcept;
  bool Matches(const SolidNodeContributions&) const noexcept;
  std::size_t owned_payload_bytes() const noexcept;
  std::size_t startup_payload_bytes() const noexcept;
 private:
  struct Impl;
  std::shared_ptr<const Impl> impl_;
};
}  // namespace tl::fea
