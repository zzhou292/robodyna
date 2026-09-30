// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../../assembly/SolidNodeContributions.h"
#include "../solid18/Solid18ForceTypes.h"
#include "../solid24/Solid24ForceTypes.h"
#include "../solid6z/Solid6zForceTypes.h"
#include "../solid18/law44/ForceTypes.h"
#include "../../materials/law90/Types.h"
#include "control/Selection.h"

namespace tl::fea::solids {
using Family = SolidCoefficientFamily;
// Caller-supplied prepared mechanics. Repeated source MIDs must have identical
// named material values; LAW36/44/90 curves are borrowed only during Initialize.
struct Input18 { solid18::Reference reference; solid18::Material material; };
struct Input24 { solid24::Reference reference; solid24::Material material; };
struct Input6z {
  solid6z::Reference reference;
  solid6z::Material material;
  solid6z::ForceProfile profile;
};
struct Input18Law44 { solid18::law44::Reference reference; solid18::law44::Material material; };
struct Input18Law90 { solid18::total_strain::Reference reference; tl::material::law90::PreparedMaterial material; };
// The extended profile requires at least one LAW44 or LAW90 parent. It does not
// authorize current three-family resident execution.
enum class ModelProfile { OriginalThreeFamilies, ExtendedLaw44Law90 };
struct ModelInput {
  std::uint64_t source_instance_id = 0;
  util::ConstView<Input18> solid18{nullptr,0};
  util::ConstView<Input24> solid24{nullptr,0};
  util::ConstView<Input6z> solid6z{nullptr,0};
  util::ConstView<Input18Law44> solid18_law44{nullptr,0};
  util::ConstView<Input18Law90> solid18_law90{nullptr,0};
  ModelProfile profile = ModelProfile::OriginalThreeFamilies;
  control::Input controls{}; // Optional semantic source selection; legacy stores no rows.
};
struct Material36 { std::uint64_t source_material_id = 0; solid18::Material value; };
struct Material42 { std::uint64_t source_material_id = 0; solid24::Material value; };
struct Material44 { std::uint64_t source_material_id = 0; solid18::law44::Material value; };
struct Material90 { std::uint64_t source_material_id = 0; tl::material::law90::PreparedMaterial value; };
struct Parent18Law44 {
  solid18::law44::Reference reference;
  std::size_t material_index = SIZE_MAX;
  std::size_t domain_nodes[8]{};
};
struct Parent18Law90 {
  solid18::total_strain::Reference reference;
  std::size_t material_index = SIZE_MAX;
  std::size_t domain_nodes[8]{};
};
struct Parent18 {
  solid18::Reference reference;
  std::size_t material_index = SIZE_MAX;
  std::size_t domain_nodes[8]{};
};
struct Parent24 {
  solid24::Reference reference;
  std::size_t material_index = SIZE_MAX;
  std::size_t domain_nodes[8]{};
};
struct Parent6z {
  solid6z::Reference reference;
  solid6z::ForceProfile profile;
  std::size_t material_index = SIZE_MAX;
  // Six original typed source slots; the last two entries are canonical SIZE_MAX.
  std::size_t domain_nodes[8]{};
};
struct ModelLimits {
  std::size_t max_parents = 16384;
  std::size_t max_materials = 1024;
  std::size_t max_curve_points = 1048576;
  std::size_t max_nodes = 524288;
  std::size_t max_host_bytes = 256u << 20;
};
enum class ModelStatus {
  Success, AlreadyInitialized, InvalidInput, ResourceLimit, DuplicateIdentity,
  MaterialMismatch, SourceMismatch
};
struct ModelReport {
  ModelStatus status = ModelStatus::Success;
  const char* message = "OK";
  Family family = Family::Solid18;
  std::size_t parent = SIZE_MAX;
  explicit operator bool() const noexcept { return status == ModelStatus::Success; }
};

// Immutable complete mechanics identity, with one owned curve pool deduplicated
// by original MID, explicitly profiled typed parent spans and an exact coefficient snapshot.
// The domain is shared and all material curves are copied into the owned arena.
// LAW90 is admitted only with its single reference density PM1=PM89. No force cache,
// clock, inverse coefficients, source-coverage claim or owner admission is made.
class Model {
 public:
  Model() = default;
  Model(const Model&) noexcept = default;
  Model(Model&& other) noexcept : impl_(other.impl_) {}
  Model& operator=(const Model&) = delete;
  Model& operator=(Model&&) = delete;
  ModelReport Initialize(const NodalNodeDomain&, ModelInput, ModelLimits = {}) noexcept;
  bool prepared() const noexcept { return bool(impl_); }
  ModelProfile profile() const noexcept;
  const control::Selection* control_selection() const noexcept;
  std::uint64_t source_instance_id() const noexcept;
  const NodalNodeDomain* domain() const noexcept;
  const SolidNodeContributions* contributions() const noexcept;
  util::ConstView<Parent18> solid18() const noexcept;
  util::ConstView<Parent24> solid24() const noexcept;
  util::ConstView<Parent6z> solid6z() const noexcept;
  util::ConstView<Parent18Law44> solid18_law44() const noexcept;
  util::ConstView<Parent18Law90> solid18_law90() const noexcept;
  util::ConstView<Material44> materials44() const noexcept;
  util::ConstView<Material90> materials90() const noexcept;
  util::ConstView<Material36> materials36() const noexcept;
  util::ConstView<Material42> materials42() const noexcept;
  bool Matches(const Model&) const noexcept;
  bool SharesStorage(const Model&) const noexcept;
  std::size_t owned_payload_bytes() const noexcept;
  std::size_t startup_payload_bytes() const noexcept;
 private:
  struct Impl;
  std::shared_ptr<const Impl> impl_;
};
} // namespace tl::fea::solids
