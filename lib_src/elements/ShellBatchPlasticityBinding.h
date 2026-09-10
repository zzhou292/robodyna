#pragma once
#include "ShellBatchBinding.h"
#include "ShellBatchPlasticity.h"
#include "ShellPlasticityCatalogLimits.h"

namespace tl::fea {
inline constexpr std::size_t MaxShellPlasticityCurvePoints=1024;
struct ShellPlasticityCurveInput {
  std::uint64_t curve_id=0;
  material::TabulatedShellPlasticityCurve curve;
};
struct ShellPlasticityMaterialInput {
  std::uint64_t material_id=0,curve_id=0;
  double young_pa=0,poisson_ratio=0,density_kg_m3=0;
  material::TabulatedShellPlasticityRate rate;
  // Trailing tagged declaration preserves older positional aggregates.
  // LinearLaw44 requires curve_id=0 and owns its source SIGY/ETAN values.
  material::ShellPlasticityHardeningKind hardening=material::ShellPlasticityHardeningKind::Tabulated;
  material::Law44LinearHardening linear{};
};
struct ShellPlasticitySectionInput {
  std::uint64_t section_id=0;
  double thickness_m=0;
  unsigned through_thickness_points=3;
};
struct ShellPlasticityParentInput {
  ShellBindingFamily family=ShellBindingFamily::None;
  std::size_t family_index=NoShellBindingNode;
  std::uint64_t source_parent_id=0,source_part_id=0,material_id=0,section_id=0;
};
// Borrowed startup declarations. IDs are unique within each declaration kind;
// every declaration is referenced, and every native parent occurs exactly once.
// Explicit input parent order is retained, never inferred by sorting source IDs.
struct ShellBatchPlasticityBindingInput {
  const ShellPlasticityCurveInput* curves=nullptr;
  const ShellPlasticityMaterialInput* materials=nullptr;
  const ShellPlasticitySectionInput* sections=nullptr;
  const ShellPlasticityParentInput* parents=nullptr;
  std::size_t curve_count=0,material_count=0,section_count=0,parent_count=0;
};
enum class ShellPlasticityBindingStatus {
  Success,AlreadyInitialized,InvalidInput,ResourceLimit,InvalidCurve,
  InvalidMaterial,InvalidSection,InvalidParent,IdentityMismatch,UnreferencedDeclaration,
};
struct ShellPlasticityBindingReport {
  ShellPlasticityBindingStatus status=ShellPlasticityBindingStatus::Success;
  std::size_t entry=NoShellBindingNode;
  ShellBindingFamily family=ShellBindingFamily::None;
  const char* message="OK";
};
namespace shell_batch_plasticity_detail { class HostStorage; }

// Immutable host catalog for the COMPLETE authenticated native collection.
// All curves/declarations/mapping are owned after Initialize. Stored prepared
// coefficients have null curve pointers; Parameters rebinds a returned value to
// this object's pool, so copying/moving the catalog cannot retain stale pointers.
// This object adds no owner, clock, accepted/trial state or publication authority.
class ShellBatchPlasticityBinding {
 public:
  ShellBatchPlasticityBinding()=default;
  ShellBatchPlasticityBinding(const ShellBatchPlasticityBinding&) noexcept=default;
  ShellBatchPlasticityBinding(ShellBatchPlasticityBinding&& other) noexcept
      :ShellBatchPlasticityBinding(static_cast<const ShellBatchPlasticityBinding&>(other)) {}
  ShellBatchPlasticityBinding& operator=(const ShellBatchPlasticityBinding&)=delete;
  ShellPlasticityBindingReport Initialize(const ShellBatchBinding&,const ShellBatchPlasticityBindingInput&) noexcept;
  ShellPlasticityBindingReport Initialize(const ShellBatchBinding&,const ShellBatchPlasticityBindingInput&,
      const ShellHostBindingLimits&) noexcept;
  // Explicit host-only vehicle admission. Legacy Initialize overloads retain
  // their original bounds, including rejection of ShellHostBindingLimits::Vehicle().
  ShellPlasticityBindingReport InitializeCatalog(const ShellBatchBinding&,
      const ShellBatchPlasticityBindingInput&,const ShellPlasticityCatalogLimits&) noexcept;
  // Includes complete inventory backing, even when shared with the binding.
  std::size_t host_bytes() const noexcept;
  std::size_t startup_scratch_bytes() const noexcept;
  bool prepared() const noexcept { return prepared_; }
  bool Matches(const ShellBatchBinding&) const noexcept;
  bool SameScope(const ShellBatchPlasticityBinding&) const noexcept;
  std::size_t material_count() const noexcept { return data_.material_count; }
  std::size_t curve_count() const noexcept { return data_.curve_count; }
  std::size_t section_count() const noexcept { return data_.section_count; }
  std::size_t parent_count() const noexcept { return data_.parent_count; }
  std::size_t curve_point_count() const noexcept { return data_.point_count; }
  const ShellBatchInventory& inventory() const noexcept { return data_.inventory; }
  const ShellPlasticityParentInput* parent(std::size_t input_index) const noexcept;
  // Caller keeps this catalog alive while using the returned host curve view.
  // Failure leaves output untouched; invalid family/index never selects zero.
  bool Parameters(ShellBindingFamily,std::size_t family_index,sections::PointParameters* output) const noexcept;
 private:
  struct Curve { std::uint64_t id=0; std::size_t offset=0,count=0; };
  struct Material {
    ShellPlasticityMaterialInput declaration;
    sections::PointParameters coefficients; // curve is ALWAYS empty in owned storage.
    std::size_t curve_index=0;
  };
  struct Parent {
    ShellPlasticityParentInput declaration;
    std::size_t material_index=0,section_index=0;
  };
  struct Data {
    ShellBatchInventory inventory;
    tl::util::BoundedStartupArray<Curve,MaxShellCollectionParents> curves;
    tl::util::BoundedStartupArray<Material,MaxShellCollectionParents> materials;
    tl::util::BoundedStartupArray<ShellPlasticitySectionInput,MaxShellCollectionParents> sections;
    tl::util::BoundedStartupArray<Parent,MaxShellCollectionParents> parents;
    tl::util::BoundedStartupArray<std::size_t,MaxShellCollectionParents> qeph_parent,t3_parent;
    std::array<double,MaxShellPlasticityCurvePoints> curve_x{},curve_y{};
    std::size_t curve_count=0,material_count=0,section_count=0,parent_count=0,point_count=0;
    std::size_t qeph_count=0,t3_count=0;
  } data_;
  bool prepared_=false;
  ShellPlasticityBindingReport Build(const ShellBatchBinding&,const ShellBatchPlasticityBindingInput&);
  static ShellPlasticityBindingReport CopyCurves(const ShellBatchPlasticityBindingInput&,Data&) noexcept;
  static ShellPlasticityBindingReport PrepareMaterials(const ShellBatchPlasticityBindingInput&,Data&) noexcept;
  static ShellPlasticityBindingReport CopySections(const ShellBatchPlasticityBindingInput&,Data&) noexcept;
  static ShellPlasticityBindingReport BindParents(const ShellBatchBinding&,const ShellBatchPlasticityBindingInput&,Data&);
  friend class shell_batch_plasticity_detail::HostStorage;
};
static_assert(sizeof(ShellBatchPlasticityBinding)<128*1024,"Bounded host-only material catalog");
} // namespace tl::fea
