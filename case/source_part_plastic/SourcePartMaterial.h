#pragma once
#include "lib_src/materials/TabulatedShellPlasticity.h"
#include <array>
#include <cstdint>
#include <filesystem>
#include <string>

namespace crash::cases::source_part_plastic {
inline constexpr std::size_t MaterialCurvePoints = 46;
enum class FilterCutoffResolution { UnresolvedReaderDefault, OpenRadiossDirectImportDefault };
struct SourceMaterialDeclaration {
    std::uint64_t part_id = 0, material_id = 0, section_id = 0, curve_id = 0;
    double young_pa = 0, poisson_ratio = 0, density_kg_m3 = 0, thickness_m = 0;
    double supplied_sigy_pa = 0;
    double source_rate_coefficient_per_s = 0, source_rate_exponent = 0;
    int source_vp = 0;
    unsigned source_elform = 0, through_thickness_points = 0;
    std::array<std::uint16_t, 4> material_blank_masks{};
    std::array<std::uint32_t, 4> material_source_lines{};
    bool source_lcsr_blank = true, source_failure_blank = true, source_tdel_blank = true;
    // MAT024 has no direct Fcut field. The direct converted-model getter gives
    // zero for its absent field; the pinned starter resolves ISMOOTH=1 to 10000/s.
    FilterCutoffResolution filter_cutoff = FilterCutoffResolution::UnresolvedReaderDefault;
    double resolved_filter_cutoff_per_s = 0;
};
enum class MaterialStatus { Ok, InvalidArgument, ReadFailure, HashMismatch, InvalidDeclaration };
struct MaterialReport {
    MaterialStatus status = MaterialStatus::InvalidArgument;
    std::string message;
    explicit operator bool() const noexcept { return status == MaterialStatus::Ok; }
};
class SourcePartMaterial;
MaterialReport LoadPinnedSourcePartMaterial(const std::filesystem::path&, SourcePartMaterial*);

// Owning source declaration only. The selected solver rate/stabilization policy
// belongs to the case configuration; reading this object never disables the
// source C/P values or claims complete MAT024 equivalence. Copies own their curve.
class SourcePartMaterial {
  public:
    bool prepared() const noexcept { return prepared_; }
    const SourceMaterialDeclaration& declaration() const noexcept { return declaration_; }
    const auto& plastic_strain() const noexcept { return plastic_strain_; }
    const auto& yield_stress_pa() const noexcept { return yield_stress_pa_; }
    tl::material::TabulatedShellPlasticityCurve curve() const noexcept {
        return prepared_ ? tl::material::TabulatedShellPlasticityCurve{
            plastic_strain_.data(), yield_stress_pa_.data(), MaterialCurvePoints}
            : tl::material::TabulatedShellPlasticityCurve{};
    }
    static constexpr const char* ExperimentalRateOffPolicy =
        "Experimental rate-independent LAW44-derived plane stress; original curve retained; "
        "source C/P and rate filtering explicitly inactive; no failure or curve extrapolation; "
        "not complete source MAT024 equivalence";
  private:
    SourceMaterialDeclaration declaration_;
    std::array<double, MaterialCurvePoints> plastic_strain_{}, yield_stress_pa_{};
    bool prepared_ = false;
    friend MaterialReport LoadPinnedSourcePartMaterial(const std::filesystem::path&, SourcePartMaterial*);
};
static_assert(sizeof(SourcePartMaterial) < 2048, "Fixed source material setup storage");
} // namespace crash::cases::source_part_plastic
