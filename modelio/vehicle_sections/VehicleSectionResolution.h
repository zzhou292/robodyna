#pragma once
#include "modelio/vehicle_source/VehicleSourcePlan.h"
#include "lib_src/elements/ShellBatchFailureBinding.h"
#include "lib_src/elements/ShellReferencePlacement.h"
#include <memory>

namespace crash::modelio::vehicle {
inline constexpr const char* ResolutionSchema = "robo-dyna.vehicle-section-resolution.v1";
inline constexpr const char* GlassResolutionSchema = "robo-dyna.vehicle-section-resolution.v2";
struct ResolutionLimits {
    std::size_t declaration_bytes = 4 * 1024 * 1024;
    std::size_t host_bytes = 512 * 1024 * 1024;
    std::size_t parents = 524288, parts = 1024, tables = 1024, curve_points = 1024;
};
enum class SectionDisposition { Existing, ConstantFailure, Unresolved, GlassTab1 };
enum class SourceShellTopology { Q4, T3 };
struct SectionPartResolution {
    SectionDisposition status = SectionDisposition::Unresolved;
    std::size_t material_index = SIZE_MAX, section_index = SIZE_MAX;
    double failure_strain = 0;
    tl::fea::ShellReferencePlacement placement = tl::fea::ShellReferencePlacement::Centered;
    std::optional<double> source_nloc;
};
struct SectionParentResolution {
    std::uint64_t source_parent_id = 0;
    std::uint32_t canonical_parent = 0, part_index = 0, topology_index = 0;
    SourceShellTopology topology = SourceShellTopology::Q4;
};
struct ResolutionCounts {
    std::size_t parts = 0, shells = 0;
    std::size_t existing_parts = 0, existing_shells = 0;
    std::size_t failure_parts = 0, failure_shells = 0;
    std::size_t unresolved_parts = 0, unresolved_shells = 0;
    std::size_t glass_parts = 0, glass_shells = 0, placed_glass_shells = 0;
};
// Source values only. Retains the historical plan and its shared canonical backing.
// Unavailable rows remain in the complete topology index; no partial native binding,
// node mass, force/history, owner, contact or connected load-path admission exists.
class VehicleSectionResolution {
  public:
    static VehicleSectionResolution Read(const VehicleSourcePlan&, const std::filesystem::path&,
                                         const assembly::ArtifactIdentity&, ResolutionLimits = {});
    static VehicleSectionResolution ReadBytes(const VehicleSourcePlan&, const std::string&,
                                              const assembly::ArtifactIdentity&, ResolutionLimits = {});
    VehicleSectionResolution(const VehicleSectionResolution&) noexcept = default;
    VehicleSectionResolution(VehicleSectionResolution&& other) noexcept : data_(other.data_) {}
    VehicleSectionResolution& operator=(const VehicleSectionResolution&) = delete;
    VehicleSectionResolution& operator=(VehicleSectionResolution&&) = delete;
    const VehicleSourcePlan& source() const noexcept;
    const assembly::ArtifactIdentity& identity() const noexcept;
    const ResolutionCounts& counts() const noexcept;
    bool includes_glass() const noexcept;
    const std::vector<SectionPartResolution>& parts() const noexcept;
    const std::vector<SectionParentResolution>& parents() const noexcept;
    // Glass Material exposes physical/analytic coefficients and literal cards;
    // generic rate fields are unused. native_material() supplies its explicit
    // FilteredZeroC policy without fabricating original C/P/VP declarations.
    const assembly::Material* material(std::size_t part_index) const noexcept;
    const assembly::Section* section(std::size_t part_index) const noexcept;
    const tl::fea::ShellPlasticityMaterialInput* native_material(std::size_t part_index) const noexcept;
    // Source-order topology indices include unavailable slots. A returned value
    // alone cannot initialize a partial complete-family TL collection.
    const tl::fea::ShellFailureParentInput* native_parent(std::size_t parent_index) const noexcept;
    const std::vector<assembly::Curve>& failure_curves() const noexcept;
    std::size_t startup_budget_bytes() const noexcept;
  private:
    struct Data;
    explicit VehicleSectionResolution(std::shared_ptr<const Data> data) : data_(std::move(data)) {}
    std::shared_ptr<const Data> data_;
};
} // namespace crash::modelio::vehicle
