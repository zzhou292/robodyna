#pragma once
#include "modelio/vehicle_source/VehicleSourcePlan.h"
#include "lib_src/elements/ShellBatchFailureBinding.h"
#include "lib_src/elements/ShellReferencePlacement.h"
#include <memory>

namespace crash::modelio::vehicle {
inline constexpr const char* ResolutionSchema = "robo-dyna.vehicle-section-resolution.v1";
inline constexpr const char* OriginalMidlayerProfileName = "original_mat024_elform9_nip1_midlayer_v1";
inline constexpr const char* GlassResolutionSchema = "robo-dyna.vehicle-section-resolution.v2";
struct ResolutionLimits {
    std::size_t declaration_bytes = 4 * 1024 * 1024;
    std::size_t host_bytes = 512 * 1024 * 1024;
    std::size_t parents = 524288, parts = 1024, tables = 1024, curve_points = 1024;
};
enum class SectionDisposition { Existing, ConstantFailure, Unresolved, GlassTab1, Midlayer };
enum class ResolutionProfile { Artifact, OriginalMidlayerV1 };
struct ResolutionKey {
    assembly::ArtifactIdentity artifact;
    ResolutionProfile profile = ResolutionProfile::Artifact;
    bool operator==(const ResolutionKey& other) const noexcept {
        return profile == other.profile && artifact.bytes == other.artifact.bytes &&
               artifact.sha256 == other.artifact.sha256;
    }
};
struct NativeParentMapping {
    tl::fea::ShellBindingFamily family = tl::fea::ShellBindingFamily::None;
    std::size_t family_index = SIZE_MAX;
};
struct NativeFormulationCounts { std::size_t qeph=0, t3=0, qbat=0; };
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
    std::size_t midlayer_parts = 0, midlayer_shells = 0;
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
    // Explicit compiled interpretation of already authenticated original raw cards.
    // Requires the original complete no-tire V2 resolution, never another sidecar.
    static VehicleSectionResolution ResolveOriginalMidlayer(const VehicleSectionResolution&,
        ResolutionProfile, ResolutionLimits = {});
    static std::size_t ForecastOriginalMidlayer(const VehicleSectionResolution&,
        ResolutionProfile, ResolutionLimits = {});
    VehicleSectionResolution(const VehicleSectionResolution&) noexcept = default;
    VehicleSectionResolution(VehicleSectionResolution&& other) noexcept : data_(other.data_) {}
    VehicleSectionResolution& operator=(const VehicleSectionResolution&) = delete;
    VehicleSectionResolution& operator=(VehicleSectionResolution&&) = delete;
    const VehicleSourcePlan& source() const noexcept;
    // Identity of the retained input artifact. Derived-profile association uses
    // resolution_key(), which includes the explicit interpretation version.
    const assembly::ArtifactIdentity& identity() const noexcept;
    const ResolutionKey& resolution_key() const noexcept;
    // Nonzero only for the explicit compiled profile.
    const NativeFormulationCounts& native_counts() const noexcept;
    // Only the compiled profile owns contiguous available-family mappings.
    // Artifact profiles retain their historical topology-based native_parent().
    const NativeParentMapping* native_mapping(std::size_t parent_index) const noexcept;
    // Use only with a nonnull section(); unresolved rows remain unadmitted.
    tl::fea::ShellSectionFormulation section_formulation(std::size_t part_index) const noexcept;
    const ResolutionCounts& counts() const noexcept;
    bool includes_glass() const noexcept;
    const std::vector<SectionPartResolution>& parts() const noexcept;
    const std::vector<SectionParentResolution>& parents() const noexcept;
    // Glass and midlayer Material expose physical/analytic coefficients and literal cards;
    // generic rate fields are unused. native_material() supplies its explicit
    // FilteredZeroC policy without fabricating original C/P/VP declarations.
    const assembly::Material* material(std::size_t part_index) const noexcept;
    const assembly::Section* section(std::size_t part_index) const noexcept;
    const tl::fea::ShellPlasticityMaterialInput* native_material(std::size_t part_index) const noexcept;
    // Artifact profiles use historical topology indices; the explicit compiled
    // profile uses native_mapping(). Neither admits a partial native collection.
    const tl::fea::ShellFailureParentInput* native_parent(std::size_t parent_index) const noexcept;
    const std::vector<assembly::Curve>& failure_curves() const noexcept;
    std::size_t startup_budget_bytes() const noexcept;
  private:
    struct Data;
    explicit VehicleSectionResolution(std::shared_ptr<const Data> data) : data_(std::move(data)) {}
    std::shared_ptr<const Data> data_;
};
} // namespace crash::modelio::vehicle
