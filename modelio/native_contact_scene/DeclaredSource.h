#pragma once
#include "lib_src/math/Fixed3.h"
#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <vector>
namespace crash::modelio::native_scene {
struct NativeMaterial {
    double density_tonne_mm3=0,young_n_mm2=0,poisson=0,yield_n_mm2=0,plastic_hardening_n_mm2=0;
    double rate_c_per_s=0,rate_p=0,rate_filter_hz=0;
};
struct SourceNode {std::uint64_t id=0;tl::math::Vec3 xyz_mm;};
struct SourceParent {
    std::uint64_t id=0,part=0;unsigned corners=0;
    std::array<std::uint32_t,4> nodes{}; // Zero-based original exported-node order; repeated T3 slot4.
};
enum class DeclaredContactSurface { FixedWall, AllShells };
struct DeclaredRigidPatch {
    std::uint64_t source_part_id=0,reference_body_id=0,reference_primary_id=0;
    tl::math::Vec3 reference_primary_mm;
    std::vector<std::uint64_t> member_source_ids,centroid_source_order;
};
struct DeclaredTiedPatch {
    std::uint32_t interface_id=0,master_surface_id=0,secondary_group_id=0;
    std::uint64_t master_parent_id=0,master_part_id=0,dependent_parent_id=0,dependent_part_id=0;
    std::array<std::uint32_t,4> master_nodes{},secondary_nodes{}; // Actual physical domain rows.
    // Raw card controls retained beside their source-resolved reader values.
    int ignore=0,spotflag=0,level=0,search=0,deletion=0,stiffness_mode=0,tied_removal=0;
    double search_distance_mm=0,stiffness_scale=0,viscosity=0;
    int resolved_level=0,resolved_search=0,resolved_hierarchy=0;
};
struct DeclaredData {
    DeclaredContactSurface contact_surface=DeclaredContactSurface::FixedWall;
    std::string export_sha256,definition_sha256,definition_bytes,definition_schema;
    NativeMaterial material;
    std::optional<DeclaredRigidPatch> rigid_patch;
    std::optional<DeclaredTiedPatch> tied_patch;
    std::vector<SourceNode> nodes;
    std::vector<SourceParent> wall,patch;
    std::vector<std::uint32_t> wall_nodes,patch_nodes;
    tl::math::Vec3 velocity_mm_s;
    double thickness_mm=0,end_time_s=0,nodal_scale=0,animation_interval_s=0;
    std::optional<double> step_cap_s;
};
struct ReadLimits {std::size_t file_bytes=4u<<20,nodes=4096,parents=4096;};
// Reads the existing create-only export of the sole JSON source compiler. The
// caller supplies expected export SHA. Files/roster/unit consistency are checked;
// this does not certify an arbitrary external compiler or admit numerical physics.
class DeclaredSource {
  public:
    static DeclaredSource Read(const std::filesystem::path& exported_manifest,const std::string& expected_sha,ReadLimits={});
    const DeclaredData& data() const noexcept{return *data_;}
  private:
    explicit DeclaredSource(std::shared_ptr<const DeclaredData> data):data_(std::move(data)){}
    std::shared_ptr<const DeclaredData> data_;
};
} // namespace crash::modelio::native_scene
