#pragma once
#include "modelio/source_assembly/SourceAssemblyData.h"
#include "output/full_shell/static_bundle/Types.h"
#include <memory>

namespace crash::modelio::vehicle {
namespace source = output::full_shell::source;
inline constexpr const char* DeclarationSchema="robo-dyna.vehicle-source-declarations.v1";
struct Limits {
    std::size_t declaration_bytes=8*1024*1024, host_bytes=512*1024*1024;
    std::size_t parents=524288, nodes=524288, parts=1024, tables=1024, curve_points=1024;
};
enum class Disposition { SupportedDeclaration, Unresolved };
struct Obligation { std::string stage, reason; };
struct PartDisposition {
    std::uint64_t part_id=0, material_id=0, section_id=0;
    std::size_t shell_count=0, typed_index=SIZE_MAX;
    Disposition status=Disposition::Unresolved;
    std::vector<Obligation> obligations;
    // Original PART/SECTION/MATERIAL blocks, in that order, for unresolved roles.
    std::array<assembly::SourceBlock,3> unresolved_sources;
};
struct ParentIndex { std::uint32_t canonical_parent=0, part_index=0; };
struct Counts {
    std::size_t parts=0, parents=0, nodes=0, q4=0, t3=0;
    std::size_t supported_parts=0, supported_parents=0;
};
// Declaration-only source readiness. No SourceAssembly, M/J, force, solver,
// accepted stamp or native-family authorization is constructed. Complete source
// arrays/positions remain in the shared immutable CanonicalSource backing.
class VehicleSourcePlan {
  public:
    static VehicleSourcePlan Read(const source::CanonicalSource&,const std::filesystem::path&,
                                  const assembly::ArtifactIdentity&,Limits={});
    static VehicleSourcePlan ReadBytes(const source::CanonicalSource&,const std::string&,
                                       const assembly::ArtifactIdentity&,Limits={});
    VehicleSourcePlan(const VehicleSourcePlan&) noexcept=default;
    VehicleSourcePlan(VehicleSourcePlan&& other) noexcept:data_(other.data_) {}
    VehicleSourcePlan& operator=(const VehicleSourcePlan&)=delete;
    VehicleSourcePlan& operator=(VehicleSourcePlan&&)=delete;
    const source::CanonicalSource& canonical() const noexcept;
    const assembly::ArtifactIdentity& identity() const noexcept;
    const Counts& counts() const noexcept;
    const std::vector<PartDisposition>& parts() const noexcept;
    // Parent order is canonical source record order; nodes are ascending NID.
    const std::vector<ParentIndex>& parents() const noexcept;
    const std::vector<std::uint32_t>& canonical_nodes() const noexcept;
    // Null for an unresolved part or invalid index. No fabricated elastic law.
    const assembly::Material* material(std::size_t part_index) const noexcept;
    const assembly::Section* section(std::size_t part_index) const noexcept;
    const std::vector<assembly::Curve>& curves() const noexcept;
    std::size_t startup_budget_bytes() const noexcept;
  private:
    struct Data;
    explicit VehicleSourcePlan(std::shared_ptr<const Data> data):data_(std::move(data)) {}
    std::shared_ptr<const Data> data_;
};
} // namespace crash::modelio::vehicle
