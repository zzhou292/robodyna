#pragma once
#include "Config.h"
#include <array>
#include <memory>
namespace crash::cases::source_assembly_dynamics {
inline constexpr double NativeRotationQualifiedBound=1.5;
inline constexpr const char* NativeRotationPolicy="native_shell_geometry_v1";
inline constexpr const char* NativeRotationQualification="70e7f7bf738eb087053816cc908da35a6f81399a";
struct NativeRotationReference {
    tl::math::Matrix3 frame;
    std::array<tl::math::Vec3,4> world_normals{};
    std::uint64_t source_parent=0;
    std::size_t arity=0;
};
inline constexpr tl::math::Vec3 NativeTriangleNormals[3]{{0,0,1},{0,0,1},{0,0,1}};
struct NativeRotationMeasure { double frame=0,nodal_normal=0; };
// Immutable observation reference only. Native startup reference evaluation
// supplies its geometry; it owns no shell mass, history, nodal state or clock.
class NativeRotationReferences {
  public:
    std::size_t parent_count() const noexcept{return parent_count_;}
    std::size_t node_count() const noexcept{return node_count_;}
    std::uint64_t source_instance() const noexcept{return source_instance_;}
    const NativeRotationReference& parent(std::size_t i) const noexcept{return parents_[i];}
    bool grouped(std::size_t i) const noexcept{return grouped_[i]!=0;}
  private:
    NativeRotationReferences(std::size_t,std::size_t,std::uint64_t);
    std::size_t parent_count_=0,node_count_=0;std::uint64_t source_instance_=0;
    std::unique_ptr<NativeRotationReference[]> parents_;
    std::unique_ptr<std::uint8_t[]> grouped_;
    friend Report PrepareNativeRotation(const source_assembly::SourceAssemblyBindings&,double,
                                         std::unique_ptr<const NativeRotationReferences>&);
};
Report PrepareNativeRotation(const source_assembly::SourceAssemblyBindings&,double fixed_dt,
                             std::unique_ptr<const NativeRotationReferences>&);
// Source frame and actual current force-stage frame/world nodal normals, all
// endpoint geometry. No total-q director assumption. Failure preserves output.
Report MeasureNativeRotation(const NativeRotationReference&,const tl::math::Matrix3& current,
    const tl::math::Vec3* local_normals,std::size_t arity,double limit,NativeRotationMeasure&) noexcept;
}
