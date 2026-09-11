#include "ShellCollectionContactGeometryInternal.h"
#include "lib_src/collision/Q4ParametricContact.h"
#include <algorithm>
#include <new>
#include <vector>

namespace crash::cases {
namespace sc=tlfea::contact;
namespace fe=tl::fea;
using Code=ShellContactGeometryStatus;
ShellCollectionContactGeometry::ShellCollectionContactGeometry()=default;
ShellCollectionContactGeometry::~ShellCollectionContactGeometry()=default;
ShellContactGeometryReport ShellCollectionContactGeometry::Initialize(
    const fe::ShellBatchBinding& binding,const ShellContactGeometryLimits& limits) {
    return InitializeImpl(binding,nullptr,limits);
}
ShellContactGeometryReport ShellCollectionContactGeometry::InitializeMapped(
    const fe::ShellNodeMap& mapping,const ShellContactGeometryLimits& limits) {
    if(!mapping.prepared())return {Code::InvalidInput,"A prepared exact shell-to-domain map is required"};
    return InitializeImpl(*mapping.shells(),&mapping,limits);
}
ShellContactGeometryReport ShellCollectionContactGeometry::InitializeImpl(
    const fe::ShellBatchBinding& binding,const fe::ShellNodeMap* mapping,const ShellContactGeometryLimits& limits) {
    if(impl_||!binding.prepared()) return {Code::InvalidInput,"Fresh geometry and a complete native binding are required"};
    const auto q=binding.qeph_count(),t=binding.t3_count(),b=binding.qbat_count(),p=q+t+b;
    const auto n=mapping?mapping->owner_node_count():binding.node_count();
    const bool vehicle=limits.weights.profile==sc::NodalWallWeightProfile::Vehicle;
    if(!vehicle&&limits.weights.profile!=sc::NodalWallWeightProfile::Legacy)
        return {Code::InvalidInput,"Unknown shell contact weight profile"};
    const auto maximum_nodes=vehicle?sc::MaxVehicleWallWeightNodes:sc::MaxNodalWallWeightNodes;
    const auto maximum_parents=vehicle?sc::MaxVehicleWallWeightParents:sc::MaxNodalWallWeightParents;
    const std::size_t maximum_startup=vehicle?ShellContactGeometryLimits::Vehicle().max_startup_bytes:32*1024*1024;
    if(!n||n>maximum_nodes||!p||p>maximum_parents||
       n>limits.weights.max_nodes||p>limits.weights.max_parents||
       (vehicle&&(limits.weights.max_nodes>maximum_nodes||limits.weights.max_parents>maximum_parents))||
       !limits.max_startup_bytes||limits.max_startup_bytes>maximum_startup)
        return {Code::ResourceLimit,"Shell contact startup exceeds explicit collection limits"};
    // Counts passed profile hard bounds, so this sum cannot overflow. Keep the
    // conservative legacy ledger; Vehicle also reserves transient weight indexes.
    // Charge the complete allowed weights payload, including its inline arrays.
    const auto maximum_weights=vehicle?sc::MaxVehicleWallWeightOwnedBytes:4*1024*1024;
    if(!limits.weights.max_owned_bytes||limits.weights.max_owned_bytes>maximum_weights||
       (vehicle&&(!limits.weights.max_startup_bytes||limits.weights.max_startup_bytes>sc::MaxVehicleWallWeightScratchBytes)))
        return {Code::ResourceLimit,"Invalid immutable contact weight payload budget"};
    const auto payload=sizeof(Impl)+(mapping?mapping->owned_payload_bytes():binding.host_bytes())+3*n*sizeof(double)+p*sizeof(ShellContactParent)+
        (q+b)*sizeof(sc::Q4ParametricReference)+t*sizeof(sc::T3MaterialMeasure)+
        p*sizeof(sc::NodalWallParentInput)+limits.weights.max_owned_bytes+
        (vehicle?limits.weights.max_startup_bytes:0);
    if(payload>limits.max_startup_bytes)
        return {Code::ResourceLimit,"Shell contact peak startup payload exceeds its byte budget"};
    try {
        auto next=std::make_unique<Impl>(binding,mapping);next->startup_bytes=payload;
        next->PrepareCoordinates();
        const auto report=next->PrepareReferences(limits);
        if(!report)return report;
        impl_=std::move(next);return {Code::Ok,"Complete shell contact reference prepared"};
    } catch(const std::bad_alloc&) { return {Code::ResourceLimit,"Shell contact startup allocation failed"}; }
}
bool ShellCollectionContactGeometry::prepared() const noexcept { return bool(impl_); }
const fe::ShellBatchBinding* ShellCollectionContactGeometry::binding() const noexcept { return impl_?&impl_->binding:nullptr; }
const fe::ShellNodeMap* ShellCollectionContactGeometry::mapping() const noexcept {
    return impl_&&impl_->mapping?&*impl_->mapping:nullptr;
}
const sc::NodalWallWeights* ShellCollectionContactGeometry::weights() const noexcept { return impl_?&impl_->weights:nullptr; }
sc::VectorView ShellCollectionContactGeometry::positions() const noexcept {
    return impl_?sc::VectorView{impl_->coordinates.data(),static_cast<std::uint32_t>(impl_->node_count()),3,1}:sc::VectorView{};
}
std::array<sc::Vec3,2> ShellCollectionContactGeometry::reference_bounds() const noexcept {
    return impl_?impl_->bounds:std::array<sc::Vec3,2>{};
}
sc::PlanarContactReport ShellCollectionContactGeometry::CheckWallCoverage(const sc::PlanarWallGeometry& wall,
    sc::PlanarWallBox motion,double clearance,std::uint64_t id,sc::PlanarWallBoxCoverage* output) const {
    using Status=sc::PlanarContactStatus;
    if(!impl_||!wall.initialized())return {Status::NotInitialized,"Shell collection or finite wall is not prepared"};
    const auto lo=motion.minimum,hi=motion.maximum;
    if(!output||!id||!sc::IsFinite(lo)||!sc::IsFinite(hi)||
       lo.x>impl_->bounds[0].x||lo.y>impl_->bounds[0].y||lo.z>impl_->bounds[0].z||
       hi.x<impl_->bounds[1].x||hi.y<impl_->bounds[1].y||hi.z<impl_->bounds[1].z)
        return {Status::InvalidInput,"Motion envelope must contain the complete native shell collection"};
    const double x=wall.wall_x();
    return sc::CheckPlanarWallBox(wall,{{x,lo.y,lo.z},{x,hi.y,hi.z}},clearance,id,
        sc::PlanarWallBoxMode::ConservativeExpansion,output);
}
const ShellContactParent* ShellCollectionContactGeometry::parent_from_weight(std::size_t i) const noexcept {
    return impl_&&i<impl_->parents.size()?&impl_->parents[i]:nullptr;
}
std::size_t ShellCollectionContactGeometry::startup_payload_bytes() const noexcept { return impl_?impl_->startup_bytes:0; }
} // namespace crash::cases
