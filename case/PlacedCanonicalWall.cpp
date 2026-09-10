#include "PlacedCanonicalWall.h"
#include "CanonicalWallArtifacts.h"
#include <cmath>
#include <new>

namespace crash::case_data {
namespace contact=tlfea::contact;
struct PlacedCanonicalWall::Data {
    WallTessellation source;
    std::vector<contact::PlanarWallVertex> vertices;
    std::vector<contact::PlanarWallTriangle> triangles;
    contact::PlanarWallGeometry geometry;
    WallPlacement placement;
    std::string manifest;
    contact::PlanarWallView view() const noexcept {
        return {vertices.data(),static_cast<std::uint32_t>(vertices.size()),
                triangles.data(),static_cast<std::uint32_t>(triangles.size())};
    }
};
PlacedCanonicalWall::PlacedCanonicalWall()=default;
PlacedCanonicalWall::~PlacedCanonicalWall()=default;
bool PlacedCanonicalWall::initialized() const noexcept {return bool(data_);}
contact::PlanarWallView PlacedCanonicalWall::view() const noexcept {return data_?data_->view():contact::PlanarWallView{};}
const contact::PlanarWallGeometry* PlacedCanonicalWall::geometry() const noexcept {return data_?&data_->geometry:nullptr;}
const WallPlacement* PlacedCanonicalWall::placement() const noexcept {return data_?&data_->placement:nullptr;}
const WallProvenance* PlacedCanonicalWall::source_provenance() const noexcept {return data_?data_->source.source_provenance():nullptr;}
const std::string* PlacedCanonicalWall::source_manifest() const noexcept {return data_?&data_->manifest:nullptr;}
PlacedWallReport PlacedCanonicalWall::Initialize(const CanonicalWall& canonical,const std::string& bytes,double shift) {
    if(data_)return {PlacedWallStatus::AlreadyInitialized,"Placed wall is immutable after publication"};
    if(!canonical.loaded()||canonical.vertices().size()!=62||canonical.triangles().size()!=100||
       canonical.source_quads().size()!=46||!std::isfinite(shift))
        return {PlacedWallStatus::InvalidInput,"Placement requires the complete loaded original wall and finite X translation"};
    try {
        auto next=std::make_unique<Data>();
        // Reuse the authenticated original source copy and validation. This
        // branch performs no tessellation or synthetic feature construction.
        const auto prepared=next->source.Initialize(canonical,bytes,WallTessellationKind::Original);
        if(prepared.status!=WallTessellationStatus::Ok)
            return {PlacedWallStatus::InvalidInput,prepared.diagnostic};
        const auto original=next->source.view();
        next->vertices.assign(original.vertices,original.vertices+original.vertex_count);
        next->triangles.assign(original.triangles,original.triangles+original.triangle_count);
        const double x=original.vertices[0].position.x+shift;
        if(!std::isfinite(x))return {PlacedWallStatus::InvalidInput,"Placed wall X is not representable"};
        for(auto& vertex:next->vertices)vertex.position.x=x;
        const auto checked=next->geometry.Initialize(next->view());
        if(checked.status!=contact::PlanarContactStatus::Ok)
            return {PlacedWallStatus::GeometryFailure,checked.message};
        next->placement={next->source.metadata()->source_manifest_sha256,next->source.metadata()->mesh_sha256,shift,x};
        next->manifest=bytes;
        data_=std::move(next);return {PlacedWallStatus::Ok,"Complete original wall placed by explicit X translation"};
    } catch(const std::bad_alloc&) {
        return {PlacedWallStatus::ResourceLimit,"Placed wall bounded allocation failed"};
    }
}
} // namespace crash::case_data
