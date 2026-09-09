#include "chrono/core/ChMatrix.h"
#include "WallTessellation.h"
#include "WallTessellationGeometry.h"
#include "CanonicalWallArtifacts.h"
#include "chrono/GuidedPlateModel.h"
#include "collision/Q4ContactIntegration.h"
#include "lib_utest/q4_planar_geometry_fixture.h"
#include "output/ArtifactIO.h"
#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <iostream>

namespace {
using namespace crash::case_data;
namespace ct=tlfea::contact;
namespace ref=crash::reference;
namespace out=crash::output;
namespace detail=crash::case_data::wall_tessellation_detail;
using Kind=WallTessellationKind;
using Code=WallTessellationStatus;
std::string asset;
class WallTessellationCheck:public ::testing::Test {
  protected:
    CanonicalWall wall;std::string bytes;
    void SetUp() override {
        ASSERT_NO_THROW(bytes=ReadPinnedWallManifest(asset));
        std::istringstream input(bytes);const auto r=wall.Load(input);ASSERT_EQ(r.status,WallStatus::Ok)<<r.message;
    }
};
void SamePoint(ct::Vec3 a,ct::Vec3 b) {
    EXPECT_EQ(out::Bits(a.x),out::Bits(b.x));EXPECT_EQ(out::Bits(a.y),out::Bits(b.y));EXPECT_EQ(out::Bits(a.z),out::Bits(b.z));
}
void SameMesh(ct::PlanarWallView a,ct::PlanarWallView b) {
    ASSERT_EQ(a.vertex_count,b.vertex_count);ASSERT_EQ(a.triangle_count,b.triangle_count);
    for(unsigned i=0;i<a.vertex_count;++i) {
        SamePoint(a.vertices[i].position,b.vertices[i].position);
        EXPECT_EQ(a.vertices[i].source_node_id,b.vertices[i].source_node_id);EXPECT_EQ(a.vertices[i].assembled_source_node_id,b.vertices[i].assembled_source_node_id);
    }
    for(unsigned i=0;i<a.triangle_count;++i) {
        const auto& p=a.triangles[i];const auto& q=b.triangles[i];
        for(unsigned n=0;n<3;++n)EXPECT_EQ(p.nodes[n],q.nodes[n]);
        EXPECT_EQ(p.triangle_id,q.triangle_id);EXPECT_EQ(p.source_quad_id,q.source_quad_id);EXPECT_EQ(p.assembled_source_quad_id,q.assembled_source_quad_id);
    }
}
std::map<detail::Edge,unsigned> IndependentEdges(ct::PlanarWallView view) {
    std::map<detail::Edge,unsigned> result;
    for(unsigned t=0;t<view.triangle_count;++t)for(unsigned n=0;n<3;++n) {
        auto a=view.triangles[t].nodes[n],b=view.triangles[t].nodes[(n+1)%3];if(a>b)std::swap(a,b);++result[{a,b}];
    }
    return result;
}
std::set<detail::Edge> IndependentBoundary(ct::PlanarWallView view) {
    std::set<detail::Edge> result;for(const auto& e:IndependentEdges(view))if(e.second==1)result.insert(e.first);return result;
}
struct PhysicalPatch {
    std::array<double,3*ref::kCouponNodes> x{},v{};
    const ref::GuidedPlateModel& model;
    explicit PhysicalPatch(const ref::GuidedPlateModel& m):model(m) {
        for(unsigned n=0;n<ref::kCouponNodes;++n) {
            const auto p=m.shell().data().reference_configuration.position[n];x[3*n]=p.x;x[3*n+1]=p.y;x[3*n+2]=p.z;
        }
    }
    ct::Q4SurfaceView view() const {return {{x.data(),ref::kCouponNodes,3,1},{v.data(),ref::kCouponNodes,3,1},model.data().parents.data(),ref::kCouponElements};}
    ct::Q4FixedYZMassView mass() const {return {model.shell().data().inverse_mass.data(),model.data().translation_fixed_bits.data(),ref::kCouponNodes,0};}
};
void SameReference(ct::Q4PlanarReferenceView a,ct::Q4PlanarReferenceView b) {
    ASSERT_EQ(a.parent_count,b.parent_count);EXPECT_EQ(out::Bits(a.wall_x),out::Bits(b.wall_x));
    for(unsigned p=0;p<a.parent_count;++p) {
        const auto& x=a.parents[p];const auto& y=b.parents[p];EXPECT_EQ(x.covered,y.covered);
        EXPECT_EQ(out::Bits(x.projected_area),out::Bits(y.projected_area));
        EXPECT_EQ(out::Bits(x.area_enclosure.lower),out::Bits(y.area_enclosure.lower));EXPECT_EQ(out::Bits(x.area_enclosure.upper),out::Bits(y.area_enclosure.upper));
        EXPECT_EQ(x.parent.feature_id,y.parent.feature_id);EXPECT_EQ(x.parent.parent_element_id,y.parent.parent_element_id);
        for(unsigned n=0;n<4;++n) {EXPECT_EQ(x.parent.nodes[n],y.parent.nodes[n]);SamePoint(x.reference_projection[n],y.reference_projection[n]);}
    }
}

TEST_F(WallTessellationCheck, OriginalPreservesActualSourceAndDeterministicIdentity) {
    WallTessellation a,b;auto r=a.Initialize(wall,bytes,Kind::Original);ASSERT_EQ(r.status,Code::Ok)<<r.diagnostic;
    r=b.Initialize(wall,bytes,Kind::Original);ASSERT_EQ(r.status,Code::Ok)<<r.diagnostic;
    SameMesh(a.view(),b.view());ASSERT_TRUE(a.metadata());ASSERT_TRUE(a.geometry());EXPECT_TRUE(a.geometry()->initialized());
    EXPECT_EQ(a.metadata()->mesh_sha256,b.metadata()->mesh_sha256);EXPECT_EQ(a.metadata()->source_manifest_sha256,kCanonicalWallManifestSha256);
    EXPECT_EQ(a.view().vertex_count,62u);EXPECT_EQ(a.view().triangle_count,100u);EXPECT_TRUE(a.metadata()->midpoints.empty());
    EXPECT_TRUE(a.metadata()->flipped_source_quads.empty());EXPECT_EQ(a.metadata()->original_boundary_edges,22u);
    EXPECT_EQ(a.metadata()->derived_boundary_edges,22u);EXPECT_TRUE(a.metadata()->exposed_boundary_exact);
    EXPECT_EQ(a.source_provenance()->wall_sha256,wall.provenance().wall_sha256);
    EXPECT_EQ(a.source_provenance()->model_archive_reference_sha256,wall.provenance().model_archive_reference_sha256);
    for(unsigned i=0;i<62;++i) {
        const auto& v=wall.vertices()[i];SamePoint(a.view().vertices[i].position,{v.position_m[0],v.position_m[1],v.position_m[2]});
        EXPECT_EQ(a.view().vertices[i].source_node_id,v.source_node_id);
    }
    for(unsigned i=0;i<100;++i) {
        const auto& original=wall.triangles()[i];const auto& actual=a.view().triangles[i];
        EXPECT_EQ(actual.triangle_id,original.triangle_id);EXPECT_EQ(a.metadata()->faces[i].original_triangle_ids[0],original.triangle_id);
        EXPECT_EQ(a.metadata()->faces[i].original_triangle_count,1u);EXPECT_FALSE(a.metadata()->faces[i].connectivity_changed);
        for(unsigned n=0;n<3;++n)EXPECT_EQ(actual.nodes[n],original.vertex_indices[n]);
    }
}

TEST_F(WallTessellationCheck, OppositeDiagonalsChangeFortyFiveConvexPairsAndRetainStitchedGroup) {
    WallTessellation original,flipped,repeated;auto r=original.Initialize(wall,bytes,Kind::Original);ASSERT_EQ(r.status,Code::Ok)<<r.diagnostic;
    r=flipped.Initialize(wall,bytes,Kind::FlipConvexPairs);ASSERT_EQ(r.status,Code::Ok)<<r.diagnostic;
    r=repeated.Initialize(wall,bytes,Kind::FlipConvexPairs);ASSERT_EQ(r.status,Code::Ok)<<r.diagnostic;
    SameMesh(flipped.view(),repeated.view());EXPECT_EQ(flipped.metadata()->mesh_sha256,repeated.metadata()->mesh_sha256);
    EXPECT_NE(original.metadata()->mesh_sha256,flipped.metadata()->mesh_sha256);EXPECT_EQ(flipped.metadata()->flipped_source_quads.size(),45u);
    EXPECT_EQ(IndependentBoundary(original.view()),IndependentBoundary(flipped.view()));EXPECT_TRUE(flipped.metadata()->exposed_boundary_exact);
    EXPECT_TRUE(flipped.metadata()->midpoints.empty());EXPECT_EQ(flipped.view().vertex_count,62u);EXPECT_EQ(flipped.view().triangle_count,100u);
    unsigned changed=0,stitched=0;
    for(unsigned t=0;t<100;++t) {
        const auto& a=original.view().triangles[t];const auto& b=flipped.view().triangles[t];const auto& lineage=flipped.metadata()->faces[t];
        EXPECT_EQ(a.source_quad_id,b.source_quad_id);EXPECT_EQ(a.assembled_source_quad_id,b.assembled_source_quad_id);
        if(a.source_quad_id==1046) {
            ++stitched;EXPECT_EQ(a.triangle_id,b.triangle_id);EXPECT_FALSE(lineage.connectivity_changed);
            for(unsigned n=0;n<3;++n)EXPECT_EQ(a.nodes[n],b.nodes[n]);
        } else {
            ++changed;EXPECT_GE(b.triangle_id,kWallDerivedFaceIdBase);EXPECT_TRUE(lineage.connectivity_changed);
            EXPECT_EQ(lineage.original_triangle_count,2u);
            std::set<unsigned> before(std::begin(a.nodes),std::end(a.nodes)),after(std::begin(b.nodes),std::end(b.nodes));EXPECT_NE(before,after);
        }
    }
    EXPECT_EQ(changed,90u);EXPECT_EQ(stitched,10u);
}

TEST_F(WallTessellationCheck, UniformSubdivisionSharesMidpointsAndBoundsRoundedExposedEdges) {
    WallTessellation original,split;auto r=original.Initialize(wall,bytes,Kind::Original);ASSERT_EQ(r.status,Code::Ok)<<r.diagnostic;
    r=split.Initialize(wall,bytes,Kind::UniformFour);ASSERT_EQ(r.status,Code::Ok)<<r.diagnostic;
    const auto a=original.view(),b=split.view();const auto& meta=*split.metadata();
    ASSERT_EQ(b.vertex_count,223u);ASSERT_EQ(b.triangle_count,400u);ASSERT_EQ(meta.midpoints.size(),161u);ASSERT_EQ(meta.faces.size(),400u);
    EXPECT_EQ(meta.derived_boundary_edges,44u);EXPECT_FALSE(meta.exposed_boundary_exact);EXPECT_GT(meta.exposed_boundary_displacement_bound_m,0);
    EXPECT_LE(meta.exposed_boundary_displacement_bound_m,original.geometry()->tolerance());
    std::set<std::uint64_t> ids;std::set<detail::Edge> source_edges,expected_boundary;
    const auto edges=IndependentEdges(a);
    // x86 extended precision exactly represents sums of these bounded source
    // coordinate significands. This oracle is independent of TL interval ops.
    static_assert(std::numeric_limits<long double>::digits>=64);
    for(const auto& m:meta.midpoints) {
        ASSERT_LT(m.vertex,b.vertex_count);const auto edge=detail::Key(m.original_edge[0],m.original_edge[1]);
        EXPECT_TRUE(source_edges.insert(edge).second);ASSERT_TRUE(edges.count(edge));EXPECT_EQ(m.exposed_edge,edges.at(edge)==1);
        const auto& p=b.vertices[m.vertex];const auto& u=a.vertices[edge.first];const auto& v=a.vertices[edge.second];
        EXPECT_GE(p.source_node_id,kWallMidpointIdBase);EXPECT_LT(p.source_node_id,kWallDerivedFaceIdBase);
        EXPECT_EQ(p.source_node_id,p.assembled_source_node_id);EXPECT_TRUE(ids.insert(p.source_node_id).second);
        EXPECT_EQ(m.original_source_nodes[0],u.source_node_id);EXPECT_EQ(m.original_source_nodes[1],v.source_node_id);
        EXPECT_EQ(m.original_assembled_nodes[0],u.assembled_source_node_id);EXPECT_EQ(m.original_assembled_nodes[1],v.assembled_source_node_id);
        EXPECT_EQ(out::Bits(p.position.x),out::Bits(a.vertices[0].position.x));EXPECT_EQ(m.coordinate_error.x,0);
        const long double y=(static_cast<long double>(u.position.y)+v.position.y)*.5L,z=(static_cast<long double>(u.position.z)+v.position.z)*.5L;
        EXPECT_LE(static_cast<long double>(m.exact_midpoint_y.lower),y);EXPECT_GE(static_cast<long double>(m.exact_midpoint_y.upper),y);
        EXPECT_LE(static_cast<long double>(m.exact_midpoint_z.lower),z);EXPECT_GE(static_cast<long double>(m.exact_midpoint_z.upper),z);
        EXPECT_LE(std::abs(static_cast<long double>(p.position.y)-y),static_cast<long double>(m.coordinate_error.y));
        EXPECT_LE(std::abs(static_cast<long double>(p.position.z)-z),static_cast<long double>(m.coordinate_error.z));
        if(m.exposed_edge) {
            expected_boundary.insert(detail::Key(edge.first,m.vertex));expected_boundary.insert(detail::Key(m.vertex,edge.second));
            if(u.position.y==v.position.y)EXPECT_EQ(p.position.y,u.position.y);
            if(u.position.z==v.position.z)EXPECT_EQ(p.position.z,u.position.z);
        }
    }
    EXPECT_EQ(expected_boundary,IndependentBoundary(b));EXPECT_EQ(source_edges.size(),edges.size());
    for(unsigned t=0;t<400;++t) {
        const auto& parent=a.triangles[t/4];const auto& child=b.triangles[t];const auto& lineage=meta.faces[t];
        EXPECT_GE(child.triangle_id,kWallDerivedFaceIdBase);EXPECT_EQ(child.source_quad_id,parent.source_quad_id);
        EXPECT_EQ(child.assembled_source_quad_id,parent.assembled_source_quad_id);EXPECT_EQ(lineage.original_triangle_count,1u);
        EXPECT_EQ(lineage.original_triangle_ids[0],parent.triangle_id);EXPECT_EQ(lineage.subtriangle,t%4);
    }
}

struct ContactSummary {
    std::array<ct::Q4IntegrationResult,2> parent;
    ct::Vec3 force,moment;double power=0,potential=0,area=0;
};
ContactSummary Contact(const PhysicalPatch& patch,const ct::Q4PlanarGeometry& geometry) {
    ContactSummary result;q4_planar_test::Scratch scratch;
    for(unsigned p=0;p<2;++p) {
        ct::Q4PreparedIntegration prepared;
        const auto ready=ct::PrepareQ4PlanarIntegration(geometry.view(),patch.view(),patch.mass(),p,
            ref::GuidedPlateData::stiffness_per_area,ref::GuidedPlateData::maximum_penetration,1,&prepared);
        out::Require(ready==ct::PlanarContactStatus::Ok&&prepared.covered,"Prescribed test contact preparation failed");
        const auto integrated=ct::IntegrateQ4NormalContact(prepared.input,patch.model.data().integration,scratch.view(),&result.parent[p]);
        out::Require(integrated.status==ct::Q4IntegrationStatus::Ok,"Prescribed test contact integration failed");
        const auto& r=result.parent[p];result.potential+=r.potential.value;result.area+=r.active_area.upper;
        for(unsigned n=0;n<4;++n) {
            const auto i=r.nodal.nodes[n];const ct::Vec3 x{patch.x[3*i],patch.x[3*i+1],patch.x[3*i+2]},v{patch.v[3*i],patch.v[3*i+1],patch.v[3*i+2]};
            result.force=ct::Add(result.force,r.nodal.forces[n]);result.moment=ct::Add(result.moment,ct::geometry_detail::Cross(x,r.nodal.forces[n]));
            result.power+=ct::Dot(r.nodal.forces[n],v);
        }
    }
    return result;
}
TEST_F(WallTessellationCheck, OriginalFootprintAndIdenticalPrescribedContactRemainInvariant) {
    WallTessellation original,flip,split;
    for(auto entry:{std::make_pair(&original,Kind::Original),std::make_pair(&flip,Kind::FlipConvexPairs),std::make_pair(&split,Kind::UniformFour)}) {
        const auto r=entry.first->Initialize(wall,bytes,entry.second);ASSERT_EQ(r.status,Code::Ok)<<r.diagnostic;
    }
    ref::GuidedPlateModel model(original.view(),0x57414c4c54455354ULL);PhysicalPatch patch(model);
    std::array<ct::Q4PlanarGeometry,3> geometry;
    std::array<const WallTessellation*,3> variants{{&original,&flip,&split}};
    for(unsigned i=0;i<3;++i) {
        const auto r=variants[i]->CheckCoverage(patch.view(),patch.mass(),ref::GuidedPlateData::exposed_clearance,geometry[i]);
        ASSERT_EQ(r.status,Code::Ok)<<r.diagnostic;SameReference(geometry[0].view(),geometry[i].view());
    }
    bool partial=false,nonuniform=false;
    for(unsigned state=0;state<3;++state) {
        PhysicalPatch current(model);const double wall_x=original.geometry()->wall_x();
        for(unsigned n:{0u,3u,4u,5u}) {
            const double gap=state==0?-.0002:state==1?(n<4?-.0001:(n==4?.0002:.0001)):(n<4?.0001:.0002);
            current.x[3*n]=wall_x+gap;current.v[3*n]=.01*(n+1);
        }
        const auto expected=Contact(current,geometry[0]);
        if(state==0)EXPECT_EQ(expected.potential,0);
        else {EXPECT_GT(expected.potential,0);EXPECT_LT(expected.force.x,0);EXPECT_NE(expected.power,0);}
        for(unsigned i=1;i<3;++i) {
            const auto actual=Contact(current,geometry[i]);
            SamePoint(actual.force,expected.force);SamePoint(actual.moment,expected.moment);
            EXPECT_EQ(actual.power,expected.power);EXPECT_EQ(actual.potential,expected.potential);
            for(unsigned p=0;p<2;++p) {
                const auto& a=actual.parent[p];const auto& e=expected.parent[p];
                EXPECT_LE(std::abs(a.potential.value-e.potential.value),a.potential.error+e.potential.error);
                EXPECT_LE(std::abs(a.resultant.value-e.resultant.value),a.resultant.error+e.resultant.error);
                EXPECT_EQ(a.potential.lower,e.potential.lower);EXPECT_EQ(a.potential.upper,e.potential.upper);
                for(unsigned n=0;n<4;++n)EXPECT_LE(std::abs(a.force[n].value-e.force[n].value),a.force[n].error+e.force[n].error);
                partial|=a.active_area.lower>0&&a.active_area.upper<geometry[i].view().parents[p].area_enclosure.lower;
                nonuniform|=std::abs(a.force[0].value-a.force[1].value)>a.force[0].error+a.force[1].error;
            }
        }
    }
    EXPECT_TRUE(partial);EXPECT_TRUE(nonuniform);
}

TEST_F(WallTessellationCheck, FailedInitializationAndCoveragePreservePublishedOutputs) {
    WallTessellation mesh;EXPECT_FALSE(mesh.initialized());EXPECT_EQ(mesh.metadata(),nullptr);
    auto r=mesh.Initialize(wall,bytes+" ",Kind::Original);EXPECT_EQ(r.status,Code::InvalidInput);EXPECT_FALSE(mesh.initialized());
    r=mesh.Initialize(wall,bytes,static_cast<Kind>(255));EXPECT_EQ(r.status,Code::InvalidInput);EXPECT_FALSE(mesh.initialized());
    r=mesh.Initialize(wall,bytes,Kind::UniformFour);ASSERT_EQ(r.status,Code::Ok)<<r.diagnostic;
    const auto* identity=mesh.metadata();const auto hash=identity->mesh_sha256;const auto* vertices=mesh.view().vertices;
    r=mesh.Initialize(wall,bytes,Kind::Original);EXPECT_EQ(r.status,Code::AlreadyInitialized);EXPECT_EQ(mesh.metadata(),identity);
    EXPECT_EQ(mesh.metadata()->mesh_sha256,hash);EXPECT_EQ(mesh.view().vertices,vertices);
    ref::GuidedPlateModel model(mesh.view(),123);PhysicalPatch patch(model);ct::Q4PlanarGeometry output;
    r=mesh.CheckCoverage(patch.view(),patch.mass(),ref::GuidedPlateData::exposed_clearance,output);ASSERT_EQ(r.status,Code::Ok)<<r.diagnostic;
    const auto before=output;
    for(unsigned n=0;n<ref::kCouponNodes;++n)patch.x[3*n+1]+=10;
    r=mesh.CheckCoverage(patch.view(),patch.mass(),ref::GuidedPlateData::exposed_clearance,output);
    EXPECT_EQ(r.status,Code::GeometryFailure);SameReference(output.view(),before.view());
    WallTessellation empty;r=empty.CheckCoverage(patch.view(),patch.mass(),ref::GuidedPlateData::exposed_clearance,output);
    EXPECT_EQ(r.status,Code::NotInitialized);SameReference(output.view(),before.view());
}

TEST(WallTessellationGeometry, ConvexCycleRejectsConcavityAndUnresolvedTurnsWithoutPublishing) {
    auto square=q4_planar_test::Square();std::array<std::uint32_t,4> cycle{{99,98,97,96}};
    ct::PlanarWallGeometry geometry;ASSERT_EQ(geometry.Initialize(square.view()).status,ct::PlanarContactStatus::Ok);
    ASSERT_TRUE(detail::ConvexCycle(square.view(),0,1,geometry.tolerance(),cycle));
    const auto saved=cycle;
    // Move one corner into the other triangle; only the internal predicate is
    // exercised here, not an authenticated canonical variant.
    square.vertices[2].position={0,-.2,.2};
    EXPECT_FALSE(detail::ConvexCycle(square.view(),0,1,geometry.tolerance(),cycle));EXPECT_EQ(cycle,saved);
    square=q4_planar_test::Square();square.vertices[2].position=square.vertices[1].position;
    EXPECT_FALSE(detail::ConvexCycle(square.view(),0,1,geometry.tolerance(),cycle));EXPECT_EQ(cycle,saved);
}
} // namespace
int main(int argc,char** argv) {
    ::testing::InitGoogleTest(&argc,argv);
    if(argc!=2){std::cerr<<"Required original canonical wall manifest argument\n";return 2;}
    asset=argv[1];return RUN_ALL_TESTS();
}
