// SPDX-License-Identifier: AGPL-3.0-or-later
#include "DiscoverySerialization.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

namespace {
namespace ct=tlfea::contact;
namespace ft=fixed_triangle_test;
namespace serial=exact_storage_discovery;
struct Scene {
    const char* name;
    std::vector<ct::CurrentFixedTriangle> triangles;
    std::vector<ct::FixedTrianglePair> pairs;
    bool require_intersection=false,require_seam_dedup=false;
};
Scene Pair(const char* name,const ct::Vec3 (&a)[3],const std::uint64_t (&ia)[3],
           const ct::Vec3 (&b)[3],const std::uint64_t (&ib)[3],bool intersects=false) {
    return {name,{ft::Triangle(100,0,a,ia),ft::Triangle(200,0,b,ib)},{{0,1}},intersects};
}
std::vector<Scene> Corpus() {
    // Existing Geometry/Intersection/Stratum owning fixtures, with unchanged
    // coordinate values and canonical Fixture.h construction utilities.
    const std::uint64_t ia[]{1,2,3},ib[]{11,12,13},shared_vertex[]{1,12,13},shared_edge[]{2,1,4};
    const ct::Vec3 a[]{{0,0,0},{2,0,0},{0,2,0}};
    const ct::Vec3 separate[]{{.25,.25,1},{2.25,.25,1},{.25,2.25,1}};
    const ct::Vec3 vertex[]{{0,0,0},{-2,0,1},{0,-2,1}};
    const ct::Vec3 edge[]{{2,0,0},{0,0,0},{2,-2,0}};
    const ct::Vec3 folded[]{{2,0,0},{0,0,0},{1,1,0}};
    const ct::Vec3 plane[]{{-2,-2,0},{2,-2,0},{0,2,0}};
    const ct::Vec3 piercing[]{{0,-.5,-1},{0,.5,1},{0,1,-1}};
    const ct::Vec3 contained[]{{.25,.25,0},{.75,.25,0},{.25,.75,0}};
    std::vector<Scene> result{
        Pair("separated",a,ia,separate,ib),
        Pair("shared_vertex",a,ia,vertex,shared_vertex,true),
        Pair("shared_edge",a,ia,edge,shared_edge,true),
        Pair("folded_shared_edge",a,ia,folded,shared_edge,true),
        Pair("transverse_piercing",plane,ia,piercing,ib,true),
        Pair("coplanar_overlap",a,ia,contained,ib,true),
        Pair("coincident_distinct_source",a,ia,a,ib,true),
        Pair("identical_source_face",a,ia,a,ia,true)};
    const ct::Vec3 source[]{{0,0,1},{2,0,2},{0,2,2}};
    const ct::Vec3 upper[]{{-1,0,0},{1,0,0},{0,2,0}};
    const ct::Vec3 lower[]{{1,0,0},{-1,0,0},{0,-2,0}};
    const std::uint64_t is[]{50,51,52},iu[]{11,12,13},il[]{12,11,14};
    result.push_back({"seam_producers",{ft::Triangle(100,0,source,is),ft::Triangle(200,0,upper,iu),
        ft::Triangle(300,0,lower,il)},{{0,1},{0,2}},false,true});
    const ct::Vec3 target_edge[]{{134217730.,-134217729.,67108863.},
        {134217715.,-134217718.,67108875.},{134217737.,-134217739.,67108854.}};
    const ct::Vec3 qe{134217883.0000143,-134218126.0000105,67109458.99998856};
    const ct::Vec3 source_edge[]{qe,{qe.x+100,qe.y,qe.z},{qe.x,qe.y+100,qe.z}};
    result.push_back(Pair("rounded_boundary_exact_edge",source_edge,is,target_edge,ia));
    const ct::Vec3 target_vertex[]{{4294967305.,-4294967284.,2147483632.},
        {4294967321.,-4294967293.,2147483642.},{4294967323.,-4294967267.,2147483660.}};
    const ct::Vec3 qv{4294963947.,-4294969411.,2147487132.};
    const ct::Vec3 source_vertex[]{qv,{qv.x+100,qv.y,qv.z},{qv.x,qv.y+100,qv.z}};
    result.push_back(Pair("rounded_boundary_exact_vertex",source_vertex,is,target_vertex,ia));
    // Regular parallel facets with a finite strict gap and wide exponent
    // spread exercise the production wide fallback in closest-stratum work.
    const double tiny=std::ldexp(1.,-100);
    const ct::Vec3 wide_gap[]{{0,0,tiny},{2,0,tiny},{0,2,tiny}};
    result.push_back(Pair("wide_exponent_parallel_gap",a,ia,wide_gap,ib));
    return result;
}
Scene Permuted(Scene scene,unsigned permutation) {
    constexpr unsigned order[6][3]{{0,1,2},{0,2,1},{1,0,2},{1,2,0},{2,0,1},{2,1,0}};
    for(auto& triangle:scene.triangles) {
        const auto before=triangle;
        for(unsigned i=0;i<3;++i) {
            triangle.vertices[i]=before.vertices[order[permutation][i]];
            triangle.vertex_keys[i]=before.vertex_keys[order[permutation][i]];
        }
        for(unsigned i=0;i<3;++i) {
            const auto& first=triangle.vertex_keys[i];const auto& second=triangle.vertex_keys[(i+1)%3];
            bool found=false;
            for(const auto& edge:before.edge_keys)
                if((ft::Same(first,edge.endpoints[0]) && ft::Same(second,edge.endpoints[1])) ||
                   (ft::Same(first,edge.endpoints[1]) && ft::Same(second,edge.endpoints[0]))) {
                    triangle.edge_keys[i]=edge;found=true;break;
                }
            serial::Require(found,"Permutation lost a source edge");
        }
    }
    if(permutation&1) {
        std::reverse(scene.triangles.begin(),scene.triangles.end());
        for(auto& pair:scene.pairs) {
            pair.first=static_cast<std::uint32_t>(scene.triangles.size()-1-pair.first);
            pair.second=static_cast<std::uint32_t>(scene.triangles.size()-1-pair.second);
            std::swap(pair.first,pair.second);
        }
    }
    if(permutation>=3)std::reverse(scene.pairs.begin(),scene.pairs.end());
    return scene;
}
ct::FixedTriangleDiscoveryReport Query(ct::FixedTriangleFeatureDiscovery& owner,const Scene& scene,bool masked) {
    if(!masked)return owner.Discover(scene.triangles.data(),scene.triangles.size(),scene.pairs.data(),scene.pairs.size());
    std::vector<ct::FixedTriangleFeatureTaskMask> masks(scene.pairs.size());
    for(std::size_t i=0;i<masks.size();++i) {
        const auto pair=scene.pairs[i];
        serial::Require(ct::BuildFixedTriangleFeatureTaskMask(scene.triangles[pair.first],scene.triangles[pair.second],&masks[i])==
            ct::FixedTriangleDiscoveryStatus::Ok,"Production local task mask construction failed");
    }
    return owner.DiscoverMasked(scene.triangles.data(),scene.triangles.size(),scene.pairs.data(),scene.pairs.size(),masks.data());
}
void Success(const ct::FixedTriangleDiscoveryReport& report,const ct::FixedTriangleFeatureDiscovery& owner,const Scene& scene) {
    serial::Require(report.status==ct::FixedTriangleDiscoveryStatus::Ok,report.message);
    serial::Require(owner.features().complete && owner.intersections().complete,"Successful query did not publish complete views");
    serial::Require(report.feature_candidates==owner.features().count && report.intersections==owner.intersections().count,"Report and publication count disagree");
    serial::Require(report.potential_tasks==report.local_masked_tasks+report.exact_executed_tasks,"Task partition disagrees");
    if(scene.require_intersection)serial::Require(owner.intersections().count!=0,"Expected explicit intersection is absent");
    if(scene.require_seam_dedup)serial::Require(report.feature_candidates<report.raw_feature_candidates,"Seam corpus did not exercise deduplication");
}
void CorpusQueries(unsigned workers,serial::Writer& writer) {
    ct::FixedTriangleFeatureDiscovery owner;
    auto limits=ft::Limits(8);limits.worker_count=workers;
    const auto initial=owner.Initialize(limits);
    serial::Require(initial.status==ct::FixedTriangleDiscoveryStatus::Ok,initial.message);
    writer.Emit("corpus",workers,0,"initialize",initial,owner);
    for(const auto& scene:Corpus())for(unsigned permutation=0;permutation<6;++permutation) {
        const auto current=Permuted(scene,permutation);
        for(bool masked:{false,true}) {
            const auto report=Query(owner,current,masked);
            writer.Emit(scene.name,workers,permutation,masked?"masked":"unmasked",report,owner);
            Success(report,owner,current);
        }
    }
}
void FailureQueries(unsigned workers,serial::Writer& writer) {
    const auto corpus=Corpus();const auto& good=corpus[0];
    ct::FixedTriangleFeatureDiscovery owner;
    auto limits=ft::Limits(8);limits.worker_count=workers;ft::Initialize(&owner,limits);
    auto report=Query(owner,good,false);Success(report,owner,good);
    writer.Emit("recovery",workers,0,"published_baseline",report,owner);
    const auto before=serial::Publication(owner);
    auto* before_features=owner.features().data;
    auto* before_intersections=owner.intersections().data;
    const auto failure=[&](const char* name,const ct::FixedTriangleDiscoveryReport& failed,ct::FixedTriangleDiscoveryStatus expected) {
        writer.Emit("recovery",workers,0,name,failed,owner);
        serial::Require(failed.status==expected,"Unexpected recovery failure class");
        serial::Require(serial::Publication(owner)==before,"Failed query changed previous complete publication");
        serial::Require(owner.features().data==before_features && owner.intersections().data==before_intersections,
            "Failed query switched the retained publication buffers");
        const auto retried=Query(owner,good,false);Success(retried,owner,good);
        writer.Emit("recovery",workers,0,"retry",retried,owner);
        serial::Require(serial::Publication(owner)==before,"Recovery retry changed baseline publication");
        // Reborrow after success: success may legitimately swap publication buffers.
        before_features=owner.features().data;before_intersections=owner.intersections().data;
    };
    auto malformed=good;malformed.pairs.push_back({0,2});
    failure("late_out_of_range",Query(owner,malformed,false),ct::FixedTriangleDiscoveryStatus::OutOfRange);
    auto degenerate=good;degenerate.triangles[0].vertices[2]=degenerate.triangles[0].vertices[1];
    failure("degenerate_geometry",Query(owner,degenerate,false),ct::FixedTriangleDiscoveryStatus::DegenerateTriangle);
    const ct::FixedTriangleFeatureTaskMask forged{ct::FixedTriangleFeatureTaskBit(0)};
    failure("forged_remote_mask",owner.DiscoverMasked(good.triangles.data(),good.triangles.size(),good.pairs.data(),1,&forged),ct::FixedTriangleDiscoveryStatus::InvalidInput);
    auto nonfinite=good;nonfinite.triangles[0].vertices[0].x=std::numeric_limits<double>::infinity();
    report=Query(owner,nonfinite,false);
    writer.Emit("recovery",workers,0,"nonfinite_geometry",report,owner);
    serial::Require(report.status!=ct::FixedTriangleDiscoveryStatus::Ok,"Nonfinite geometry was accepted");
    serial::Require(serial::Publication(owner)==before,"Nonfinite input replaced old publication");
    serial::Require(owner.features().data==before_features && owner.intersections().data==before_intersections,
        "Nonfinite input switched retained publication buffers");
    report=Query(owner,good,false);Success(report,owner,good);
    writer.Emit("recovery",workers,0,"nonfinite_retry",report,owner);

    // Reuse a tight owner across a positive publication, complete capacity
    // failure and retry. The published shared-vertex result has nine features;
    // the separated query needs fifteen and cannot publish a partial prefix.
    ct::FixedTriangleFeatureDiscovery tight;
    limits=ft::Limits(2,30,14,2,2);limits.worker_count=workers;ft::Initialize(&tight,limits);
    const auto& adjacent=corpus[1];
    report=Query(tight,adjacent,false);Success(report,tight,adjacent);
    writer.Emit("capacity",workers,0,"published_nine",report,tight);
    serial::Require(tight.features().count==9,"Capacity fixture did not publish nine features");
    const auto retained=serial::Publication(tight);
    const auto* retained_features=tight.features().data;
    const auto* retained_intersections=tight.intersections().data;
    report=Query(tight,good,false);
    writer.Emit("capacity",workers,0,"complete_count_failure",report,tight);
    serial::Require(report.status==ct::FixedTriangleDiscoveryStatus::ResourceLimit && report.feature_candidates==15,"Expected complete feature-cap failure absent");
    serial::Require(serial::Publication(tight)==retained,"Feature-cap failure replaced old publication");
    serial::Require(tight.features().data==retained_features && tight.intersections().data==retained_intersections,
        "Feature-cap failure switched retained publication buffers");
    report=Query(tight,adjacent,true);Success(report,tight,adjacent);
    writer.Emit("capacity",workers,0,"masked_retry",report,tight);
    serial::Require(serial::Publication(tight)==retained,"Masked retry changed equivalent retained feature/intersection values");
}
} // namespace
int main() {
    try {
        serial::Writer writer(std::cout);writer.Begin();
        for(unsigned workers:{1u,4u}) {CorpusQueries(workers,writer);FailureQueries(workers,writer);}
        writer.End();return 0;
    } catch(const std::exception& error) {
        std::cerr<<"discovery parity driver: "<<error.what()<<'\n';return 1;
    }
}
