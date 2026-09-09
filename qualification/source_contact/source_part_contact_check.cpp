#include "SourcePartContactFixture.h"

#include "case/CanonicalWallArtifacts.h"
#include "case/WallTessellation.h"
#include "collision/SurfaceMaterialMeasure.h"
#include "collision/PlanarWallBox.h"
#include "output/ArtifactIO.h"
#include <gtest/gtest.h>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <set>
#include <sstream>
#include <type_traits>

namespace {
namespace fixture=crash::qualification::source_contact;
namespace ct=tlfea::contact;
namespace io=crash::output;
namespace cw=crash::case_data;
std::filesystem::path readiness_path,wall_path;
constexpr std::array<std::uint64_t,6> NativeT3Ids{{2126272,2126274,2126283,2213538,2213540,2213595}};
constexpr std::array<std::uint64_t,3> MixedProjectionIds{{2126280,2126284,2213539}};

template<class T> auto Bytes(const T& value) {
    static_assert(std::is_trivially_copyable_v<T>);
    std::array<unsigned char,sizeof(T)> bytes; std::memcpy(bytes.data(),&value,sizeof(T)); return bytes;
}
std::string Exact(double value) {std::ostringstream s;s<<std::setprecision(17)<<value;return s.str();}
struct LVec {
    long double x=0,y=0,z=0;
    LVec operator+(LVec b) const {return {x+b.x,y+b.y,z+b.z};}
    LVec operator-(LVec b) const {return {x-b.x,y-b.y,z-b.z};}
    LVec operator*(long double a) const {return {x*a,y*a,z*a};}
};
LVec Long(ct::Vec3 a) {return {a.x,a.y,a.z};}
LVec Cross(LVec a,LVec b) {return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};}
long double Norm(LVec a) {return std::hypot(a.x,a.y,a.z);}
// Independent value oracle: differentiate the original edge interpolation,
// not the implementation's stored corner cross-product bounds.
LVec Q4Cross(const fixture::SourcePartContactFixture& data,const ct::SurfaceQ4& p,long double u,long double v) {
    const auto x=data.positions();
    const LVec a=Long(x.at(p.nodes[0])),b=Long(x.at(p.nodes[1]));
    const LVec c=Long(x.at(p.nodes[2])),d=Long(x.at(p.nodes[3]));
    const auto du=((a-b)*(1+v)+(d-c)*(1-v))*.25L;
    const auto dv=((a-d)*(1+u)+(b-c)*(1-u))*.25L;
    return Cross(du,dv);
}
void Contains(ct::Q4IntegralInterval bounds,long double value) {
    ASSERT_TRUE(std::isfinite(bounds.lower));ASSERT_TRUE(std::isfinite(bounds.upper));
    EXPECT_LE(static_cast<long double>(bounds.lower),value);
    EXPECT_GE(static_cast<long double>(bounds.upper),value);
}
void SamePosition(ct::Vec3 a,ct::Vec3 b) {
    EXPECT_EQ(io::Bits(a.x),io::Bits(b.x));EXPECT_EQ(io::Bits(a.y),io::Bits(b.y));EXPECT_EQ(io::Bits(a.z),io::Bits(b.z));
}
class SourcePartContactCheck:public ::testing::Test {
  protected:
    fixture::SourcePartContactFixture data;
    void SetUp() override {
        const auto report=fixture::LoadPinnedSourcePartContact(readiness_path,&data);
        ASSERT_EQ(report.status,fixture::FixtureStatus::Ok)<<report.diagnostic;
    }
};

TEST_F(SourcePartContactCheck, AuthenticatedCoordinatesAndEverySourceBindingSurvive) {
    const auto bytes=io::ReadBounded(readiness_path,fixture::ReadinessBytes);
    ASSERT_EQ(io::Sha256(bytes),fixture::ReadinessSha256);
    io::Document raw;
    raw.Parse<rapidjson::kParseNumbersAsStringsFlag|rapidjson::kParseIterativeFlag>(bytes.data(),bytes.size());
    ASSERT_FALSE(raw.HasParseError());
    const auto& nodes=raw["geometry"]["nodes"];
    std::set<std::uint64_t> node_ids,parent_ids;std::set<std::uint32_t> canonical_nodes,canonical_parents;
    for (unsigned n=0;n<fixture::NodeCount;++n) {
        const auto& binding=data.nodes()[n];
        EXPECT_TRUE(node_ids.insert(binding.source_id).second);EXPECT_TRUE(canonical_nodes.insert(binding.canonical_index).second);
        const auto& source=nodes[n];
        EXPECT_EQ(binding.source_id,std::strtoull(source["source_id"].GetString(),nullptr,10));
        EXPECT_EQ(binding.canonical_index,std::strtoull(source["canonical_index"].GetString(),nullptr,10));
        EXPECT_EQ(binding.source_line,std::strtoull(source["source_line"].GetString(),nullptr,10));
        EXPECT_EQ(binding.blank_mask,std::strtoull(source["blank_mask"].GetString(),nullptr,10));
        for (unsigned j=0;j<2;++j) EXPECT_EQ(binding.codes[j],std::strtol(source["codes"][j].GetString(),nullptr,10));
        for (unsigned j=0;j<3;++j) {
            const auto& token=source["position_m"][j];ASSERT_TRUE(token.IsString());char* end=nullptr;
            const double expected=std::strtod(token.GetString(),&end);ASSERT_EQ(end,token.GetString()+token.GetStringLength());
            EXPECT_EQ(io::Bits(data.coordinates()[3*n+j]),io::Bits(expected));
        }
    }
    std::set<std::uint64_t> triangles;unsigned q4_count=0;
    const auto& shells=raw["geometry"]["shells"];
    for (unsigned p=0;p<fixture::ParentCount;++p) {
        const auto& binding=data.parents()[p];const auto& source=shells[p];
        EXPECT_TRUE(parent_ids.insert(binding.source_id).second);EXPECT_TRUE(canonical_parents.insert(binding.canonical_index).second);
        EXPECT_EQ(binding.source_id,std::strtoull(source["source_id"].GetString(),nullptr,10));
        EXPECT_EQ(binding.canonical_index,std::strtoull(source["canonical_index"].GetString(),nullptr,10));
        EXPECT_EQ(binding.source_line,std::strtoull(source["source_line"].GetString(),nullptr,10));
        EXPECT_EQ(binding.blank_mask,std::strtoull(source["blank_mask"].GetString(),nullptr,10));
        EXPECT_EQ(binding.arity,std::strtoull(source["arity"].GetString(),nullptr,10));
        for (unsigned n=0;n<6;++n) EXPECT_EQ(binding.raw_record[n],std::strtoull(source["raw_record"][n].GetString(),nullptr,10));
        for (unsigned n=0;n<4;++n) {
            const auto& node=data.nodes()[binding.local_node_indices[n]];
            EXPECT_EQ(binding.local_node_indices[n],std::strtoull(source["local_node_indices"][n].GetString(),nullptr,10));
            EXPECT_EQ(binding.canonical_node_indices[n],std::strtoull(source["canonical_node_indices"][n].GetString(),nullptr,10));
            EXPECT_EQ(binding.raw_record[2+n],node.source_id);EXPECT_EQ(binding.canonical_node_indices[n],node.canonical_index);
        }
        if (binding.arity==3) {
            triangles.insert(binding.source_id);EXPECT_EQ(binding.local_node_indices[3],binding.local_node_indices[2]);
        } else ++q4_count;
    }
    EXPECT_EQ(q4_count,fixture::Q4Count);
    EXPECT_EQ(triangles,(std::set<std::uint64_t>{NativeT3Ids.begin(),NativeT3Ids.end()}));
    EXPECT_EQ(node_ids.size(),fixture::NodeCount);EXPECT_EQ(parent_ids.size(),fixture::ParentCount);
}

TEST_F(SourcePartContactCheck, EveryOriginalParentHasIntrinsicMeasureAndIndependentDensity) {
    const auto before=Bytes(data);unsigned q4_count=0,t3_count=0;
    double aggregate_lower=0,aggregate_upper=0;
    const double samples[]{-1,-.5,0,.375,1};
    for (unsigned p=0;p<fixture::ParentCount;++p) {
        SCOPED_TRACE(data.parents()[p].source_id);ct::Q4IntegralInterval area;
        if (data.parents()[p].arity==4) {
            ct::SurfaceQ4 parent;ASSERT_TRUE(data.q4_parent(p,parent));
            ct::Q4MaterialMeasure measure;
            ASSERT_EQ(ct::PrepareQ4MaterialMeasure(data.positions(),parent,&measure),ct::SurfaceMeasureStatus::Ok);
            ASSERT_TRUE(measure.prepared());area=measure.area_enclosure();
            EXPECT_EQ(measure.parent().parent_element_id,parent.parent_element_id);
            for (unsigned n=0;n<4;++n) SamePosition(measure.position(n),data.positions().at(parent.nodes[n]));
            ct::Q4IntegralInterval root;
            ASSERT_EQ(ct::BoundQ4MaterialDensity(measure,{},&root),ct::SurfaceMeasureStatus::Ok);
            for (double u:samples) for (double v:samples) {
                const long double expected=Norm(Q4Cross(data,parent,u,v));
                ct::Q4CertifiedIntegral density;
                ASSERT_EQ(ct::EvaluateQ4MaterialDensity(measure,u,v,&density),ct::SurfaceMeasureStatus::Ok);
                Contains({density.lower,density.upper},expected);Contains(root,expected);
                EXPECT_LE(std::abs(static_cast<long double>(density.value)-expected),static_cast<long double>(density.error));
            }
            ct::Q4IntegralInterval sub;
            ASSERT_EQ(ct::BoundQ4MaterialDensity(measure,{-.75,.5,-.25,.625},&sub),ct::SurfaceMeasureStatus::Ok);
            for (double u:{-.75,-.125,.5}) for (double v:{-.25,.1875,.625}) Contains(sub,Norm(Q4Cross(data,parent,u,v)));
            ++q4_count;
        } else {
            ct::SurfaceTriangle parent;ASSERT_TRUE(data.t3_parent(p,parent));ct::T3MaterialMeasure measure;
            ASSERT_EQ(ct::PrepareT3MaterialMeasure(data.positions(),parent,&measure),ct::SurfaceMeasureStatus::Ok);
            ASSERT_TRUE(measure.prepared());area=measure.area_enclosure();
            const auto a=Long(data.positions().at(parent.nodes[0])),b=Long(data.positions().at(parent.nodes[1]));
            const auto c=Long(data.positions().at(parent.nodes[2]));const long double density=Norm(Cross(b-a,c-a));
            Contains({measure.density().lower,measure.density().upper},density);Contains(area,.5L*density);
            for (unsigned n=0;n<3;++n) SamePosition(measure.position(n),data.positions().at(parent.nodes[n]));
            EXPECT_EQ(measure.parent().parent_element_id,parent.parent_element_id);++t3_count;
        }
        EXPECT_GT(area.lower,0);EXPECT_GE(area.upper,area.lower);aggregate_lower+=area.lower;aggregate_upper+=area.upper;
    }
    EXPECT_EQ(q4_count,fixture::Q4Count);EXPECT_EQ(t3_count,fixture::T3Count);EXPECT_EQ(Bytes(data),before);
    // Coarse diagnostic sums only; not an outward aggregate/contact integral.
    RecordProperty("coarse_area_lower_sum_m2",Exact(aggregate_lower));RecordProperty("coarse_area_upper_sum_m2",Exact(aggregate_upper));
    RecordProperty("source_mechanics_qualified","false");
}

TEST_F(SourcePartContactCheck, ActualFiniteMeshWallCoversAllSourceSweptBoxesIncludingMixedProjection) {
    const auto bytes=cw::ReadPinnedWallManifest(wall_path.string());cw::CanonicalWall wall;
    std::istringstream input(bytes);ASSERT_EQ(wall.Load(input).status,cw::WallStatus::Ok);
    cw::WallTessellation prepared;
    const auto report=prepared.Initialize(wall,bytes,cw::WallTessellationKind::Original);
    ASSERT_EQ(report.status,cw::WallTessellationStatus::Ok)<<report.diagnostic;
    ASSERT_EQ(prepared.view().vertex_count,62u);ASSERT_EQ(prepared.view().triangle_count,100u);
    ASSERT_TRUE(prepared.geometry());const double wall_x=prepared.geometry()->wall_x();
    std::set<std::uint64_t> mixed;long double smallest_fraction=1;
    for (unsigned p=0;p<fixture::ParentCount;++p) {
        const auto& binding=data.parents()[p];SCOPED_TRACE(binding.source_id);
        const auto first=data.positions().at(binding.local_node_indices[0]);
        ct::PlanarWallBox box{{wall_x,first.y,first.z},{wall_x,first.y,first.z}};
        for (unsigned n=0;n<binding.arity;++n) {
            const auto x=data.positions().at(binding.local_node_indices[n]);
            // Explicit prescribed lateral endpoint translation only, no source
            // rewrite or dynamics. Positive shape weights enclose the full
            // linear path in these endpoint-coordinate extrema.
            for (const auto endpoint:{x,ct::Vec3{x.x,x.y+.002,x.z-.003}}) {
                box.minimum.y=std::min(box.minimum.y,endpoint.y);box.maximum.y=std::max(box.maximum.y,endpoint.y);
                box.minimum.z=std::min(box.minimum.z,endpoint.z);box.maximum.z=std::max(box.maximum.z,endpoint.z);
            }
        }
        ct::PlanarWallBoxCoverage coverage;
        const auto checked=ct::CheckPlanarWallBox(*prepared.geometry(),box,1e-6,binding.source_id,
                                                ct::PlanarWallBoxMode::ConservativeExpansion,&coverage);
        ASSERT_EQ(checked.status,ct::PlanarContactStatus::Ok)<<checked.message;EXPECT_TRUE(coverage.covered);
        SamePosition(coverage.physical.minimum,box.minimum);SamePosition(coverage.physical.maximum,box.maximum);
        if (binding.arity==4) {
            ct::SurfaceQ4 parent;ASSERT_TRUE(data.q4_parent(p,parent));bool positive=false,negative=false;
            for (double u:{-1.,1.}) for (double v:{-1.,1.}) {
                const auto cross=Q4Cross(data,parent,u,v);positive=positive||cross.x>0;negative=negative||cross.x<0;
                smallest_fraction=std::min(smallest_fraction,std::abs(cross.x)/Norm(cross));
            }
            if (positive&&negative) mixed.insert(binding.source_id);
        }
    }
    EXPECT_EQ(mixed,(std::set<std::uint64_t>{MixedProjectionIds.begin(),MixedProjectionIds.end()}));
    EXPECT_LT(smallest_fraction,.002L);EXPECT_GT(smallest_fraction,0);
    RecordProperty("minimum_absolute_normal_x_fraction",Exact(static_cast<double>(smallest_fraction)));
    RecordProperty("covered_source_parents",static_cast<int>(fixture::ParentCount));
    RecordProperty("contact_force_or_motion_admitted","false");
}

TEST_F(SourcePartContactCheck, FailedReadHashAndTypedRequestsPreserveCompleteFixtureAndRetry) {
    const auto before=Bytes(data);const auto saved_coordinates=data.coordinates();
    const auto saved_nodes=data.nodes();const auto saved_parents=data.parents();
    fixture::SourcePartContactFixture empty;ct::SurfaceQ4 quad;
    quad.feature_id=999;const auto old_quad=Bytes(quad);
    EXPECT_FALSE(empty.q4_parent(0,quad));EXPECT_EQ(Bytes(quad),old_quad);
    EXPECT_FALSE(data.q4_parent(fixture::ParentCount,quad));EXPECT_EQ(Bytes(quad),old_quad);
    ct::SurfaceTriangle triangle;triangle.feature_id=888;const auto old_triangle=Bytes(triangle);
    for (unsigned p=0;p<fixture::ParentCount;++p) {
        if (data.parents()[p].arity==3) {EXPECT_FALSE(data.q4_parent(p,quad));EXPECT_EQ(Bytes(quad),old_quad);}
        else {EXPECT_FALSE(data.t3_parent(p,triangle));EXPECT_EQ(Bytes(triangle),old_triangle);}
    }
    const auto missing=readiness_path.string()+".missing-c5c-fixture";
    EXPECT_EQ(fixture::LoadPinnedSourcePartContact(missing,&data).status,fixture::FixtureStatus::ReadFailure);
    EXPECT_EQ(Bytes(data),before);
    const auto directory=std::filesystem::temp_directory_path()/
        ("robo-dyna-source-contact-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    ASSERT_TRUE(std::filesystem::create_directory(directory));
    struct Cleanup {std::filesystem::path path;~Cleanup(){std::error_code error;std::filesystem::remove_all(path,error);}} cleanup{directory};
    auto bytes=io::ReadBounded(readiness_path,fixture::ReadinessBytes);ASSERT_FALSE(bytes.empty());bytes[0]='[';
    const auto changed=directory/"changed.json";io::WriteBytes(changed,bytes);
    EXPECT_EQ(fixture::LoadPinnedSourcePartContact(changed,&data).status,fixture::FixtureStatus::HashMismatch);
    EXPECT_EQ(Bytes(data),before);io::WriteBytes(directory/"oversized.json",bytes+" ");
    EXPECT_EQ(fixture::LoadPinnedSourcePartContact(directory/"oversized.json",&data).status,fixture::FixtureStatus::ReadFailure);
    EXPECT_EQ(Bytes(data),before);
    EXPECT_EQ(fixture::LoadPinnedSourcePartContact(readiness_path,nullptr).status,fixture::FixtureStatus::InvalidArgument);
    EXPECT_EQ(fixture::LoadPinnedSourcePartContact(readiness_path,&data).status,fixture::FixtureStatus::Ok);
    // Successful reload preserves physical fields, though padding is not an API.
    EXPECT_EQ(data.coordinates(),saved_coordinates);
    for (unsigned n=0;n<fixture::NodeCount;++n) {
        EXPECT_EQ(data.nodes()[n].source_id,saved_nodes[n].source_id);
        EXPECT_EQ(data.nodes()[n].canonical_index,saved_nodes[n].canonical_index);
        EXPECT_EQ(data.nodes()[n].source_line,saved_nodes[n].source_line);
        EXPECT_EQ(data.nodes()[n].codes,saved_nodes[n].codes);EXPECT_EQ(data.nodes()[n].blank_mask,saved_nodes[n].blank_mask);
    }
    for (unsigned p=0;p<fixture::ParentCount;++p) {
        EXPECT_EQ(data.parents()[p].source_id,saved_parents[p].source_id);
        EXPECT_EQ(data.parents()[p].canonical_index,saved_parents[p].canonical_index);
        EXPECT_EQ(data.parents()[p].source_line,saved_parents[p].source_line);
        EXPECT_EQ(data.parents()[p].raw_record,saved_parents[p].raw_record);
        EXPECT_EQ(data.parents()[p].canonical_node_indices,saved_parents[p].canonical_node_indices);
        EXPECT_EQ(data.parents()[p].local_node_indices,saved_parents[p].local_node_indices);
        EXPECT_EQ(data.parents()[p].arity,saved_parents[p].arity);EXPECT_EQ(data.parents()[p].blank_mask,saved_parents[p].blank_mask);
    }
}
} // namespace

int main(int argc,char** argv) {
    ::testing::InitGoogleTest(&argc,argv);
    if (argc!=3) {std::cerr<<"Usage: source_part_contact_check pinned-readiness.json canonical-wall.manifest.json [gtest options]\n";return 2;}
    readiness_path=argv[1];wall_path=argv[2];
    if (!std::filesystem::is_regular_file(readiness_path)||!std::filesystem::is_regular_file(wall_path)) {
        std::cerr<<"Both authenticated source fixtures are required\n";return 2;
    }
    return RUN_ALL_TESTS();
}
