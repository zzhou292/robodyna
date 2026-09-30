#pragma once
#include "SourceNodalWallCuda.h"
#include "case/CanonicalWallArtifacts.h"
#include "case/WallTessellation.h"
#include <gtest/gtest.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <limits>
#include <sstream>

namespace crash::qualification::source_contact::nodal::test {
namespace cw=crash::case_data;
inline std::filesystem::path readiness_path,wall_path;
// Frozen host arithmetic budget, distinct from physical model discrepancy and
// from the source per-parent force/energy certificates carried by every result.
inline constexpr double Arithmetic=2e-12;
inline std::string Number(double x) { std::ostringstream out; out<<std::setprecision(17)<<x; return out.str(); }
inline void Near(double a,double b) {
    EXPECT_TRUE(std::isfinite(a)); EXPECT_TRUE(std::isfinite(b));
    EXPECT_LE(std::abs(static_cast<long double>(a)-b),Arithmetic*(1+std::abs(static_cast<long double>(b))));
}
inline void Cert(sc::Q4CertifiedIntegral x,sc::Q4CertifiedIntegral y) {
    for (auto a:{x,y}) {
        EXPECT_TRUE(std::isfinite(a.value)); EXPECT_TRUE(std::isfinite(a.lower));
        EXPECT_TRUE(std::isfinite(a.upper)); EXPECT_TRUE(std::isfinite(a.error));
        EXPECT_GE(a.value,0); EXPECT_GE(a.lower,0); EXPECT_LE(a.lower,a.upper);
        EXPECT_GE(static_cast<long double>(a.error),std::max(std::abs(static_cast<long double>(a.value)-a.lower),
            std::abs(static_cast<long double>(a.value)-a.upper)));
    }
    EXPECT_LE(x.lower,y.upper); EXPECT_LE(y.lower,x.upper);
    EXPECT_LE(std::abs(static_cast<long double>(x.value)-y.value),static_cast<long double>(x.error)+y.error);
}
inline void Vector(sc::Vec3 a,sc::Vec3 b) { Near(a.x,b.x); Near(a.y,b.y); Near(a.z,b.z); }
inline void Compare(const Result& expected,const Result& actual,bool compare_faces=true) {
    const auto& a=expected.contact; const auto& b=actual.contact;
    ASSERT_TRUE(a.valid); ASSERT_TRUE(b.valid); ASSERT_EQ(a.node_count,NodeCount); ASSERT_EQ(b.node_count,NodeCount);
    ASSERT_EQ(a.parent_count,ParentCount); ASSERT_EQ(b.parent_count,ParentCount);
    EXPECT_EQ(a.base_epoch,b.base_epoch); EXPECT_EQ(a.attempt,b.attempt);
    Cert(a.resultant,b.resultant); Cert(a.potential,b.potential);
    Vector(a.wall_reaction,b.wall_reaction); Vector(a.wall_moment,b.wall_moment); Near(a.surface_power,b.surface_power);
    for (unsigned p=0;p<ParentCount;++p) {
        const auto& x=a.parents[p]; const auto& y=b.parents[p]; SCOPED_TRACE(x.parent_element_id);
        EXPECT_TRUE(y.valid); EXPECT_EQ(x.parent_element_id,y.parent_element_id); EXPECT_EQ(x.parent_face_id,y.parent_face_id);
        EXPECT_EQ(x.feature_id,y.feature_id); EXPECT_EQ(x.family,y.family); EXPECT_EQ(x.arity,y.arity);
        Cert(x.resultant,y.resultant); Cert(x.potential,y.potential);
        EXPECT_LE(y.resultant.error,cf::ForceBudget); EXPECT_LE(y.potential.error,cf::EnergyBudget);
        for (unsigned l=0;l<4;++l) { Cert(x.force[l],y.force[l]); EXPECT_LE(y.force[l].error,cf::ForceBudget); }
    }
    for (unsigned n=0;n<NodeCount;++n) {
        const auto& x=a.nodes[n]; const auto& y=b.nodes[n]; SCOPED_TRACE(n);
        EXPECT_TRUE(y.valid); EXPECT_EQ(x.node,y.node); EXPECT_EQ(y.node,n); EXPECT_EQ(x.fixed,y.fixed);
        EXPECT_EQ(x.touching_or_penetrating,y.touching_or_penetrating);
        EXPECT_EQ(x.base_epoch,y.base_epoch); EXPECT_EQ(x.attempt,y.attempt);
        Cert(x.force,y.force); Cert(x.potential,y.potential); Cert(x.stiffness,y.stiffness);
        Vector(x.force_world,y.force_world); Vector(x.wall_point,y.wall_point);
        Vector(x.wall_reaction,y.wall_reaction); Vector(x.wall_moment,y.wall_moment); Near(x.surface_power,y.surface_power);
        EXPECT_EQ(y.local_velocity_first_timestep,0); EXPECT_EQ(x.row.valid,y.row.valid); EXPECT_EQ(x.row.count,y.row.count);
        EXPECT_EQ(x.row.base_epoch,y.row.base_epoch); EXPECT_EQ(x.row.attempt,y.row.attempt);
        for (unsigned l=0;l<sc::kMaxNormalNodes;++l) {
            EXPECT_EQ(x.row.nodes[l],y.row.nodes[l]); Near(x.row.stiffness[l],y.row.stiffness[l]);
            Near(x.row.damping[l],y.row.damping[l]);
        }
        if (compare_faces) EXPECT_EQ(expected.wall_face[n],actual.wall_face[n]);
        EXPECT_NE(actual.wall_face[n],0u);
    }
}
inline void Exact(sc::Q4CertifiedIntegral a,sc::Q4CertifiedIntegral b) {
    EXPECT_EQ(a.value,b.value); EXPECT_EQ(a.lower,b.lower); EXPECT_EQ(a.upper,b.upper); EXPECT_EQ(a.error,b.error);
}
inline void Exact(sc::Vec3 a,sc::Vec3 b) { EXPECT_EQ(a.x,b.x); EXPECT_EQ(a.y,b.y); EXPECT_EQ(a.z,b.z); }
inline void ExactResult(const Result& a,const Result& b) {
    Compare(a,b); Exact(a.contact.resultant,b.contact.resultant); Exact(a.contact.potential,b.contact.potential);
    Exact(a.contact.wall_reaction,b.contact.wall_reaction); Exact(a.contact.wall_moment,b.contact.wall_moment);
    EXPECT_EQ(a.contact.surface_power,b.contact.surface_power);
    for (unsigned p=0;p<ParentCount;++p) {
        Exact(a.contact.parents[p].resultant,b.contact.parents[p].resultant);
        Exact(a.contact.parents[p].potential,b.contact.parents[p].potential);
        for (unsigned l=0;l<4;++l) Exact(a.contact.parents[p].force[l],b.contact.parents[p].force[l]);
    }
    for (unsigned n=0;n<NodeCount;++n) {
        const auto& x=a.contact.nodes[n]; const auto& y=b.contact.nodes[n];
        Exact(x.force,y.force); Exact(x.potential,y.potential); Exact(x.stiffness,y.stiffness);
        Exact(x.force_world,y.force_world); Exact(x.wall_point,y.wall_point); Exact(x.wall_reaction,y.wall_reaction);
        Exact(x.wall_moment,y.wall_moment); EXPECT_EQ(x.surface_power,y.surface_power);
        for (unsigned l=0;l<sc::kMaxNormalNodes;++l) {
            EXPECT_EQ(x.row.stiffness[l],y.row.stiffness[l]); EXPECT_EQ(x.row.damping[l],y.row.damping[l]);
        }
    }
}
template<class T> auto Bytes(const T& value) {
    std::array<unsigned char,sizeof(T)> bytes; std::memcpy(bytes.data(),&value,sizeof(T)); return bytes;
}
template<class T> void Unchanged(const T& value,const std::array<unsigned char,sizeof(T)>& bytes) {
    EXPECT_EQ(std::memcmp(&value,bytes.data(),sizeof(T)),0);
}
inline void Hole(const cw::WallTessellation& original,sc::Vec3 target,sc::PlanarWallGeometry& output,sc::Vec3& center) {
    const auto view=original.view(); std::vector<unsigned> incidence(view.vertex_count,0);
    for (unsigned p=0;p<view.triangle_count;++p) for (unsigned n: view.triangles[p].nodes) ++incidence[n];
    unsigned chosen=UINT32_MAX; double nearest=std::numeric_limits<double>::infinity();
    for (unsigned p=0;p<view.triangle_count;++p) {
        const auto& face=view.triangles[p]; bool removable=true; sc::Vec3 c;
        for (auto n:face.nodes) { removable=removable && incidence[n]>1; c=sc::Add(c,view.vertices[n].position); }
        c=sc::Scale(c,1./3); const double distance=(c.y-target.y)*(c.y-target.y)+(c.z-target.z)*(c.z-target.z);
        if (removable && distance<nearest) { chosen=p; nearest=distance; center=c; }
    }
    ASSERT_NE(chosen,UINT32_MAX);
    std::vector<sc::PlanarWallTriangle> faces;
    for (unsigned p=0;p<view.triangle_count;++p) if (p!=chosen) faces.push_back(view.triangles[p]);
    const sc::PlanarWallView hole{view.vertices,view.vertex_count,faces.data(),static_cast<unsigned>(faces.size())};
    ASSERT_EQ(output.Initialize(hole).status,sc::PlanarContactStatus::Ok);
    ASSERT_EQ(output.faces().size(),99u); // Explicit negative fixture, not the canonical wall.
}

class Check : public ::testing::Test {
  protected:
    SourcePartContactFixture source;
    cw::CanonicalWall wall;
    std::string wall_bytes,diagnostic;
    PreparedSource prepared;
    void SetUp() override {
        const auto report=LoadPinnedSourcePartContact(readiness_path,&source);
        ASSERT_EQ(report.status,FixtureStatus::Ok)<<report.diagnostic;
        wall_bytes=cw::ReadPinnedWallManifest(wall_path.string()); std::istringstream stream(wall_bytes);
        ASSERT_EQ(wall.Load(stream).status,cw::WallStatus::Ok);
        ASSERT_TRUE(prepared.Initialize(source,diagnostic))<<diagnostic;
    }
    void Geometry(cw::WallTessellation& output,cw::WallTessellationKind kind=cw::WallTessellationKind::Original) {
        ASSERT_EQ(output.Initialize(wall,wall_bytes,kind).status,cw::WallTessellationStatus::Ok);
        ASSERT_NE(output.geometry(),nullptr);
    }
    void Coherent(const sc::PlanarWallGeometry& geometry,Input& input) {
        const double shift=cf::WholeShift(source,geometry.wall_x());
        ASSERT_EQ(BuildInput(prepared,source.coordinates(),cf::Shift(source,shift),cf::Velocity(shift),prepared.mass(),
            geometry,31,&input,diagnostic).status,Status::Ok)<<diagnostic;
    }
    unsigned MaximumX() const {
        unsigned index=0;
        for (unsigned n=1;n<NodeCount;++n) if (source.positions().at(n).x>source.positions().at(index).x) index=n;
        return index;
    }
    void Metadata() {
        RecordProperty("face_query_backend","prepared-planar-wall-box-v1");
        RecordProperty("model",sc::NodalWallContactModel); RecordProperty("readiness_sha256",ReadinessSha256);
        RecordProperty("mass_policy",cf::MassPolicy); RecordProperty("source_mass_equivalence","false");
        RecordProperty("mechanics_or_owner_admission","false"); RecordProperty("native_q4",88); RecordProperty("native_t3",6);
        RecordProperty("physical_nodes",117); RecordProperty("native_parent_shares",370);
        RecordProperty("stiffness_per_area",Number(cf::Stiffness)); RecordProperty("target_depth_m",Number(cf::Depth));
        RecordProperty("maximum_depth_m",Number(cf::Cap)); RecordProperty("force_budget_N",Number(cf::ForceBudget));
        RecordProperty("potential_budget_J",Number(cf::EnergyBudget));
    }
};
} // namespace crash::qualification::source_contact::nodal::test
