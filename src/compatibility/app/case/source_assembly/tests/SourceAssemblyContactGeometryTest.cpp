#include "SourceAssemblyBindingTestSupport.h"
#include "case/shell_collection/ShellCollectionContactGeometry.h"
#include <limits>
#include <vector>

namespace crash::cases::source_assembly::test {
namespace sc=tlfea::contact;
namespace {
// Independent long-double measure of the represented coordinates. This is a
// contact-area check only; no structural mass is calculated from this area.
long double Area(const fe::ShellBatchBinding& b,const sc::NodalWallParentWeight& p) {
    long double x[4][3]{};
    for(unsigned l=0;l<p.arity;++l) {
        const auto v=b.nodes()[p.nodes[l]].position;x[l][0]=v.x;x[l][1]=v.y;x[l][2]=v.z;
    }
    long double a[3],c[3];
    for(unsigned k=0;k<3;++k) {
        a[k]=p.arity==4?(x[0][k]-x[1][k]-x[2][k]+x[3][k])/4:x[1][k]-x[0][k];
        c[k]=p.arity==4?(x[0][k]+x[1][k]-x[2][k]-x[3][k])/4:x[2][k]-x[0][k];
    }
    const long double cross[]{a[1]*c[2]-a[2]*c[1],a[2]*c[0]-a[0]*c[2],a[0]*c[1]-a[1]*c[0]};
    return (p.arity==4?4.L:.5L)*std::sqrt(cross[0]*cross[0]+cross[1]*cross[1]+cross[2]*cross[2]);
}
void Encloses(sc::Q4CertifiedIntegral certificate,long double truth) {
    EXPECT_LE(static_cast<long double>(certificate.lower),truth);
    EXPECT_GE(static_cast<long double>(certificate.upper),truth);
}
}
TEST(SourceAssemblyContactGeometry, EveryActualParentAndNativeNodeRetainsItsCertifiedAreaAndSourceMapping) {
    const auto assembly=SourceAssemblyBindings::Prepare(Load(),Options());
    ShellCollectionContactGeometry geometry;const auto report=geometry.Initialize(assembly.shells());
    ASSERT_TRUE(report)<<report.message<<" EID="<<report.parent.source_id;
    const auto& b=assembly.shells(); const auto& w=*geometry.weights();
    ASSERT_EQ(w.parent_count(),915u);ASSERT_EQ(w.node_count(),1030u);
    EXPECT_EQ(geometry.binding()->inventory(),b.inventory());
    std::vector<long double> node_area(b.node_count());long double total=0;
    std::size_t quads=0,triangles=0;
    for(unsigned p=0;p<w.parent_count();++p) {
        const auto& parent=w.parent(p);const auto* key=geometry.parent_from_weight(p);ASSERT_NE(key,nullptr);
        EXPECT_EQ(key->source_id,parent.parent_element_id);EXPECT_EQ(key->source_id,parent.feature_id);
        const bool q=key->family==fe::ShellBindingFamily::Qeph;
        if(q) ++quads;else ++triangles;
        EXPECT_EQ(parent.arity,q?4u:3u);
        EXPECT_EQ(key->source_id,q?b.qeph_source_id(key->family_index):b.t3_source_id(key->family_index));
        for(unsigned l=0;l<parent.arity;++l)
            EXPECT_EQ(parent.nodes[l],q?b.qeph_nodes(key->family_index)[l]:b.t3_nodes(key->family_index)[l]);
        const auto area=Area(b,parent);Encloses(parent.area,area);Encloses(parent.share,area/parent.arity);total+=area;
        for(unsigned l=0;l<parent.arity;++l) node_area[parent.nodes[l]]+=area/parent.arity;
    }
    EXPECT_EQ(quads,804u);EXPECT_EQ(triangles,111u);Encloses(w.total_area(),total);
    const auto bounds=geometry.reference_bounds();const auto positions=geometry.positions();
    for(unsigned n=0;n<w.node_count();++n) {
        EXPECT_EQ(w.node(n).node,n);Encloses(w.node(n).area,node_area[n]);
        const auto x=positions.at(n);const auto native=b.nodes()[n].position;
        SameBits(x.x,native.x);SameBits(x.y,native.y);SameBits(x.z,native.z);
        EXPECT_GE(x.x,bounds[0].x);EXPECT_LE(x.x,bounds[1].x);
        EXPECT_GE(x.y,bounds[0].y);EXPECT_LE(x.y,bounds[1].y);
        EXPECT_GE(x.z,bounds[0].z);EXPECT_LE(x.z,bounds[1].z);
    }
    EXPECT_EQ(geometry.parent_from_weight(w.parent_count()),nullptr);
    EXPECT_EQ(geometry.parent_from_weight(std::numeric_limits<std::size_t>::max()),nullptr);
    RecordProperty("maximum_reference_x_m",Number(bounds[1].x));
    RecordProperty("total_reference_contact_area_m2",Number(w.total_area().value));
    RecordProperty("startup_payload_budget_bytes",std::to_string(geometry.startup_payload_bytes()));
}
TEST(SourceAssemblyContactGeometry, OwnedGeometrySurvivesSourceDestructionAndPreservesPublicationOnFailure) {
    ShellCollectionContactGeometry geometry;
    {
        const auto source=SourceAssemblyBindings::Prepare(Load(),Options());
        for(unsigned variant=0;variant<4;++variant) {
            ShellContactGeometryLimits cap;
            if(variant==0)cap.max_startup_bytes=1;
            if(variant==1)cap.weights.max_nodes=1029;
            if(variant==2)cap.weights.max_parents=914;
            if(variant==3)cap.weights.max_owned_bytes=1;
            EXPECT_FALSE(geometry.Initialize(source.shells(),cap));EXPECT_FALSE(geometry.prepared());
        }
        ASSERT_TRUE(geometry.Initialize(source.shells()));
    }
    const auto* held=geometry.weights();const auto* binding=geometry.binding();const auto x=geometry.positions().at(1029);
    EXPECT_FALSE(geometry.Initialize(*binding));EXPECT_EQ(geometry.weights(),held);EXPECT_EQ(geometry.binding(),binding);
    SameBits(geometry.positions().at(1029).x,x.x);EXPECT_EQ(held->node(1029).node,1029u);
    fe::ShellBatchBinding empty;ShellCollectionContactGeometry retry;
    EXPECT_FALSE(retry.Initialize(empty));ASSERT_TRUE(retry.Initialize(*binding));
    EXPECT_EQ(retry.weights()->total_area().value,held->total_area().value);
}
} // namespace crash::cases::source_assembly::test
