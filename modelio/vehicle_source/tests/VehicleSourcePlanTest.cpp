#include "TestSupport.h"
#include <set>

namespace crash::modelio::vehicle::test {
TEST(VehicleSourcePlan, CompleteOriginalSelectionKeepsTypedAndUnresolvedParents) {
    const auto& p=Plan();const auto& c=p.counts();
    EXPECT_EQ(c.parents,349645);EXPECT_EQ(c.nodes,359785);EXPECT_EQ(c.parts,867);
    EXPECT_EQ(c.q4,328344);EXPECT_EQ(c.t3,21301);EXPECT_EQ(c.supported_parts,823);EXPECT_EQ(c.supported_parents,278301);
    std::size_t elastic=0,linear=0,table=0,unresolved=0;
    for(std::size_t i=0;i<p.parts().size();++i) {
        const auto& part=p.parts()[i];const auto* m=p.material(i);const auto* s=p.section(i);
        if(part.status==Disposition::Unresolved) {
            EXPECT_EQ(m,nullptr);EXPECT_EQ(s,nullptr);unresolved+=part.shell_count;
            EXPECT_FALSE(part.obligations.empty());for(const auto& b:part.unresolved_sources)EXPECT_EQ(output::Sha256(b.raw_text),b.sha256);
        } else {
            ASSERT_NE(m,nullptr);ASSERT_NE(s,nullptr);EXPECT_EQ(m->id,part.material_id);EXPECT_EQ(s->id,part.section_id);
            if(m->law==assembly::MaterialLaw::LayeredLaw1)elastic+=part.shell_count;
            else if(m->hardening==assembly::MaterialHardening::LinearLaw44)linear+=part.shell_count;
            else table+=part.shell_count;
        }
    }
    EXPECT_EQ(elastic,27177);EXPECT_EQ(linear,52483);EXPECT_EQ(table,198641);EXPECT_EQ(unresolved,71344);
    const auto& records=source::FindArray(p.canonical().data(),"shells_records");
    const auto r=output::arrays::Decode<std::uint64_t>(records.descriptor,records.bytes);
    std::set<std::uint64_t> selected;
    std::uint32_t previous=0;bool first=true;
    for(const auto& parent:p.parents()) {
        EXPECT_TRUE(first||parent.canonical_parent>previous);first=false;previous=parent.canonical_parent;
        EXPECT_EQ(r[6*parent.canonical_parent+1],p.parts()[parent.part_index].part_id);
        EXPECT_TRUE(selected.insert(r[6*parent.canonical_parent]).second);
    }
    EXPECT_EQ(selected.size(),349645);EXPECT_EQ(p.canonical_nodes().size(),359785);
    EXPECT_EQ(&p.canonical().data(),&Canonical().data());EXPECT_LT(p.startup_budget_bytes(),512*1024*1024);
}
TEST(VehicleSourcePlan, ActualV3ProjectionsAndSharedCopyMoveKeepSourceBits) {
    const auto& p=Plan();auto copy=p;auto moved=std::move(copy);
    EXPECT_EQ(&copy.canonical().data(),&p.canonical().data());EXPECT_EQ(moved.parents().data(),p.parents().data());
    for(bool mixed:{false,true}) {
        const auto* path=std::getenv(mixed?"ROBO_VEHICLE_MIXED":"ROBO_VEHICLE_ELASTIC");ASSERT_NE(path,nullptr);
        const auto original=assembly::SourceAssembly::Read(path,assembly::test::section::Identity(mixed));
        for(const auto& part:original.data().parts) {
            const auto found=std::find_if(p.parts().begin(),p.parts().end(),[&](const auto& row){return row.part_id==part.id;});
            ASSERT_NE(found,p.parts().end());const auto i=found-p.parts().begin();
            const auto& a=*p.material(i);const auto& b=*std::find_if(original.data().materials.begin(),original.data().materials.end(),[&](const auto& m){return m.id==part.material_id;});
            EXPECT_EQ(a.law,b.law);EXPECT_EQ(a.hardening,b.hardening);EXPECT_EQ(a.curve_id,b.curve_id);
            EXPECT_EQ(output::Bits(a.young_pa),output::Bits(b.young_pa));EXPECT_EQ(output::Bits(a.density_kg_m3),output::Bits(b.density_kg_m3));
            EXPECT_EQ(output::Bits(a.poisson_ratio),output::Bits(b.poisson_ratio));EXPECT_EQ(a.source.raw_text,b.source.raw_text);
            const auto& section=*std::find_if(original.data().sections.begin(),original.data().sections.end(),[&](const auto& s){return s.id==part.section_id;});
            EXPECT_EQ(p.section(i)->thickness_m,section.thickness_m);EXPECT_EQ(p.section(i)->source.raw_text,section.source.raw_text);
        }
        const auto& node_ids=source::FindArray(p.canonical().data(),"node_ids");
        const auto ids=output::arrays::Decode<std::uint64_t>(node_ids.descriptor,node_ids.bytes);
        const auto& positions=source::FindArray(p.canonical().data(),"node_positions");
        const auto xyz=output::arrays::Decode<double>(positions.descriptor,positions.bytes);
        for(const auto& n:original.data().nodes) {
            ASSERT_EQ(ids[n.canonical_index],n.source_id);
            EXPECT_EQ(output::Bits(xyz[3*n.canonical_index]),output::Bits(n.position_m.x));
            EXPECT_EQ(output::Bits(xyz[3*n.canonical_index+1]),output::Bits(n.position_m.y));
            EXPECT_EQ(output::Bits(xyz[3*n.canonical_index+2]),output::Bits(n.position_m.z));
        }
    }
}
TEST(VehicleSourcePlan, EveryOriginalElasticParentKeepsRawNipAndResolvedDriverSeparate) {
    const auto& p=Plan();std::size_t parts=0,q=0,t=0;
    const auto& conn=source::FindArray(p.canonical().data(),"shells_node_indices");
    const auto nodes=output::arrays::Decode<std::uint32_t>(conn.descriptor,conn.bytes);
    for(std::size_t i=0;i<p.parts().size();++i) {
        const auto driver=p.law1_driver(i);const auto* m=p.material(i);
        if(!m) {EXPECT_EQ(driver.status(),assembly::Law1DriverStatus::Unavailable);continue;}
        if(m->source.keyword!="*MAT_ELASTIC") {EXPECT_EQ(driver.status(),assembly::Law1DriverStatus::NotElastic);continue;}
        ++parts;ASSERT_TRUE(driver.available());EXPECT_EQ(driver.material_id(),m->id);
        EXPECT_EQ(driver.section_id(),p.section(i)->id);EXPECT_EQ(driver.raw_nip(),3u);EXPECT_EQ(driver.resolved_npt(),0);
        EXPECT_EQ(p.section(i)->through_thickness_points,3u);
    }
    for(const auto& parent:p.parents())if(p.law1_driver(parent.part_index).available()) {
        const auto n=4*std::size_t(parent.canonical_parent);
        nodes[n+2]==nodes[n+3]?++t:++q;
    }
    EXPECT_EQ(parts,10u);EXPECT_EQ(q,26225u);EXPECT_EQ(t,952u);
    EXPECT_EQ(p.law1_driver(p.parts().size()).status(),assembly::Law1DriverStatus::Unavailable);
    auto copy=p;EXPECT_EQ(copy.law1_driver(0).status(),p.law1_driver(0).status());
    EXPECT_EQ(&copy.canonical().data(),&p.canonical().data());
}
} // namespace crash::modelio::vehicle::test
