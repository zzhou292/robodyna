#include "SourceAssemblyFlightFixture.h"
#include "output/ArtifactIO.h"

namespace crash::qualification::source_assembly {
namespace {
void Bits(double a,double b) { EXPECT_EQ(output::Bits(a),output::Bits(b)); }
void Section(const fe::ShellBatchLayeredSection& a,const fe::ShellBatchLayeredSection& b) {
    ASSERT_EQ(a.law(),b.law());
    if(a.elastic()) {
        ASSERT_NE(b.elastic(),nullptr);EXPECT_EQ(a.plastic(),nullptr);EXPECT_EQ(b.plastic(),nullptr);
        for(unsigned l=0;l<3;++l)for(unsigned c=0;c<5;++c)Bits(a.elastic()->point[l].stress[c],b.elastic()->point[l].stress[c]);
    } else {
        ASSERT_NE(a.plastic(),nullptr);ASSERT_NE(b.plastic(),nullptr);
        const auto& x=*a.plastic();const auto& y=*b.plastic();
        for(unsigned l=0;l<3;++l) {
            for(unsigned c=0;c<5;++c)Bits(x.history.point[l].stress[c],y.history.point[l].stress[c]);
            Bits(x.history.point[l].plastic_strain,y.history.point[l].plastic_strain);
            Bits(x.history.point[l].filtered_rate_per_s,y.history.point[l].filtered_rate_per_s);
        }
        const auto& u=x.diagnostics;const auto& v=y.diagnostics;
        Bits(u.plastic_work_density_increment,v.plastic_work_density_increment);Bits(u.maximum_plastic_strain,v.maximum_plastic_strain);
        Bits(u.mean_plastic_strain,v.mean_plastic_strain);Bits(u.minimum_tangent_ratio,v.minimum_tangent_ratio);
        Bits(u.mean_tangent_ratio,v.mean_tangent_ratio);Bits(u.mean_yield_before_pa,v.mean_yield_before_pa);
        Bits(u.last_point_yield_before_pa,v.last_point_yield_before_pa);Bits(x.cumulative_plastic_work_J,y.cumulative_plastic_work_J);
    }
}
}
void SameShells(const LayeredShellFields& a,const LayeredShellFields& b) {
    EXPECT_EQ(std::memcmp(a.quad.data(),b.quad.data(),a.quad.size()*sizeof(q::ForceTrial)),0);
    EXPECT_EQ(std::memcmp(a.triangle.data(),b.triangle.data(),a.triangle.size()*sizeof(t::ForceTrial)),0);
    ASSERT_EQ(a.qsection.size(),b.qsection.size());ASSERT_EQ(a.tsection.size(),b.tsection.size());
    for(std::size_t i=0;i<a.qsection.size();++i)Section(a.qsection[i],b.qsection[i]);
    for(std::size_t i=0;i<a.tsection.size();++i)Section(a.tsection[i],b.tsection[i]);
}
void CheckFlight(const Rig& r,const Fields& state,const LayeredShellFields& shells) {
    CheckFlightMotion(r,state,shells.diagnostics);
    for(const auto& h:shells.quad) {EXPECT_EQ(h.proposed_history.stamp().sample_index,state.stamp.epoch);EXPECT_EQ(h.proposed_history.stamp().time,state.stamp.time);}
    for(const auto& h:shells.triangle) {EXPECT_EQ(h.proposed_history.stamp().sample_index,state.stamp.epoch);EXPECT_EQ(h.proposed_history.stamp().time,state.stamp.time);}
    const auto& catalog=r.bindings.materials();
    for(const auto& parent:r.bindings.source().data().parents) {
        const bool quad=parent.family==src::ShellFamily::Qeph;
        fe::ShellSectionLaw law;
        ASSERT_TRUE(catalog.Law(quad?fe::ShellBindingFamily::Qeph:fe::ShellBindingFamily::T3,parent.family_index,&law));
        const auto& h=(quad?shells.qsection:shells.tsection)[parent.family_index];ASSERT_EQ(h.law(),law);
        if(law==fe::ShellSectionLaw::LayeredLaw1Nip3) {ASSERT_NE(h.elastic(),nullptr);EXPECT_EQ(h.plastic(),nullptr);}
        else {
            ASSERT_NE(h.plastic(),nullptr);EXPECT_EQ(h.elastic(),nullptr);
            EXPECT_EQ(h.plastic()->cumulative_plastic_work_J,0);EXPECT_EQ(h.plastic()->diagnostics.maximum_plastic_strain,0);
            for(const auto& point:h.plastic()->history.point)EXPECT_EQ(point.plastic_strain,0);
        }
    }
}
} // namespace crash::qualification::source_assembly
