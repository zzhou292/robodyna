#include "SourceAssemblyParentIntervals.h"
#include "SourceAssemblyParentValues.h"
#include "lib_src/elements/qeph/QephLayeredLaw1.h"
#include "lib_src/elements/t3/T3LayeredLaw1.h"
#include "lib_src/elements/qeph/QephLayeredJ2.h"
#include "lib_src/elements/t3/T3LayeredJ2.h"
#include "lib_utest/qualification/qeph/QephForceFixture.h"
#include <algorithm>
#include <cmath>

namespace crash::qualification::source_assembly {
namespace {
void Elastic(const fe::ShellBatchLayeredSection& actual,const fe::sections::ShellLayeredLaw1History& expected) {
    ASSERT_NE(actual.elastic(),nullptr);EXPECT_EQ(actual.plastic(),nullptr);
    for(unsigned l=0;l<3;++l)for(unsigned c=0;c<5;++c) {
        const auto a=actual.elastic()->point[l].stress[c],b=expected.point[l].stress[c];
        ASSERT_TRUE(std::isfinite(a));ASSERT_TRUE(std::isfinite(b));
        EXPECT_LE(std::abs(a-b),2e-12*std::max({1.,std::abs(a),std::abs(b)}));
    }
}
}
void CheckHostParents(const Rig& r,const Prepared& p,const LayeredShellFields& old,const LayeredShellFields& proposed) {
    const auto& binding=r.bindings.shells();const auto& catalog=r.bindings.materials();
    for(std::size_t e=0;e<r.quads();++e) {
        SCOPED_TRACE(binding.qeph_source_id(e));const auto in=QParentInterval(r,p,e);
        const auto& reference=binding.qeph_reference(e);const auto& h=old.qsection[e];
        if(h.law()==fe::ShellSectionLaw::LayeredLaw1Nip3) {
            tl::material::ShellElasticLaw1PointParameters material;
            ASSERT_TRUE(catalog.ElasticParameters(fe::ShellBindingFamily::Qeph,e,&material));ASSERT_NE(h.elastic(),nullptr);
            q::LayeredLaw1ForceTrial expected;
            ASSERT_EQ(q::EvaluateLayeredLaw1Force(reference,material,{old.quad[e].proposed_history,*h.elastic()},in,expected),q::Status::kSuccess);
            qeph_force_port_test::ForceAgreement(proposed.quad[e],expected.force,reference.input,in);
            Elastic(proposed.qsection[e],expected.proposed_section);
        } else {
            fe::sections::PointParameters material;ASSERT_TRUE(catalog.Parameters(fe::ShellBindingFamily::Qeph,e,&material));
            ASSERT_NE(h.plastic(),nullptr);ASSERT_NE(proposed.qsection[e].plastic(),nullptr);q::LayeredJ2ForceTrial expected;
            ASSERT_EQ(q::EvaluateLayeredJ2Force(reference,material,{old.quad[e].proposed_history,h.plastic()->history},in,expected),q::Status::kSuccess);
            qeph_force_port_test::ForceAgreement(proposed.quad[e],expected.force,reference.input,in);
            CheckPlasticSectionValues(*proposed.qsection[e].plastic(),expected.proposed_section,expected.section_diagnostics,
                h.plastic()->cumulative_plastic_work_J+expected.section_diagnostics.plastic_work_density_increment*
                old.quad[e].proposed_history.data().thickness*expected.force.kinematics.area);
        }
        EXPECT_TRUE(proposed.quad[e].proposed_history.matches_reference(reference));
    }
    for(std::size_t e=0;e<r.triangles();++e) {
        SCOPED_TRACE(binding.t3_source_id(e));const auto in=TParentInterval(r,p,e);
        const auto& reference=binding.t3_reference(e);const auto& h=old.tsection[e];
        if(h.law()==fe::ShellSectionLaw::LayeredLaw1Nip3) {
            tl::material::ShellElasticLaw1PointParameters material;
            ASSERT_TRUE(catalog.ElasticParameters(fe::ShellBindingFamily::T3,e,&material));ASSERT_NE(h.elastic(),nullptr);
            t::LayeredLaw1ForceTrial expected;
            ASSERT_EQ(t::EvaluateLayeredLaw1Force(reference,material,{old.triangle[e].proposed_history,*h.elastic()},in,expected),t::Status::kSuccess);
            CheckTriangleValues(reference,in,proposed.triangle[e],expected.force);Elastic(proposed.tsection[e],expected.proposed_section);
        } else {
            fe::sections::PointParameters material;ASSERT_TRUE(catalog.Parameters(fe::ShellBindingFamily::T3,e,&material));
            ASSERT_NE(h.plastic(),nullptr);ASSERT_NE(proposed.tsection[e].plastic(),nullptr);t::LayeredJ2ForceTrial expected;
            ASSERT_EQ(t::EvaluateLayeredJ2Force(reference,material,{old.triangle[e].proposed_history,h.plastic()->history},in,expected),t::Status::kSuccess);
            CheckTriangleValues(reference,in,proposed.triangle[e],expected.force);
            CheckPlasticSectionValues(*proposed.tsection[e].plastic(),expected.proposed_section,expected.section_diagnostics,
                h.plastic()->cumulative_plastic_work_J+expected.section_diagnostics.plastic_work_density_increment*
                old.triangle[e].proposed_history.data().thickness*expected.force.kinematics.area);
        }
    }
}
} // namespace crash::qualification::source_assembly
