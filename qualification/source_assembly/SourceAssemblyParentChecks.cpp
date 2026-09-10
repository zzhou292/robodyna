#include "SourceAssemblyFlightFixture.h"
#include "SourceAssemblyParentValues.h"
#include "SourceAssemblyParentIntervals.h"
#include "lib_src/elements/qeph/QephLayeredJ2.h"
#include "lib_src/elements/t3/T3LayeredJ2.h"
#include "lib_utest/qualification/qeph/QephForceFixture.h"
#include "lib_utest/qualification/t3/T3ForcePortFixture.h"

namespace crash::qualification::source_assembly {
namespace {
namespace qt=qeph_force_port_test;
namespace tt=t3_force_port_test;
void Near(double a,double b) {
    ASSERT_TRUE(std::isfinite(a));ASSERT_TRUE(std::isfinite(b));
    EXPECT_LE(std::abs(a-b),2e-12*std::max({1.,std::abs(a),std::abs(b)}));
}
void Triangle(const t::ReferenceData& reference,const t::PrescribedInterval& in,const t::ForceTrial& a,const t::ForceTrial& b) {
    // Reuse test-only value/field utilities; no native reference reconstruction
    // or Fortran execution is needed to compare resident storage with host math.
    tt::RatesAgreement(a.kinematics,tt::Native(b.kinematics),in);
    const auto& x=a.proposed_history.data();const auto& y=b.proposed_history.data();
    for(unsigned i=0;i<5;++i) {Near(x.stress[i],y.stress[i]);Near(x.material_stress[i],y.material_stress[i]);}
    for(unsigned i=0;i<3;++i)Near(x.bending_stress[i],y.bending_stress[i]);
    for(unsigned i=0;i<8;++i)Near(x.strain_curvature[i],y.strain_curvature[i]);
    for(unsigned i=0;i<2;++i)Near(x.internal_work[i],y.internal_work[i]);
    Near(x.thickness,y.thickness);Near(x.equivalent_strain_rate,y.equivalent_strain_rate);EXPECT_EQ(x.active,y.active);
    EXPECT_EQ(a.proposed_history.stamp().time,b.proposed_history.stamp().time);
    EXPECT_EQ(a.proposed_history.stamp().sample_index,b.proposed_history.stamp().sample_index);
    EXPECT_TRUE(a.proposed_history.matches_reference(reference));
    for(unsigned n=0;n<3;++n)for(unsigned axis=0;axis<3;++axis) {
        Near(tt::Component(a.internal_force[n],axis),tt::Component(b.internal_force[n],axis));
        Near(tt::Component(a.internal_couple[n],axis),tt::Component(b.internal_couple[n],axis));
    }
    const auto da=tt::Diagnostics(a.diagnostics),db=tt::Diagnostics(b.diagnostics);
    for(unsigned i=0;i<da.size();++i)Near(da[i],db[i]);
}
void Section(const fe::ShellBatchSectionState& actual,const fe::sections::ShellLayeredJ2History& expected,
    const fe::sections::ShellLayeredJ2Diagnostics& diagnostic,double expected_work) {
    for(unsigned p=0;p<3;++p) {
        const auto& a=actual.history.point[p];const auto& b=expected.point[p];
        for(unsigned c=0;c<5;++c)Near(a.stress[c],b.stress[c]);
        Near(a.plastic_strain,b.plastic_strain);Near(a.filtered_rate_per_s,b.filtered_rate_per_s);
    }
    const auto& a=actual.diagnostics;const auto& b=diagnostic;
    Near(a.plastic_work_density_increment,b.plastic_work_density_increment);Near(a.maximum_plastic_strain,b.maximum_plastic_strain);
    Near(a.mean_plastic_strain,b.mean_plastic_strain);Near(a.minimum_tangent_ratio,b.minimum_tangent_ratio);
    Near(a.mean_tangent_ratio,b.mean_tangent_ratio);Near(a.mean_yield_before_pa,b.mean_yield_before_pa);
    Near(a.last_point_yield_before_pa,b.last_point_yield_before_pa);Near(actual.cumulative_plastic_work_J,expected_work);
}
}
void CheckTriangleValues(const t::ReferenceData& r,const t::PrescribedInterval& in,const t::ForceTrial& a,const t::ForceTrial& b) {
    Triangle(r,in,a,b);
}
void CheckPlasticSectionValues(const fe::ShellBatchSectionState& a,const fe::sections::ShellLayeredJ2History& b,
    const fe::sections::ShellLayeredJ2Diagnostics& d,double work) { Section(a,b,d,work); }
void CheckHostParents(const Rig& r,const Prepared& p,const ShellFields& accepted,const ShellFields& proposed) {
    const auto& b=r.bindings.shells();const auto& catalog=r.bindings.materials();
    for(std::size_t e=0;e<r.quads();++e) {
        SCOPED_TRACE(b.qeph_source_id(e));const auto in=QParentInterval(r,p,e);
        fe::sections::PointParameters parameters;ASSERT_TRUE(catalog.Parameters(fe::ShellBindingFamily::Qeph,e,&parameters));
        q::LayeredJ2ForceTrial expected;
        ASSERT_EQ(q::EvaluateLayeredJ2Force(b.qeph_reference(e),parameters,
            {accepted.quad[e].proposed_history,accepted.qsection[e].history},in,expected),q::Status::kSuccess);
        qt::ForceAgreement(proposed.quad[e],expected.force,b.qeph_reference(e).input,in);
        EXPECT_TRUE(proposed.quad[e].proposed_history.matches_reference(b.qeph_reference(e)));
        Section(proposed.qsection[e],expected.proposed_section,expected.section_diagnostics,
            accepted.qsection[e].cumulative_plastic_work_J+expected.section_diagnostics.plastic_work_density_increment*
            accepted.quad[e].proposed_history.data().thickness*expected.force.kinematics.area);
    }
    for(std::size_t e=0;e<r.triangles();++e) {
        SCOPED_TRACE(b.t3_source_id(e));const auto in=TParentInterval(r,p,e);
        fe::sections::PointParameters parameters;ASSERT_TRUE(catalog.Parameters(fe::ShellBindingFamily::T3,e,&parameters));
        t::LayeredJ2ForceTrial expected;
        ASSERT_EQ(t::EvaluateLayeredJ2Force(b.t3_reference(e),parameters,
            {accepted.triangle[e].proposed_history,accepted.tsection[e].history},in,expected),t::Status::kSuccess);
        Triangle(b.t3_reference(e),in,proposed.triangle[e],expected.force);
        Section(proposed.tsection[e],expected.proposed_section,expected.section_diagnostics,
            accepted.tsection[e].cumulative_plastic_work_J+expected.section_diagnostics.plastic_work_density_increment*
            accepted.triangle[e].proposed_history.data().thickness*expected.force.kinematics.area);
    }
}
}
