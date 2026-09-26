#include "../Values.h"
#include "../PopulationProfile.h"
#include "output/ArtifactIO.h"
#include <gtest/gtest.h>
#include <iomanip>
#include <sstream>
namespace crash::cases::vehicle_self_contact::native::initial_controls::test {
namespace {
std::string Card(std::initializer_list<double> values) {
    std::ostringstream s;for(auto v:values)s<<std::setw(10)<<std::setprecision(6)<<v;return s.str();
}
modelio::self_contact::Data Source() {
    modelio::self_contact::Data d;d.sources.resize(3);
    d.source_fields.slave_set_id=12;d.source_fields.master_set_id=0;d.source_fields.slave_set_type=2;
    d.source_fields.static_friction=.5;d.source_fields.dynamic_friction=.125;d.source_fields.decay_coefficient=0.;
    auto& s=d.sources[0];s.block.keyword="*CONTACT_AUTOMATIC_SINGLE_SURFACE";
    s.cards={{2,Card({12,0,2})},{3,Card({.5,.125,0.})},{4,""},{5,Card({1})},{6,""},{7,Card({0,1})},{8,""},{9,""}};
    // Source IGNORE1 has a genuinely blank IGAP slot, not literal zero.
    s.cards[5].second=std::string(10,' ')+Card({1});return d;
}
}
TEST(InitializerControlValues, RawAndResolvedGapRemovalControlsRemainDistinct) {
    const auto source=Source();const auto raw=detail::ResolveOriginalControls(source);const Controls resolved;
    EXPECT_EQ(raw.reader_gap_mode,2);EXPECT_EQ(resolved.gap_mode,1);EXPECT_EQ(resolved.neighbor_removal,2);
    EXPECT_EQ(raw.reader_tied_removal,0);EXPECT_EQ(resolved.tied_removal,1);
    EXPECT_EQ(raw.reader_idel,1);EXPECT_EQ(raw.stfac,1.);EXPECT_EQ(raw.slsfac,1.);
    EXPECT_EQ(output::Bits(resolved.base_multiplier),output::Bits(static_cast<double>(.20f)));
    EXPECT_EQ(resolved.native_voxel_capacity,8000000u);EXPECT_EQ(resolved.curvature,0);EXPECT_EQ(resolved.native_packet_size,128u);EXPECT_EQ(raw.reader_global_gap,0.);
}
TEST(InitializerControlValues, OriginalFrictionFieldsDriveLawAndPreserveSignedZero) {
    const auto s=Source();const auto law=detail::ResolveSelfLaw(s,{.001,1000,1});
    EXPECT_EQ(law.friction_coefficients.base,.125);EXPECT_EQ(law.friction_coefficients.c[4],.375);
    EXPECT_EQ(output::Bits(law.friction_coefficients.c[5]),output::Bits(-0.));
    EXPECT_EQ(law.friction.model,2);EXPECT_EQ(law.friction.formulation,10);EXPECT_EQ(law.friction.converged,1);
    EXPECT_EQ(law.normal.damping_flag,1);EXPECT_EQ(law.normal.damping_factor,.05);
    EXPECT_EQ(law.lifecycle.main_coefficient_domain,n::MainCoefficientDomain::NativeSigned);
    EXPECT_NE(law.friction_coefficients.base,.6); // Separate added-wall declaration cannot replace original FS/FD.
    auto changed=s;changed.sources[0].cards[1].second=Card({.5,.125,.25});changed.source_fields.decay_coefficient=.25;
    EXPECT_EQ(detail::ResolveSelfLaw(changed,{.001,1000,1}).friction_coefficients.c[5],-.25);
}
TEST(InitializerControlValues, UnknownSourceGapLoadAndLawControlsRejectWithoutChangingOriginalData) {
    const auto good=Source();
    for(unsigned fault=0;fault<6;++fault) {
        auto d=good;
        if(fault==0)d.sources[0].block.keyword="*CONTACT_AUTOMATIC_SINGLE_SURFACE_ID";
        if(fault==1)d.sources[0].cards[2].second=Card({2});
        if(fault==2)d.sources[0].cards[3].second=Card({2});
        if(fault==3)d.sources[0].cards[5].second=Card({1,1});
        if(fault==4)d.source_fields.master_set_id=4;
        if(fault==5)d.sources[0].cards[7].second=Card({1});
        EXPECT_THROW(detail::ResolveOriginalControls(d),std::exception);
    }
    auto different=good;different.source_fields.dynamic_friction=.3;
    EXPECT_THROW(detail::ResolveSelfLaw(different,{.001,1000,1}),std::exception);
    EXPECT_NO_THROW(detail::ResolveOriginalControls(good));
}
TEST(InitializerControlValues, CompletePopulationKeywordTableRejectsPreviouslyUnclosedGenerators) {
    for(const char* key:{"*DEFINE_VECTOR_NODES","*DEFINE_SD_ORIENTATION","*ELEMENT_BEAM_ORIENTATION",
        "*AIRBAG_SHELL_REFERENCE_GEOMETRY","*AIRBAG_REFERENCE_GEOMETRY","*DATABASE_CROSS_SECTION_PLANE",
        "*RIGIDWALL_GEOMETRIC_CYLINDER","*PART_INERTIA"}) {
        EXPECT_THROW(detail::AdmitPopulationKeyword(key),detail::Failure);
    }
    for(const char* key:{"*AIRBAG_SIMPLE_AIRBAG_MODEL_ID","*ELEMENT_BEAM","*ELEMENT_DISCRETE",
        "*DEFINE_TRANSFORMATION","*CONSTRAINED_NODAL_RIGID_BODY","*RIGIDWALL_PLANAR_FINITE_FORCES_ID"}) {
        EXPECT_NO_THROW(detail::AdmitPopulationKeyword(key));
    }
}
TEST(InitializerControlValues, BlankAndLiteralZeroDeathRetainEvidenceWithoutInventedStopTime) {
    auto blank=Source();const auto b=detail::ResolveOriginalControls(blank);
    EXPECT_TRUE(b.source_death_blank);EXPECT_TRUE(b.sensor_disabled);EXPECT_TRUE(b.stop_nonnegative);
    EXPECT_FALSE(b.exact_stop_time_available);EXPECT_GT(b.stop_time_lower_bound_native,.02);
    auto zero=blank;zero.sources[0].cards[1].second=Card({.5,.125,0.,0.,0.,0.,0.,0.});
    const auto z=detail::ResolveOriginalControls(zero);EXPECT_FALSE(z.source_death_blank);
    EXPECT_TRUE(z.sensor_disabled);EXPECT_TRUE(z.stop_nonnegative);EXPECT_FALSE(z.exact_stop_time_available);
    zero.sources[0].cards[1].second=Card({.5,.125,0.,0.,0.,0.,0.,-1.});
    EXPECT_THROW(detail::ResolveOriginalControls(zero),std::exception);
}
}
