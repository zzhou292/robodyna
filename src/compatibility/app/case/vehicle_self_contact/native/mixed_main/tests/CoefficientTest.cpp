#include "NativeValues.h"
#include "output/ArtifactIO.h"
#include "lib_utest/qualification/radioss_type25_reader_solid_coefficients/Cases.h"
#include <gtest/gtest.h>
#include <limits>
namespace crash::cases::vehicle_self_contact::native::post_gapm::test {
namespace {
void Same(const detail::MainResult& a,const detail::MainResult& b) {
    EXPECT_EQ(output::Bits(a.primary),output::Bits(b.primary));
    EXPECT_EQ(output::Bits(a.partner),output::Bits(b.partner));
    EXPECT_EQ(output::Bits(a.solid_length),output::Bits(b.solid_length));
}
n::NativeShellMainCoefficientInput Shell(double scale,bool triangle) {
    n::NativeShellMainCoefficientInput s;
    s.face=n::MainFaceKind::OrdinaryExterior;s.layout=triangle?n::ShellLayout::Triangle3:n::ShellLayout::Quad4;
    s.property_type=1;s.scale=scale;s.property_thickness=.75;s.young=70.;return s;
}
}
TEST(PostGapmCoefficientValues, NativeSignedInternalAndExteriorBeforePositiveShellOverride) {
    std::size_t negative=0,overridden=0;
    for(const auto& c:reader_solid_test::Cases()) for(bool shell:{false,true}) for(bool triangle:{false,true}) {
        // This source overlay deliberately requires positive representable STC;
        // zero-scale signed solid packets remain covered without a shell arm.
        if(shell && !(c.input.first.scale>0.))continue;
        detail::MainPacket p;p.has_solid=true;p.solid=c.input;
        p.has_shell=shell;p.copy_partner=shell&&triangle;p.shell=Shell(c.input.first.scale,triangle);
        detail::MainResult out;
        ASSERT_EQ(detail::Evaluate(p,&out),n::CoefficientStatus::Ok);
        Same(out,Native(p,c.second_coordinates));
        negative+=out.primary<0.;overridden+=shell&&out.primary==p.shell.scale*p.shell.property_thickness*p.shell.young;
    }
    EXPECT_GT(negative,0u);EXPECT_GT(overridden,0u);
}
TEST(PostGapmCoefficientValues, ShellOnlyAndRoleZeroOverlayDoNotInventPartnerOrReadSolidPayload) {
    for(bool triangle:{false,true})for(bool partner:{false,true}) {
        detail::MainPacket p;p.has_shell=true;p.copy_partner=partner;p.shell=Shell(1.,triangle);
        p.solid.first.volume=std::numeric_limits<double>::quiet_NaN();
        p.solid.second_bulk=std::numeric_limits<double>::quiet_NaN();
        detail::MainResult out;
        ASSERT_EQ(detail::Evaluate(p,&out),n::CoefficientStatus::Ok);
        Same(out,Native(p,{}));EXPECT_EQ(out.solid_length,0.);
        if(!partner)EXPECT_EQ(out.partner,0.);
    }
}
TEST(PostGapmCoefficientValues, InvalidConsumedFieldsScaleAndPhasePreserveThenRetry) {
    const auto base=reader_solid_test::Base(true);
    for(unsigned variant=0;variant<6;++variant) {
        detail::MainPacket p;p.has_solid=true;p.solid=base.input;
        p.has_shell=true;p.copy_partner=true;p.shell=Shell(p.solid.first.scale,false);
        if(variant==0)p.solid.first.volume=0.;
        if(variant==1)p.solid.first.area=0.;
        if(variant==2)p.solid.second_volume=0.;
        if(variant==3)p.shell.scale*=2.;
        if(variant==4)p.shell.young=std::numeric_limits<double>::quiet_NaN();
        if(variant==5)p.has_shell=false;
        detail::MainResult value{71.,-0.,19.};
        EXPECT_NE(detail::Evaluate(p,&value),n::CoefficientStatus::Ok);Same(value,{71.,-0.,19.});
    }
    detail::MainPacket good;good.has_solid=true;good.solid=base.input;
    detail::MainResult value;
    EXPECT_EQ(detail::Evaluate(good,&value),n::CoefficientStatus::Ok);
    Same(value,Native(good,base.second_coordinates));
}
}
