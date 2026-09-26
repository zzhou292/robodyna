#include "../Values.h"
#include "output/ArtifactIO.h"
#include <gtest/gtest.h>
#include <limits>
#include <array>
namespace crash::cases::vehicle_self_contact::native::initial_controls::test {
extern "C" void rd_initial_gap(const int*,const int*,const double*,const double*,const double*,double*);
namespace {
struct Packet {
    std::vector<post_gapm::PhysicalOwner> owners;
    std::vector<gap_operands::Binding> bindings;
    std::vector<n::source_gaps::PhysicalShell> shells;
    n::source_gaps::Profile profile{1,0,1,1,0,0,1.,1e30,1e30};
    n::source_gaps::Report report;
    Packet() {report.status=n::source_gaps::Status::Ok;report.completed=true;report.maximum_secondary=2.;}
    void Shell(bool triangle,double part,double element,double property) {
        const auto i=shells.size();const std::uint64_t id=100+i;
        n::source_gaps::PhysicalShell row;row.source_element_id=id;
        row.layout=triangle?n::ShellLayout::Triangle3:n::ShellLayout::Quad4;
        row.part_contact_thickness=part;row.element_thickness=element;row.property_thickness=property;
        row.young=row.structural_thickness=std::numeric_limits<double>::quiet_NaN();
        shells.push_back(row);bindings.push_back({triangle?gap_operands::Family::Triangle:gap_operands::Family::Quad,id,id,900+i,i,i});
        owners.push_back({triangle?n::startup::PhysicalSupportKind::ShellTriangle:n::startup::PhysicalSupportKind::ShellQuad,id,900+i,i});
    }
    GapScalars Actual()const{return detail::ResolveGaps({owners.data(),owners.size()},{bindings.data(),bindings.size()},
        {shells.data(),shells.size()},profile,report);}
    std::array<double,2> Oracle()const {
        const int count=int(owners.size());std::vector<int> kinds(count,0);std::vector<double> values(3*count,0.);
        for(int i=0;i<count;++i)if(owners[i].kind!=n::startup::PhysicalSupportKind::EightSlotSolid) {
            const auto& row=shells[owners[i].physical_row];kinds[i]=row.layout==n::ShellLayout::Triangle3?3:4;
            values[3*i]=row.part_contact_thickness;values[3*i+1]=row.element_thickness;values[3*i+2]=row.property_thickness;
        }
        std::array<double,2> result{};rd_initial_gap(&count,kinds.data(),values.data(),&report.maximum_secondary,&profile.maximum_main,result.data());
        return result;
    }
};
void NativeEqual(const Packet& p) {
    const auto a=p.Actual();const auto e=p.Oracle();
    EXPECT_EQ(output::Bits(a.pre_ini_main_maximum),output::Bits(e[0]));
    EXPECT_EQ(output::Bits(a.global_search_gap),output::Bits(e[1]));
}
}
TEST(InitializerScalarGap, OriginalOwnerReductionExcludesUnselectedShellsSolidLengthAndPostNodalCaps) {
    Packet p;p.Shell(false,0,0,4.);p.Shell(true,0,0,6.);p.Shell(false,0,0,1e6);
    p.owners.pop_back();p.owners.push_back({n::startup::PhysicalSupportKind::EightSlotSolid,70,80,999});
    p.profile.maximum_main=.125;
    NativeEqual(p);const auto actual=p.Actual();EXPECT_EQ(actual.pre_ini_main_maximum,3.);
    EXPECT_EQ(actual.global_search_gap,5.);EXPECT_EQ(actual.shell_supports,2u);
    EXPECT_NE(actual.pre_ini_main_maximum,p.profile.maximum_main);
}
TEST(InitializerScalarGap, ConsumedThicknessBranchesAndAllZeroSignsMatchOriginalMaxOrdering) {
    const double poison=std::numeric_limits<double>::quiet_NaN();
    Packet p;p.Shell(false,8,poison,poison);p.Shell(true,0,10,poison);p.Shell(false,0,0,12);NativeEqual(p);
    for(unsigned mask=0;mask<8;++mask) {
        Packet z;z.report.maximum_secondary=0.;
        z.Shell(false,0,0,(mask&1)?-0.:0.);z.Shell(true,0,0,(mask&2)?-0.:0.);z.Shell(false,0,0,(mask&4)?-0.:0.);
        NativeEqual(z);
    }
}
TEST(InitializerScalarGap, IncompleteIdentityProfileAndLateNonfiniteRejectThenRetry) {
    Packet p;p.Shell(false,0,0,4.);p.Shell(true,0,0,6.);const auto expected=p.Actual();
    for(unsigned bad=0;bad<6;++bad) {
        auto q=p;
        if(bad==0)q.owners.back().physical_row=0;
        if(bad==1)q.bindings.pop_back();
        if(bad==2)q.shells.back().property_thickness=std::numeric_limits<double>::infinity();
        if(bad==3)q.profile.scale=2.;
        if(bad==4)q.report.completed=false;
        if(bad==5)q.owners.back().source_element=10100;
        EXPECT_THROW(q.Actual(),std::exception);
    }
    EXPECT_EQ(output::Bits(p.Actual().global_search_gap),output::Bits(expected.global_search_gap));
    auto overflow=p;overflow.shells.back().property_thickness=1e308;overflow.report.maximum_secondary=1.5e308;
    EXPECT_THROW(overflow.Actual(),std::exception);
}
}
