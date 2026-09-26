#include "Values.h"
#include "lib_src/math/ScalarBits.h"
#include <algorithm>
#include <cmath>
#include <vector>
namespace crash::cases::vehicle_self_contact::native::initial_controls::detail {
GapScalars ResolveGaps(tl::util::ConstView<post_gapm::PhysicalOwner> owners,
    tl::util::ConstView<gap_operands::Binding> bindings,tl::util::ConstView<n::source_gaps::PhysicalShell> shells,
    const n::source_gaps::Profile& profile,const n::source_gaps::Report& report) {
    using output::Require;
    Require(owners.size()!=0&&owners.size()<=524288&&shells.size()<=524288&&bindings.size()<=600000,
        "Initial scalar gap source extent exceeds qualified bounds");
    Require(profile.property_type==1&&profile.input_thickness_mode==0&&profile.level==1&&profile.gap_mode==1&&
        profile.free_edge_gap==0&&profile.contact_thickness_update==0&&tl::math::SameScalarBits(profile.scale,1.)&&
        report.completed&&report.status==n::source_gaps::Status::Ok&&std::isfinite(report.maximum_secondary)&&
        report.maximum_secondary>=0,"Initial scalar gap requires the genuine completed ordinary source profile");
    struct Row {std::uint64_t id;std::size_t original,operand;gap_operands::Family family;};
    std::vector<Row> rows;rows.reserve(shells.size());
    std::vector<unsigned char> used(shells.size(),0);
    for(const auto& b:bindings) {
        if(b.family!=gap_operands::Family::Quad&&b.family!=gap_operands::Family::Triangle)continue;
        Require(b.original_id&&b.source_index!=SIZE_MAX&&b.operand_row<shells.size()&&!used[b.operand_row],
            "Initial scalar gap shell binding is incomplete or duplicated");
        const auto& s=shells[b.operand_row];
        Require(s.source_element_id==b.native_id&&s.source_element_id==b.original_id&&
            s.layout==(b.family==gap_operands::Family::Quad?n::ShellLayout::Quad4:n::ShellLayout::Triangle3),
            "Initial scalar gap shell identity/family differs");
        used[b.operand_row]=1;rows.push_back({b.original_id,b.source_index,b.operand_row,b.family});
    }
    Require(rows.size()==shells.size(),"Initial scalar gap does not cover every physical shell operand");
    std::sort(rows.begin(),rows.end(),[](const auto& a,const auto& b){return a.id<b.id;});
    for(std::size_t i=1;i<rows.size();++i)Require(rows[i-1].id!=rows[i].id,"Repeated source shell identity");
    GapScalars result;result.secondary_maximum=report.maximum_secondary;
    for(const auto& owner:owners) {
        if(owner.kind==n::startup::PhysicalSupportKind::EightSlotSolid)continue;
        Require(owner.kind==n::startup::PhysicalSupportKind::ShellQuad||owner.kind==n::startup::PhysicalSupportKind::ShellTriangle,
            "Initial scalar gap has unresolved final physical support");
        const auto it=std::lower_bound(rows.begin(),rows.end(),owner.source_element,
            [](const auto& row,auto id){return row.id<id;});
        Require(it!=rows.end()&&it->id==owner.source_element&&it->original==owner.physical_row&&
            (it->family==gap_operands::Family::Quad)==(owner.kind==n::startup::PhysicalSupportKind::ShellQuad),
            "Selected INCOQ3 shell support is not the authentic physical gap operand");
        const auto& s=shells[it->operand];
        // Match the consumed ordinary branch, leaving unrelated coefficient and
        // structural fields unread. The explicit unit/scale profile is already1.
        const double thickness=s.part_contact_thickness!=0?s.part_contact_thickness:
            s.element_thickness!=0?s.element_thickness:s.property_thickness;
        Require(std::isfinite(thickness)&&thickness>=0,"Selected native shell gap thickness is nonfinite or negative");
        const double gap=n::source_shells::detail::HalfGap(s,0);
        Require(std::isfinite(gap)&&(thickness==0||gap>0),"Selected native half-gap is unrepresentable");
        // Native finite MAX retains the later equal operand, including zero sign.
        result.pre_ini_main_maximum=result.pre_ini_main_maximum>gap?result.pre_ini_main_maximum:gap;
        ++result.shell_supports;
    }
    result.global_search_gap=result.secondary_maximum+result.pre_ini_main_maximum;
    Require(std::isfinite(result.global_search_gap)&&result.global_search_gap>=0,"Initial global search gap is unrepresentable");
    return result;
}
}
