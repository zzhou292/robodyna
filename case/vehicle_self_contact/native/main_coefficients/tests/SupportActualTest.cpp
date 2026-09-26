#include "../SupportQuery.h"
#include "../SolidSupportQuery.h"
#include "../../mixed_interface/tests/ActualFixture.h"
#include "output/BoundedArrayJson.h"
#include <algorithm>
#include <array>
#include <cstdlib>
namespace crash::cases::vehicle_self_contact::native::main_coefficients::test {
namespace mi = native::mixed_interface;
namespace {
constexpr std::size_t ExtraCap = std::size_t{512} << 20;
constexpr std::size_t ExportCap = 2u << 20;
struct Census {
    std::array<std::size_t,4> matches{}; //0/1/2/>2 distinct physical solids.
    std::size_t pre_internal=0,final_internal=0,solid_order_unresolved=0;
    std::size_t shell_owners=0,solid_owners=0,shell_owner_unresolved=0,no_support=0;
    std::size_t primary_reversals=0,negative_first_volumes=0,zero_first_volumes=0;
    std::array<std::uint64_t,16> unresolved_primary_ids{};
    std::size_t examples=0;
};
void Write(const Census& c,const mi::MixedInterfaceSource& source,std::size_t total) {
    const auto* value=std::getenv("ROBO_POST_GAPM_QUERY_OUTPUT");
    output::Require(value&&*value,"Missing create-only post-GAPM query census output");
    const std::filesystem::path directory(value);
    output::Require(std::filesystem::create_directory(directory),"Post-GAPM query census output exists");
    output::Document d;d.SetObject();
    output::String(d,"schema","robo_dyna.post_gapm_support_query_census.v1");
    output::String(d,"scope","Source query assessment only; no main K/gaps, topology or runtime readiness");
    output::String(d,"mixed_source_digest",source.provenance().output_digest);
    output::Integer(d,"primaries",source.sides().primary_count);
    output::Integer(d,"expanded_mains",source.sides().main_count);
    const char* labels[]{"solid_matches0","solid_matches1","solid_matches2","solid_matches_more_than2"};
    for(unsigned i=0;i<4;++i)output::Integer(d,labels[i],c.matches[i]);
    output::Integer(d,"pre_shell_internal_count",c.pre_internal);
    output::Integer(d,"final_internal_count",c.final_internal);
    output::Integer(d,"unresolved_solid_order",c.solid_order_unresolved);
    output::Integer(d,"final_shell_support",c.shell_owners);
    output::Integer(d,"final_solid_support",c.solid_owners);
    output::Integer(d,"unresolved_shell_owner",c.shell_owner_unresolved);
    output::Integer(d,"missing_all_support",c.no_support);
    output::Integer(d,"primary_reversals",c.primary_reversals);
    output::Integer(d,"negative_exterior_first_volumes",c.negative_first_volumes);
    output::Integer(d,"zero_exterior_first_volumes",c.zero_first_volumes);
    // Pinned regular TYPE25 converter explicitly writes Idel1; DEFINTER only
    // substitutes0, and SURFI setsIPARI100 when P-S>0. GAPM clears only if its
    // PRE-shell internal counter is0. These are source controls, not a guard override.
    output::Integer(d,"source_idel",1);
    const bool incoming=source.sides().primary_count>source.sides().shell_primary_count;
    output::Boolean(d,"incoming_solid_erosion",incoming);
    output::Boolean(d,"final_solid_erosion",incoming&&c.pre_internal!=0);
    output::Boolean(d,"owners_complete",c.solid_order_unresolved==0&&c.shell_owner_unresolved==0&&c.no_support==0);
    output::Boolean(d,"post_gapm_source_ready",false);
    output::Boolean(d,"internal_volume_observations_available",false);
    output::Integer(d,"mixed_construction_peak_reservation",source.forecast().peak_bytes);
    output::Integer(d,"query_census_extra_reservation",ExtraCap);
    output::Integer(d,"complete_qualification_reservation",total);
    for(std::size_t i=0;i<c.examples;++i)
        output::Integer(d,("unresolved_primary_ordinal_"+std::to_string(i)).c_str(),c.unresolved_primary_ids[i]);
    output::WriteJson(directory/"support.json",d);
    const auto bytes=output::ReadBounded(directory/"support.json",ExportCap);
    output::Document receipt;receipt.SetObject();
    output::String(receipt,"schema","robo_dyna.source_query_receipt.v1");
    output::String(receipt,"sha256",output::Sha256(bytes));
    output::Integer(receipt,"bytes",bytes.size());
    output::WriteJson(directory/"manifest.json",receipt);
}
}
TEST(PostGapmSupportActual, CompleteSharedContextReportsEverySupportAndInternalPhase) {
    const auto& initial=mi::test::ActualInitialSource();
    const auto mixed_forecast=mi::MixedInterfaceSource::Preflight(initial);
    const auto& input=initial.geometry();
    const auto& selected=initial.selection();
    ASSERT_EQ(selected.data().sources.at(0).block.keyword,"*CONTACT_AUTOMATIC_SINGLE_SURFACE");
    const auto& context=initial.context();
    const auto f=SelectedShellMainSource::Preflight(context,selected);
    // Conservatively keep the full old mixed construction peak while adding
    // only the public partitions actually used by these source queries.
    const auto work=f.part_operands+f.face_keys+f.source_metadata+
        detail::SupportQueryBytes(input.nodes.size(),input.shells.size())+
        detail::SolidSupportQueryBytes(input.nodes.size(),input.solids.size())+ExportCap;
    ASSERT_LE(work,ExtraCap);
    ASSERT_LE(mixed_forecast.peak_bytes,(std::size_t{10}<<30)-ExtraCap);
    const auto total=mixed_forecast.peak_bytes+ExtraCap;
    const auto mixed=mi::MixedInterfaceSource::Prepare(initial);
    ASSERT_EQ(mixed.report.status,mi::Status::Ready) << mixed.report.reason;
    ASSERT_TRUE(mixed.source);
    const auto& source=*mixed.source;
    const auto* combine_path=std::getenv("ROBO_SELF_CONTACT_COMBINE_MEMBER");
    ASSERT_TRUE(combine_path);
    const auto combine=output::ReadBounded(combine_path,modelio::self_contact::Limits{}.combine_member_bytes);
    const bool grouping=detail::GroupingContext(selected,combine);
    const auto packed=detail::PackShells(context,input,{});
    const auto shells=detail::PrepareSupportQueries(input,packed);
    const auto& flags=initial.emitted_solid_flags();
    const auto solids=detail::PrepareSolidSupportQueries(input,{flags.data(),flags.size()});
    Census c;
    for(std::size_t p=0;p<source.sides().primary_count;++p) {
        auto main=source.sides().mains[p];
        const auto solid=detail::QuerySolidSupport(input,solids,main);
        const auto count=solid.matches.size();
        ++c.matches[std::min<std::size_t>(count,3)];
        const bool unresolved=solid.state==detail::SolidSupportState::NeedsNativeReaderOrder;
        // Native >2 branch always retains a second positive support word,
        // although which owner it names is unavailable without source order.
        const bool internal=unresolved||solid.second!=SIZE_MAX;
        c.pre_internal+=internal;
        if(unresolved) {
            ++c.solid_order_unresolved;
            if(c.examples<c.unresolved_primary_ids.size())c.unresolved_primary_ids[c.examples++]=p;
        } else if(solid.state==detail::SolidSupportState::Resolved && solid.exterior_orientation) {
            n::NativeExteriorMainGeometryInput geometry;
            geometry.layout=main.nodes[2]==main.nodes[3]?n::ShellLayout::Triangle3:n::ShellLayout::Quad4;
            for(unsigned k=0;k<4;++k)geometry.face[k]=input.nodes[main.nodes[k]].native_position;
            for(unsigned k=0;k<8;++k)geometry.solid_raw[k]=input.nodes[input.solids[solid.first].nodes[k]].native_position;
            n::NativeExteriorMainGeometryResult value;
            ASSERT_EQ(n::EvaluateNativeExteriorMainGeometry(geometry,&value),n::CoefficientStatus::Ok) << p;
            c.negative_first_volumes+=value.signed_volume<0.;
            c.zero_first_volumes+=value.signed_volume==0.;
            c.primary_reversals+=value.reversed;
            const auto before=main;
            for(unsigned k=0;k<4;++k)main.nodes[k]=before.nodes[value.source_corner[k]];
            // IC>=2 never enters this helper: no hypothetical exterior
            // projection/volume is advertised as an internal-branch observation.
        }
        const auto shell=detail::QuerySupport(input,packed,shells,main,grouping);
        if(!shell.winners.empty()) {
            ++c.shell_owners;
            c.shell_owner_unresolved+=shell.owner==SIZE_MAX;
        } else if(count) {
            ++c.solid_owners;
            c.final_internal+=internal;
        } else ++c.no_support;
    }
    Write(c,source,total); // Keep complete diagnostics even if source closure fails.
    EXPECT_EQ(c.matches[0]+c.matches[1]+c.matches[2]+c.matches[3],source.sides().primary_count);
    EXPECT_EQ(c.shell_owners+c.solid_owners+c.no_support,source.sides().primary_count);
    EXPECT_EQ(c.no_support,0u);
    EXPECT_LE(c.final_internal,c.pre_internal);
    RecordProperty("pre_shell_internal_count",std::to_string(c.pre_internal));
    RecordProperty("final_internal_count",std::to_string(c.final_internal));
    RecordProperty("unresolved_solid_order",std::to_string(c.solid_order_unresolved));
    RecordProperty("scope","source queries only; unresolved classes retained and not promoted to source/runtime readiness");
}
}
