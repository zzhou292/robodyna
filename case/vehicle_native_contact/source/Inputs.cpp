#include "Internal.h"
#include "MemberStorage.h"
#include "modelio/physical_scope/PhysicalScope.h"
#include "modelio/solid_control/DirectSource.h"
#include "modelio/solid_control/EffectiveSource.h"
#include "modelio/self_contact/OriginalSelection.h"
namespace crash::cases::vehicle_native_contact::source::detail {
modelio::native_spring_ids::ImportMembers Members::Input()const {
    modelio::native_spring_ids::ImportMembers result;
    result.profile=modelio::native_spring_ids::Profile::DirectKeywordR14FreshRadiossPoSortById;
    result.entry_member="combine.key";
    result.members={{"yaris-coarse-v1l.key",vehicle},{"set-yaris-coarse-v1l.key",auxiliary},
        {"combine.key",combine},{"wall.key",wall}};
    return result;
}
std::size_t Members::bytes()const {
    std::size_t result=0;for(const auto* value:{&vehicle,&auxiliary,&combine,&wall})result=Add(result,Add(value->capacity(),1));
    return result;
}
std::shared_ptr<const Inputs> PrepareInputs(const vehicle_run::OriginalPaths& paths,
    const modelio::solid_control_packets::Artifact& artifact,Limits limits,Forecast& forecast) {
    const Limits hard;
    output::Require(limits.host_bytes&&limits.host_bytes<=hard.host_bytes&&limits.member_bytes&&
        limits.member_bytes<=hard.member_bytes,"Invalid native V6 source limits");
    for(const auto* path:{&paths.canonical,&paths.scope,&paths.member,&paths.declarations,&paths.glass_resolution,
            &paths.type13,&paths.auxiliary_member,&paths.original_wall_member,&paths.wall_manifest,&paths.self_contact_combine_member})
        output::Require(!path->empty()&&path->native().size()<=4096,"Native V6 source requires complete bounded explicit paths");
    output::Require(artifact.case_profile=="native_v6_raw8_heph_explicit_cin28"&&artifact.bytes&&
        artifact.bytes<=modelio::solid_control_packets::Limits{}.file_bytes,
        "Native V6 source requires the authenticated raw8 HEPH/CIN28 packet artifact");
    constexpr std::size_t maximum_members=42846753+44991+(64u<<10)+10604;
    output::Require(maximum_members<=limits.member_bytes&&maximum_members<=limits.host_bytes,
        "Original four-member source closure exceeds byte cap before reads");
    forecast.metadata_bytes=sizeof(Inputs)+sizeof(OriginalSources)+sizeof(Forecast)+65536;
    forecast.member_read_compaction_peak=ReadAndCompactPeak(limits.member_bytes,42846753);
    Admit(forecast,forecast.input_peak,0,forecast.member_read_compaction_peak,limits,"input_graph");
    namespace io=vehicle_run::detail;
    Members members;
    members.vehicle=CompactMember(io::ReadOriginal(paths.member,42846753,"67208317e6c8eb1dd43b80001508915ccaace7bc0a745e1aa5a3b33f394df301"),42846753);
    members.auxiliary=CompactMember(io::ReadOriginal(paths.auxiliary_member,44991,"b93d5370a899f6f70299ea61cd55142c1f8b765b8ab7f9ac979d078486028929"),44991);
    members.combine=CompactMember(output::ReadBounded(paths.self_contact_combine_member,64u<<10),64u<<10);
    members.wall=CompactMember(io::ReadOriginal(paths.original_wall_member,10604,"ef02a4701b37d27cec81b1f9a02ab555f55ac61f68b070e8b0c18dc23b1d5155"),10604);
    forecast.member_storage_bytes=members.bytes();
    output::Require(forecast.member_storage_bytes<=limits.member_bytes,"Owned original member capacities exceed cap");
    // Existing source readers enforce their own inclusive caps. Charge the
    // simultaneously retained stages conservatively before entering the chain.
    const auto reader_bound=Add(output::full_shell::source::SourceLimits{}.host_bytes,
        Add(modelio::vehicle::Limits{}.host_bytes,Add(modelio::physical_scope::Limits{}.host_bytes,
        modelio::solid_source::Limits::ExtendedSolids().host_bytes)));
    Admit(forecast,forecast.input_peak,0,reader_bound,limits,"input_graph");
    const auto canonical=io::ReadCanonical(paths,members.vehicle);
    const auto plan=modelio::vehicle::VehicleSourcePlan::Read(canonical,paths.declarations,
        {3648589,"a96bc12b9c8467253da0898565c7875ad80f58f963b45d1dc405f5dddab76b1d"});
    const auto rigid=modelio::physical_scope::rigid::RigidPartSource::Prepare(plan,members.vehicle);
    const auto masses=modelio::physical_scope::rigid::point_mass::Source::Prepare(rigid);
    const auto tied=modelio::tied_shell::TiedShellDeclaration::Prepare(canonical,members.vehicle);
    const auto type13=modelio::type13::SourceType13::Read(paths.type13,
        {5150841,"c15fc2096317ac0206397ac50776f8456ddd23495e0c65aeee98e093ebd0b1b1"});
    const auto solids=modelio::solid_source::VehicleSolidSource::Prepare(canonical,members.vehicle,
        modelio::solid_source::Policy::NativeConvertedSupportsV6,modelio::solid_source::Limits::ExtendedSolids());
    const auto beams=modelio::beam18::Source::Prepare(canonical,members.vehicle,
        modelio::beam18::Policy::OriginalCircularFourPointLaw44V1);
    const auto scope=modelio::physical_scope::PhysicalScope::PrepareVehicleSupports(masses,tied,type13,solids,beams);
    const auto domain=modelio::physical_domain::VehiclePhysicalDomain::Prepare(scope,
        modelio::physical_domain::Policy::RetainedShellAssembliesNativeSupportsV6);
    Admit(forecast,forecast.input_peak,domain.forecast().total_bytes,
        modelio::vehicle::ResolutionLimits::CompleteRigidOverlay().host_bytes,limits,"input_graph");
    const auto resolution=io::Resolve(plan,paths,members.vehicle);
    const auto import_forecast=modelio::native_spring_ids::ImportContext::Preflight(canonical,members.Input());
    forecast.packet_authority_reservation=Add(import_forecast.total_bytes,
        Add(modelio::solid_control::DirectLimits{}.retained_bytes,
        Add(modelio::solid_control::Limits{}.retained_bytes,modelio::solid_control_packets::Limits{}.startup_bytes)));
    Admit(forecast,forecast.input_peak,domain.forecast().total_bytes,resolution.startup_budget_bytes(),limits,"input_graph");
    const auto imported=modelio::native_spring_ids::ImportContext::Prepare(canonical,members.Input());
    output::Require(imported.data().diagnostic.status==modelio::native_spring_ids::Readiness::Ready,
        "Native V6 four-member import context is unavailable");
    const auto direct=modelio::solid_control::DirectSource::Prepare(canonical,members.Input());
    const auto effective=modelio::solid_control::EffectiveSource::Prepare(direct,imported,members.Input());
    output::Require(effective.report.status==modelio::solid_control::Status::Ready&&effective.source,
        effective.report.reason.c_str());
    forecast.packet_authority_reservation=Add(Add(imported.data().forecast.total_bytes,direct.owned_payload_bytes()),
        Add(effective.source->owned_payload_bytes(),modelio::solid_control_packets::Limits{}.startup_bytes));
    Admit(forecast,forecast.input_peak,domain.forecast().total_bytes,resolution.startup_budget_bytes(),limits,"input_graph");
    const auto packets=modelio::solid_control_packets::NativePacketSource::Prepare(solids,*effective.source,artifact);
    // Keep the conservative packet startup reservation in later coexistence
    // budgets: it also covers retained direct/import authority, without guessing
    // private allocations or claiming a cross-graph canonical discount.
    return std::make_shared<const Inputs>(Inputs{std::move(members),canonical,resolution,domain,packets});
}
} // namespace crash::cases::vehicle_native_contact::source::detail

namespace crash::cases::vehicle_native_contact::source {
struct OriginalSources::Data {
    std::shared_ptr<const detail::Inputs> backing;
    Owner owner;Self self;Wall wall;Controls controls;Forecast forecast;
};
OriginalSources OriginalSources::Prepare(const vehicle_run::OriginalPaths& paths,
    const modelio::solid_control_packets::Artifact& artifact,Limits limits) {
    Forecast forecast;
    const auto backing=detail::PrepareInputs(paths,artifact,limits,forecast);
    const auto owner=detail::PrepareOwner(*backing,paths,limits,forecast);
    const auto contact=detail::PrepareContact(*backing,owner,limits,forecast);
    const vehicle_native_contact::detail::SourceInputs inputs{owner,contact.self,contact.wall,contact.controls};
    forecast.sources=vehicle_native_contact::detail::AdmitSources(inputs,limits.host_bytes);
    detail::Admit(forecast,forecast.contact_peak,0,forecast.sources.construction_peak,limits,"input_graph");
    forecast.retained_bytes=detail::Add(forecast.sources.retained_bytes,detail::Extras(forecast));
    return OriginalSources(std::make_shared<const Data>(Data{backing,owner,contact.self,contact.wall,contact.controls,forecast}));
}
const Owner& OriginalSources::owner()const noexcept{return data_->owner;}
const Self& OriginalSources::self()const noexcept{return data_->self;}
const Wall& OriginalSources::wall()const noexcept{return data_->wall;}
const Controls& OriginalSources::controls()const noexcept{return data_->controls;}
const modelio::solid_control_packets::NativePacketSource& OriginalSources::packets()const noexcept{return data_->backing->packets;}
vehicle_native_contact::detail::SourceInputs OriginalSources::inputs()const noexcept{return {owner(),self(),wall(),controls()};}
const Forecast& OriginalSources::forecast()const noexcept{return data_->forecast;}
} // namespace crash::cases::vehicle_native_contact::source
