#include "tied_cin_witness/Internal.h"
#include "modelio/vehicle_sections/VehicleSectionResolution.h"
#include <optional>
#include "lib_utils/BoundedArena.h"

namespace crash::cases::vehicle_startup {
struct TiedCinWitnessRoster::Data {
    std::optional<TiedCinAttachments> attachments;
    std::optional<VehicleShellBinding> binding;
    std::optional<tl::fea::ShellPhysicalBinding> physical;
    native_search::TiedCinAttachmentModel model;
    TiedCinWitnessData roster;
    TiedCinWitnessForecast forecast;
    Data(const TiedCinAttachments& a,const VehicleShellBinding& b) : attachments(a),binding(b),model(a.model()) {}
    Data(const native_search::TiedCinAttachmentModel& a,const tl::fea::ShellPhysicalBinding& b):physical(b),model(a){}
};
TiedCinWitnessForecast TiedCinWitnessRoster::Forecast(const TiedCinAttachments& cin,
        const VehicleShellBinding& binding,TiedCinWitnessLimits limits) {
    const auto& refs=binding.references();
    const auto& declaration=cin.post_kinchk().classification().context().auxiliary().declaration();
    output::Require(&refs.source().canonical().data()==&declaration.canonical().data(),
                    "CIN roster must retain the same authenticated canonical backing");
    output::Require(refs.resolution() && refs.resolution()->resolution_key().profile==
        modelio::vehicle::ResolutionProfile::OriginalRigidPartsV1 && !refs.counts().unresolved &&
        !refs.counts().rejected,"CIN roster needs the complete original shell binding");
    std::size_t retained=0;
    const auto add=[&](std::size_t bytes) {
        output::Require(bytes<=TiedCinWitnessLimits{}.host_bytes-retained,"CIN retained-source bound overflows");
        retained+=bytes;
    };
    add(refs.forecast().total_bytes);
    add(binding.shells().host_bytes());
    add(binding.forecast().fixed_bytes);
    add(cin.forecast().total_host_bytes);
    // These deliberately conservative existing bounds may include shared
    // source and retired scratch. No coincident byte count is subtracted.
    return cin_witness_detail::Budget(retained,binding.shells().node_count(),refs.rows().size(),
        cin.model().rows().count,sizeof(Data)+sizeof(TiedCinWitnessRoster)+64,limits);
}
TiedCinWitnessRoster TiedCinWitnessRoster::Prepare(const TiedCinAttachments& cin,
        const VehicleShellBinding& binding,TiedCinWitnessLimits limits) {
    const auto forecast=Forecast(cin,binding,limits);
    auto next=std::make_shared<Data>(cin,binding);
    next->forecast=forecast;
    next->roster=cin_witness_detail::Build(binding.shells(),binding.references().rows(),
                                        cin.model().rows(),*cin.model().domain(),limits);
    cin_witness_detail::CheckActualCapacity(next->roster,forecast,limits);
    return TiedCinWitnessRoster(std::move(next));
}
TiedCinWitnessForecast TiedCinWitnessRoster::ForecastPhysical(const native_search::TiedCinAttachmentModel& cin,
        const tl::fea::ShellPhysicalBinding& physical,TiedCinWitnessLimits limits) {
    output::Require(cin.prepared()&&!cin.explicitly_empty()&&physical.prepared()&&physical.catalog()&&physical.shells()&&
        cin.domain()&&physical.domain()&&cin.domain()->SharesStorage(*physical.domain()),
        "CIN physical witness source requires the same complete immutable node domain");
    const auto count=physical.catalog()->parent_count();
    output::Require(count==physical.shells()->qeph_count()+physical.shells()->t3_count()+physical.shells()->qbat_count(),
        "CIN physical witness source catalog is incomplete");
    tl::util::BoundedArenaLayout retained(limits.host_bytes);tl::util::ArenaRegion region;
    for(auto bytes:{physical.owned_payload_bytes(),cin.forecast().model_payload_bytes,cin.forecast().post_kinchk_payload_bytes,
                   cin.forecast().domain_payload_bytes})
        output::Require(retained.Append<std::byte>(bytes,region),"CIN physical source retention exceeds cap");
    auto out=cin_witness_detail::Budget(retained.bytes(),physical.shells()->node_count(),count,cin.rows().count,
        sizeof(Data)+sizeof(TiedCinWitnessRoster)+64,limits);
    output::Require(count<=(limits.host_bytes-out.total_host_bytes)/sizeof(ReferenceRow),"CIN physical parent metadata exceeds cap");
    const auto scratch=count*sizeof(ReferenceRow);out.incidence_scratch_bytes+=scratch;out.total_host_bytes+=scratch;
    return out;
}
TiedCinWitnessRoster TiedCinWitnessRoster::PreparePhysical(const native_search::TiedCinAttachmentModel& cin,
        const tl::fea::ShellPhysicalBinding& physical,TiedCinWitnessLimits limits) {
    const auto forecast=ForecastPhysical(cin,physical,limits);
    const auto* catalog=physical.catalog();std::vector<ReferenceRow> parents;parents.reserve(catalog->parent_count());
    for(std::size_t i=0;i<catalog->parent_count();++i) {
        const auto* source=catalog->parent(i);output::Require(source,"Missing prepared physical parent");
        ReferenceRow row;row.element_id=source->source_parent_id;row.part_id=source->source_part_id;
        row.reference_index=source->family_index;row.canonical_parent=UINT32_MAX; // No vehicle canonical index is claimed.
        row.status=ReferenceStatus::Success;
        row.family=source->family==tl::fea::ShellBindingFamily::Qeph?ReferenceFamily::Qeph:
            source->family==tl::fea::ShellBindingFamily::T3?ReferenceFamily::T3:
            source->family==tl::fea::ShellBindingFamily::Qbat?ReferenceFamily::Qbat:ReferenceFamily::None;
        output::Require(row.family!=ReferenceFamily::None,"Unsupported physical witness family");
        tl::fea::ShellSectionLaw law;
        output::Require(catalog->Law(source->family,source->family_index,&law),"Physical parent law is unavailable");
        row.role=law==tl::fea::ShellSectionLaw::RigidSkin?modelio::vehicle::SourceShellRole::OriginalRigidPart:
            modelio::vehicle::SourceShellRole::ConstitutiveShell;
        parents.push_back(row);
    }
    output::Require(parents.capacity()<=catalog->parent_count(),"Physical parent scratch exceeds preflight");
    auto next=std::make_shared<Data>(cin,physical);next->forecast=forecast;
    next->roster=cin_witness_detail::Build(*physical.shells(),parents,cin.rows(),*cin.domain(),limits);
    output::Require(next->roster.counts.declared_parent_witnesses==cin.rows().count,
        "Prepared CIN master identity is absent from the actual complete physical catalog");
    cin_witness_detail::CheckActualCapacity(next->roster,forecast,limits);
    return TiedCinWitnessRoster(std::move(next));
}
const TiedCinAttachments& TiedCinWitnessRoster::attachments() const {
    output::Require(bool(data_->attachments),"Physical CIN roster has no vehicle source wrapper");return *data_->attachments;
}
const VehicleShellBinding& TiedCinWitnessRoster::binding() const {
    output::Require(bool(data_->binding),"Physical CIN roster has no vehicle shell wrapper");return *data_->binding;
}
const native_search::TiedCinAttachmentModel& TiedCinWitnessRoster::model() const noexcept{return data_->model;}
const tl::fea::ShellBatchBinding& TiedCinWitnessRoster::shells() const noexcept {
    return data_->physical?*data_->physical->shells():data_->binding->shells();
}
const TiedCinWitnessData& TiedCinWitnessRoster::data() const noexcept { return data_->roster; }
const TiedCinWitnessForecast& TiedCinWitnessRoster::forecast() const noexcept { return data_->forecast; }
bool TiedCinWitnessRoster::runtime_mappable() const noexcept {
    const auto& counts=data_->roster.counts;
    return !counts.missing_domain_slots && !counts.rows_without_shell_witness && counts.maximum_per_row<=4;
}
}
