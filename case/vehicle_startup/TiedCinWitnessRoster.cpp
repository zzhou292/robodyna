#include "tied_cin_witness/Internal.h"
#include "modelio/vehicle_sections/VehicleSectionResolution.h"

namespace crash::cases::vehicle_startup {
struct TiedCinWitnessRoster::Data {
    TiedCinAttachments attachments;
    VehicleShellBinding binding;
    TiedCinWitnessData roster;
    TiedCinWitnessForecast forecast;
    Data(const TiedCinAttachments& a,const VehicleShellBinding& b) : attachments(a),binding(b) {}
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
const TiedCinAttachments& TiedCinWitnessRoster::attachments() const noexcept { return data_->attachments; }
const VehicleShellBinding& TiedCinWitnessRoster::binding() const noexcept { return data_->binding; }
const TiedCinWitnessData& TiedCinWitnessRoster::data() const noexcept { return data_->roster; }
const TiedCinWitnessForecast& TiedCinWitnessRoster::forecast() const noexcept { return data_->forecast; }
bool TiedCinWitnessRoster::runtime_mappable() const noexcept {
    const auto& counts=data_->roster.counts;
    return !counts.missing_domain_slots && !counts.rows_without_shell_witness && counts.maximum_per_row<=4;
}
}
