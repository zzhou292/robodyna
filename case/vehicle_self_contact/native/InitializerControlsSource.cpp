#include "initial_controls/Internal.h"
namespace crash::cases::vehicle_self_contact::native::initial_controls {
struct InitializerControlsSource::Data {
    explicit Data(const MainSource& value,const Wall* additional):main(value) {if(additional)wall.emplace(*additional);}
    MainSource main;std::optional<Wall> wall;
    n::TransactionConfig law;
    Controls controls,wall_controls;RawControls raw,wall_raw;GapScalars gaps;
    detail::Namespace names;Provenance provenance;Forecast forecast;
};
Forecast InitializerControlsSource::Preflight(const MainSource& main,const detail::ids::ImportMembers& members,
    const Wall* wall,Limits limits) {return detail::Budget(main,members,wall,limits);}
Preparation InitializerControlsSource::Prepare(const MainSource& main,const detail::ids::ImportMembers& members,
    const Wall* wall,Limits limits) {
    try {
        const auto forecast=Preflight(main,members,wall,limits);
        const auto imported=detail::ids::ImportContext::Prepare(main.mixed().initial().selection().canonical(),members,limits.import);
        detail::Require(imported.data().diagnostic.status==detail::ids::Readiness::Ready&&
            imported.data().source_digest==main.provenance().source_digest,
            "Initializer controls do not share the complete authenticated source import");
        auto data=std::make_shared<Data>(main,wall);data->forecast=forecast;
        data->raw=detail::ResolveOriginalControls(main.mixed().initial().selection().data());
        data->law=detail::ResolveSelfLaw(main.mixed().initial().selection().data(),main.provenance().units);
        // Explicit additional native declaration. HM_READ rawIGAP2 resolves to
        // IGAP1+FLAGREMN2; sourceIDEL0 defaults to no erosion for the fixed wall.
        if(wall)data->wall_raw={0,0,2,0,1,1,1,0.}; // Explicit added wall: sensor0, no finite stop in case horizon.
        data->gaps=detail::ResolveGaps(main.primary_owners(),main.gap_operands().bindings(),main.gap_operands().shells(),
            main.gap_profile(),main.gap_report());
        data->names=detail::ResolveNamespace(main,imported,members,wall,limits);
        detail::Require(detail::RetainedNamespace(data->names,forecast.output_values)+sizeof(Data)+8192<=forecast.output_values,
            "Initializer controls retained namespace exceeds admitted output storage");
        auto& p=data->provenance;p.source_digest=main.provenance().source_digest;p.main_digest=main.provenance().output_digest;
        if(wall)p.wall_digest=wall->digest();
        p.original_interfaces=data->names.interfaces.size()-(wall?1:0);p.declared_interfaces=wall?1:0;
        p.checked_blocks=data->names.checked_blocks;
        p.output_digest=detail::Digest(main,wall,data->raw,data->gaps,data->names,limits);
        return {{Status::Ready,"Authenticated initializer controls and complete interface namespace",{},0},InitializerControlsSource(std::move(data))};
    } catch(const detail::Failure& error) {return {error.report,std::nullopt};}
      catch(const std::bad_alloc&) {return {{Status::ResourceLimit,"Initializer controls allocation failed",{},0},std::nullopt};}
      catch(const std::exception& error) {return {{Status::InvalidInput,std::string(error.what()).substr(0,1024),{},0},std::nullopt};}
}
const MainSource& InitializerControlsSource::main()const noexcept{return data_->main;}
const Wall* InitializerControlsSource::wall()const noexcept{return data_->wall?&*data_->wall:nullptr;}
const Controls& InitializerControlsSource::controls()const noexcept{return data_->controls;}
const n::TransactionConfig& InitializerControlsSource::self_runtime_controls()const noexcept{return data_->law;}
const RawControls& InitializerControlsSource::raw_controls()const noexcept{return data_->raw;}
const Controls* InitializerControlsSource::wall_controls()const noexcept{return data_->wall?&data_->wall_controls:nullptr;}
const RawControls* InitializerControlsSource::wall_raw_controls()const noexcept{return data_->wall?&data_->wall_raw:nullptr;}
const GapScalars& InitializerControlsSource::gaps()const noexcept{return data_->gaps;}
tl::util::ConstView<Interface> InitializerControlsSource::interfaces()const noexcept{return {data_->names.interfaces.data(),data_->names.interfaces.size()};}
std::uint64_t InitializerControlsSource::self_interface_id()const noexcept{return data_->names.self;}
std::uint64_t InitializerControlsSource::wall_interface_id()const noexcept{return data_->names.wall;}
const PopulationRange& InitializerControlsSource::native_population()const noexcept{return data_->names.population;}
const Provenance& InitializerControlsSource::provenance()const noexcept{return data_->provenance;}
const Forecast& InitializerControlsSource::forecast()const noexcept{return data_->forecast;}
}
