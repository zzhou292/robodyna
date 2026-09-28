#include "Declaration.h"
#include "Controls.h"
#include "Coverage.h"
#include "case/vehicle_self_contact/native/initial_controls/Values.h"
namespace crash::cases::vehicle_native_contact::activity {
struct Declaration::Data {
    Data(const tl::fea::type45::Model& value,native::Controls s,native::Controls w,Coverage c,Forecast f)
        :joints(value),self(s),wall(w),coverage(std::move(c)),forecast(f){}
    tl::fea::type45::Model joints;
    native::Controls self,wall;Coverage coverage;Forecast forecast;
};
Declaration Declaration::Prepare(const detail::SourceInputs& in,Limits limits) {
    output::Require(limits.metadata_bytes&&limits.metadata_bytes<=Limits{}.metadata_bytes&&
        limits.source_host_cap&&limits.source_host_cap<=Limits{}.source_host_cap,"Invalid activity source limits");
    (void)detail::AdmitSources(in,limits.source_host_cap);
    namespace controls=vehicle_self_contact::native::initial_controls;
    const auto& source=in.self.main_source().mixed().initial().selection().data();
    const auto resolved=controls::detail::ResolveOriginalControls(source);
    output::Require(resolved.reader_idel==in.controls.raw_controls().reader_idel&&in.controls.wall_raw_controls(),
        "Activity controls are not the authenticated original self and declared wall source");
    output::Require(in.self.snapshot().post_gapm,"Activity self source lacks post-GAPM provenance");
    const auto self=values::Self(resolved,in.self.snapshot().post_gapm->final_solid_erosion);
    const auto wall=values::Wall(*in.controls.wall_raw_controls(),in.wall.controls().solid_erosion);
    auto audited=coverage::Audit(in,limits);
    Forecast forecast;forecast.owned_metadata_bytes=sizeof(Declaration)+sizeof(Data)+64+4096;
    forecast.construction_workspace_bytes=limits.workspace_bytes;
    output::Require(forecast.owned_metadata_bytes<=limits.metadata_bytes,"Activity declaration metadata exceeds cap");
    return Declaration(std::make_shared<const Data>(in.owner.joints(),self,wall,std::move(audited),forecast));
}
const native::Controls& Declaration::self()const noexcept{return data_->self;}
const native::Controls& Declaration::wall()const noexcept{return data_->wall;}
const tl::fea::type45::Model& Declaration::joints()const noexcept{return data_->joints;}
const Coverage& Declaration::coverage()const noexcept{return data_->coverage;}
Forecast Declaration::forecast()const noexcept{return data_->forecast;}
}
