#include "ContactStorage.h"
#include "MovingContactSource.h"
namespace crash::cases::native_scene {
namespace n=tlfea::contact::radioss_type25;
namespace cd=contact_detail;
struct ContactSource::Data : cd::ContactStorage {
    using ContactStorage::ContactStorage;
    n::FixedMainSource source;
};
struct MovingContactSource::Data : cd::ContactStorage {
    using ContactStorage::ContactStorage;
    n::MovingMainSource source;
};
ContactSource ContactSource::Prepare(const PhysicalSource& physical,ContactIdentity identity,ContactLimits limits) {
    const auto plan=cd::PlanContact(physical,identity,limits,cd::MainMotion::FixedWall,sizeof(Data));
    auto out=std::make_shared<Data>(physical);
    cd::BuildContact(*out,out->source,identity,limits,plan);
    return ContactSource(std::move(out));
}
MovingContactSource MovingContactSource::Prepare(const PhysicalSource& physical,ContactIdentity identity,ContactLimits limits) {
    const auto plan=cd::PlanContact(physical,identity,limits,cd::MainMotion::MovingShells,sizeof(Data));
    auto out=std::make_shared<Data>(physical);
    cd::BuildContact(*out,out->source,identity,limits,plan);
    if(physical.declared().data().rigid_patch)
        out->config.response_mass=n::ResponseMassPolicy::AcceptedOwnerCoefficients;
    out->source.starter=out->topology;
    out->source.activation={0,0,1,2,1,n::normal_activation::FreeRosterPolicy::FreshComplete};
    return MovingContactSource(std::move(out));
}
const PhysicalSource& ContactSource::physical_source() const noexcept{return data_->physical;}
const n::FixedMainSource& ContactSource::source() const noexcept{return data_->source;}
const n::TransactionConfig& ContactSource::config() const noexcept{return data_->config;}
const ContactForecast& ContactSource::forecast() const noexcept{return data_->forecast;}
n::search_startup::Initialization ContactSource::preprocessing() const noexcept{return data_->preprocessing;}
const PhysicalSource& MovingContactSource::physical_source() const noexcept{return data_->physical;}
const n::MovingMainSource& MovingContactSource::source() const noexcept{return data_->source;}
const n::TransactionConfig& MovingContactSource::config() const noexcept{return data_->config;}
const ContactForecast& MovingContactSource::forecast() const noexcept{return data_->forecast;}
n::search_startup::Initialization MovingContactSource::preprocessing() const noexcept{return data_->preprocessing;}
}
