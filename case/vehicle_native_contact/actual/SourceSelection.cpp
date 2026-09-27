#include "SourceSelection.h"
#include "../tests/ActualSources.h"
#include <cmath>
#include <cstdlib>
namespace crash::cases::vehicle_native_contact::test {
namespace {
std::string Required(const char* key) {
    const auto* value=std::getenv(key);
    output::Require(value&&*value,"Missing explicit native V6 source/run environment");
    return value;
}
vehicle_run::OriginalPaths Paths() {
    return {Required("ROBO_STATIC_CANONICAL"),Required("ROBO_STATIC_SCOPE"),Required("ROBO_STATIC_MEMBER"),
        Required("ROBO_VEHICLE_DECLARATIONS"),Required("ROBO_VEHICLE_GLASS_RESOLUTION"),
        Required("ROBO_DYNA_TYPE13_DECLARATION"),Required("ROBO_SELF_CONTACT_AUX_MEMBER"),
        Required("ROBO_TIED_WALL_MEMBER"),Required("ROBO_NATIVE_WALL_MANIFEST"),
        Required("ROBO_SELF_CONTACT_COMBINE_MEMBER")};
}
}
SourceSelection::SourceSelection(Config& config) {
    const auto* path=std::getenv("ROBO_NATIVE_SOLID_PACKETS");
    if(!path||!*path)return;
    const auto size=Required("ROBO_NATIVE_SOLID_PACKETS_BYTES");std::size_t used=0;
    output::Require(size.find_first_not_of("0123456789")==std::string::npos,"Packet byte count must be unsigned decimal");
    const auto bytes=std::stoull(size,&used);
    output::Require(used==size.size()&&bytes&&bytes<=modelio::solid_control_packets::Limits{}.file_bytes,
        "Packet byte count exceeds source admission");
    artifact_=modelio::solid_control_packets::Artifact{path,static_cast<std::size_t>(bytes),
        Required("ROBO_NATIVE_SOLID_PACKETS_SHA256"),"native_v6_raw8_heph_explicit_cin28"};
    const auto dt=Required("ROBO_NATIVE_FIXED_DT_S");used=0;
    const auto value=std::stod(dt,&used);
    output::Require(used==dt.size()&&std::isfinite(value)&&value>0,
        "Native V6 requires an explicit finite positive physical timestep");
    config.dynamics.startup.reserved_step_s=value;
}
detail::SourceInputs SourceSelection::Prepare() {
    if(!artifact_)return ActualSources();
    output::Require(!source_,"Native source selection cannot be prepared twice");
    source_=source::OriginalSources::Prepare(Paths(),*artifact_);
    return source_->inputs();
}
std::size_t SourceSelection::extra_retained_bytes()const noexcept {
    if(!source_)return 0;
    const auto& f=source_->forecast();return f.retained_bytes-f.sources.retained_bytes;
}
} // namespace crash::cases::vehicle_native_contact::test
