#include "Internal.h"
#include "output/BoundedArrayIO.h"
namespace crash::modelio::solid_control_packets {
struct NativePacketSource::Data {
    Data(const solid_source::VehicleSolidSource& s,const solid_control::EffectiveSource& c):solids(s),controls(c){}
    solid_source::VehicleSolidSource solids;
    solid_control::EffectiveSource controls;
    Artifact artifact;
    Forecast forecast;
    detail::Values values;
    std::size_t owned_bytes=0;
};
NativePacketSource NativePacketSource::Prepare(const solid_source::VehicleSolidSource& solids,
    const solid_control::EffectiveSource& controls,const Artifact& artifact,Limits limits) {
    using output::Require;
    const auto forecast=detail::Preflight(artifact.bytes,limits);
    output::arrays::CheckHash(artifact.sha256);
    Require(artifact.path.native().size()<=4096,"Native packet artifact path exceeds limit");
    Require(!artifact.case_profile.empty()&&artifact.case_profile.size()<=128,"Invalid expected native case profile");
    const auto& a=solids.canonical().data().inputs;const auto& b=controls.direct().canonical().data().inputs;
    Require(a.canonical_manifest.sha256==b.canonical_manifest.sha256&&a.source_member.sha256==b.source_member.sha256,
        "Solid packet controls and geometry have different original source authority");
    const auto bytes=output::ReadBounded(artifact.path,artifact.bytes);
    Require(bytes.size()==artifact.bytes&&output::Sha256(bytes)==artifact.sha256,"Native packet artifact identity differs");
    auto next=std::make_shared<Data>(solids,controls);next->artifact=artifact;next->forecast=forecast;
    next->values=detail::Read(bytes,artifact,limits);detail::Bind(next->values,solids.data(),controls.data());
    next->owned_bytes=sizeof(NativePacketSource)+sizeof(Data)+64+detail::Owned(next->values)-sizeof(detail::Values)+
        next->artifact.path.native().capacity()*sizeof(std::filesystem::path::value_type)+
        next->artifact.sha256.capacity()+next->artifact.case_profile.capacity()+3;
    Require(next->owned_bytes<=limits.retained_bytes,"Complete native packet owned payload exceeds cap");
    return NativePacketSource(std::move(next));
}
control::Input NativePacketSource::InputFor(std::uint64_t instance)const{return detail::View(data_->values,instance);}
const Artifact& NativePacketSource::artifact()const noexcept{return data_->artifact;}
const Forecast& NativePacketSource::forecast()const noexcept{return data_->forecast;}
std::size_t NativePacketSource::controlled_count()const noexcept{return data_->values.controlled_count;}
std::size_t NativePacketSource::owned_payload_bytes()const noexcept{return data_->owned_bytes;}
} // namespace crash::modelio::solid_control_packets
