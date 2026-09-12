#include "Internal.h"
namespace crash::modelio::beam18 {
struct Source::Storage {
    explicit Storage(const source::CanonicalSource& s):canonical(s){}
    source::CanonicalSource canonical;
    Data data;
    Forecast forecast;
};
Forecast Source::Preflight(const source::CanonicalSource& s,Policy policy,Limits limits) {
    return detail::Budget(s.data(),policy,limits);
}
Source Source::Prepare(const source::CanonicalSource& source,const std::string& member,Policy policy,Limits limits) {
    const auto forecast=Preflight(source,policy,limits);
    output::Require(member.size()==source.data().inputs.source_member.bytes &&
        output::Sha256(member)==source.data().inputs.source_member.sha256,"Beam18 original member authentication failed");
    auto next=std::make_shared<Storage>(source); next->forecast=forecast;next->data.policy=policy;
    detail::ReadDeclarations(source.data(),member,next->data,limits);
    detail::ReadGeometry(source.data(),member,next->data,limits);
    detail::ReadWorkingCards(source.data(),member,next->data);
    detail::PrepareReferences(next->data);
    const auto& d=next->data;
    output::Require(d.parts.size()==4 && d.rows.size()==142 && d.nodes.size()==147 &&
        d.canonical_endpoints.size()==146 && d.original_beams==4685 && d.outside_beams==4543,
        "Original beam18 source census changed");
    next->data.owned_payload_bytes=detail::OwnedPayload(d,limits);
    return Source(std::move(next));
}
const source::CanonicalSource& Source::canonical() const noexcept{return storage_->canonical;}
const Data& Source::data() const noexcept{return storage_->data;}
const Forecast& Source::forecast() const noexcept{return storage_->forecast;}
} // namespace crash::modelio::beam18
