#include "ReadInternal.h"

namespace crash::modelio::type13 {
SourceType13 SourceType13::Read(const std::filesystem::path& path,const ArtifactIdentity& identity,ReadLimits limits) {
    reader::Preflight(identity,limits);return ReadBytes(output::ReadBounded(path,identity.bytes),identity,limits);
}
SourceType13 SourceType13::ReadBytes(const std::string& bytes,const ArtifactIdentity& identity,ReadLimits limits) {
    const auto budget=reader::Preflight(identity,limits);
    reader::Require(bytes.size()==identity.bytes&&output::Sha256(bytes)==identity.sha256,"TYPE13 declaration authentication failed");
    output::Document document;
    document.Parse<rapidjson::kParseFullPrecisionFlag|rapidjson::kParseIterativeFlag|rapidjson::kParseValidateEncodingFlag>(bytes.data(),bytes.size());
    reader::Require(!document.HasParseError()&&document.IsObject(),"Invalid TYPE13 source declaration JSON");
    reader::UniqueKeys(document);reader::ReadScope(document);
    auto data=std::make_shared<Data>();data->identity=identity;data->authenticated_bytes=bytes;
    data->startup_budget_bytes=budget;
    data->canonical_manifest_sha256=reader::Text(reader::Member(document,"source"),"canonical_manifest_sha256");
    reader::ReadProperty(document,*data);reader::ReadGeometry(document,limits,*data);
    data->owned_payload_bytes=reader::OwnedPayload(*data,limits.host_bytes);
    reader::Require(data->owned_payload_bytes<=budget,"TYPE13 owned payload exceeded its startup forecast");
    return SourceType13(std::move(data));
}
const Data& SourceType13::data() const {
    reader::Require(bool(data_),"TYPE13 source input was moved from");return *data_;
}
}
