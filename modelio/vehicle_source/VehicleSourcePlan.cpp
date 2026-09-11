#include "Internal.h"
#include <algorithm>

namespace crash::modelio::vehicle {
struct VehicleSourcePlan::Data {
    explicit Data(const source::CanonicalSource& value):canonical(value) {}
    source::CanonicalSource canonical;
    assembly::ArtifactIdentity identity;
    detail::Declarations declarations;
    detail::Geometry geometry;
    std::size_t budget=0;
};
VehicleSourcePlan VehicleSourcePlan::Read(const source::CanonicalSource& source,const std::filesystem::path& path,
                                        const assembly::ArtifactIdentity& identity,Limits limits) {
    detail::Preflight(source.data(),identity,limits);
    return ReadBytes(source,output::ReadBounded(path,identity.bytes),identity,limits);
}
VehicleSourcePlan VehicleSourcePlan::ReadBytes(const source::CanonicalSource& source,const std::string& bytes,
                                             const assembly::ArtifactIdentity& identity,Limits limits) {
    const auto budget=detail::Preflight(source.data(),identity,limits);
    output::Require(bytes.size()==identity.bytes&&output::Sha256(bytes)==identity.sha256,
                    "Vehicle declaration content authentication failed");
    output::Document doc;
    doc.Parse<rapidjson::kParseFullPrecisionFlag|rapidjson::kParseIterativeFlag|rapidjson::kParseValidateEncodingFlag>(bytes.data(),bytes.size());
    output::Require(!doc.HasParseError()&&doc.IsObject(),"Invalid vehicle declaration JSON");
    detail::UniqueKeys(doc);detail::CheckAuthority(source.data(),doc);
    auto next=std::make_shared<Data>(source);next->identity=identity;next->budget=budget;
    next->declarations=detail::ReadDeclarations(source.data(),doc,limits);
    next->geometry=detail::ReadGeometry(source.data(),next->declarations.parts);
    const auto& count=detail::Member(doc,"counts");const auto& c=next->geometry.counts;
    output::Require(detail::Unsigned(count,"parts")==c.parts&&detail::Unsigned(count,"shells")==c.parents&&
        detail::Unsigned(count,"supported_parts")==c.supported_parts&&
        detail::Unsigned(count,"supported_shells")==c.supported_parents,"Vehicle declared coverage disagrees");
    return VehicleSourcePlan(std::move(next));
}
const source::CanonicalSource& VehicleSourcePlan::canonical() const noexcept {return data_->canonical;}
const assembly::ArtifactIdentity& VehicleSourcePlan::identity() const noexcept {return data_->identity;}
const Counts& VehicleSourcePlan::counts() const noexcept {return data_->geometry.counts;}
const std::vector<PartDisposition>& VehicleSourcePlan::parts() const noexcept {return data_->declarations.parts;}
const std::vector<ParentIndex>& VehicleSourcePlan::parents() const noexcept {return data_->geometry.parents;}
const std::vector<std::uint32_t>& VehicleSourcePlan::canonical_nodes() const noexcept {return data_->geometry.nodes;}
const assembly::Material* VehicleSourcePlan::material(std::size_t p) const noexcept {
    const auto& d=data_->declarations;
    return p<d.materials.size()&&d.materials[p]!=SIZE_MAX?&d.typed.materials[d.materials[p]]:nullptr;
}
const assembly::Section* VehicleSourcePlan::section(std::size_t p) const noexcept {
    const auto& d=data_->declarations;
    return p<d.sections.size()&&d.sections[p]!=SIZE_MAX?&d.typed.sections[d.sections[p]]:nullptr;
}
const std::vector<assembly::Curve>& VehicleSourcePlan::curves() const noexcept {return data_->declarations.typed.curves;}
std::size_t VehicleSourcePlan::startup_budget_bytes() const noexcept {return data_->budget;}
} // namespace crash::modelio::vehicle
