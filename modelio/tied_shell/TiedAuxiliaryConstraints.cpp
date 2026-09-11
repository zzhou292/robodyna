#include "auxiliary/Internal.h"

namespace crash::modelio::tied_shell {
struct TiedAuxiliaryConstraints::Storage {
    explicit Storage(const TiedShellDeclaration& value) : declaration(value) {}
    TiedShellDeclaration declaration;
    AuxiliaryData data;
};
namespace auxiliary_detail {
AuxiliaryData Build(const source::CanonicalData& source, const Data& declaration,
                   const std::string& member, OriginalWallPolicy policy, AuxiliaryLimits limits) {
    const auto budget = Preflight(source, declaration, member.size(), limits);
    Require(policy == OriginalWallPolicy::RetainUnresolved || policy == OriginalWallPolicy::ReplaceWithMeshWall,
            "Invalid original wall policy");
    Document canonical;
    constexpr unsigned flags = rapidjson::kParseFullPrecisionFlag | rapidjson::kParseIterativeFlag |
                               rapidjson::kParseValidateEncodingFlag;
    canonical.Parse<flags>(source.canonical_bytes.data(), source.canonical_bytes.size());
    Require(!canonical.HasParseError() && canonical.IsObject(), "Invalid authenticated auxiliary metadata");
    UniqueKeys(canonical);
    const auto& files = Member(canonical, "source_files");
    Require(files.IsObject() && files.MemberCount() <= limits.blocks, "Auxiliary source file count exceeded");
    const auto& file = Member(files, MemberName);
    const auto hash = Text(file, "sha256");
    output::arrays::CheckHash(hash);
    Require(output::Sha256(member) == hash, "Auxiliary complete member authentication failed");
    Draft draft;
    draft.data.filename = MemberName;
    draft.data.member_sha256 = hash;
    draft.data.member_bytes = member.size();
    draft.data.sources = Sources(file, member, limits);
    Groups(draft, declaration, limits);
    Nodes(draft, source, declaration, limits);
    Boundary(draft, declaration, files, policy, limits);
    draft.data.startup_budget_bytes = budget;
    draft.data.owned_payload_bytes = OwnedPayload(draft.data, limits.host_bytes);
    return std::move(draft.data);
}
}
TiedAuxiliaryConstraints TiedAuxiliaryConstraints::Prepare(const TiedShellDeclaration& declaration,
        const std::string& member, OriginalWallPolicy policy, AuxiliaryLimits limits) {
    auto data = auxiliary_detail::Build(declaration.canonical().data(), declaration.data(), member, policy, limits);
    auto next = std::make_shared<Storage>(declaration);
    next->data = std::move(data);
    return TiedAuxiliaryConstraints(std::move(next));
}
std::size_t TiedAuxiliaryConstraints::Forecast(const TiedShellDeclaration& declaration,
        std::size_t member_bytes, AuxiliaryLimits limits) {
    return auxiliary_detail::Preflight(declaration.canonical().data(), declaration.data(), member_bytes, limits);
}
const TiedShellDeclaration& TiedAuxiliaryConstraints::declaration() const noexcept { return storage_->declaration; }
const AuxiliaryData& TiedAuxiliaryConstraints::data() const noexcept { return storage_->data; }
} // namespace crash::modelio::tied_shell
