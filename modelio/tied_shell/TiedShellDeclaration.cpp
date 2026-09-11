#include "Internal.h"

namespace crash::modelio::tied_shell {
struct TiedShellDeclaration::Storage {
    explicit Storage(const source::CanonicalSource& value) : canonical(value) {}
    source::CanonicalSource canonical;
    Data data;
};
namespace detail {
Data Build(const source::CanonicalData& source, const std::string& member, Limits limits) {
    const auto budget = Preflight(source, limits);
    Require(member.size() == source.inputs.source_member.bytes &&
            output::Sha256(member) == source.inputs.source_member.sha256,
            "Tied original member authentication failed");
    Document scope, canonical;
    constexpr unsigned flags = rapidjson::kParseFullPrecisionFlag | rapidjson::kParseIterativeFlag |
                               rapidjson::kParseValidateEncodingFlag;
    scope.Parse<flags>(source.scope_bytes.data(), source.scope_bytes.size());
    canonical.Parse<flags>(source.canonical_bytes.data(), source.canonical_bytes.size());
    Require(!scope.HasParseError() && scope.IsObject() && !canonical.HasParseError() && canonical.IsObject(),
            "Invalid authenticated tied source metadata");
    UniqueKeys(scope);
    UniqueKeys(canonical);
    auto draft = Declarations(source, scope, canonical, limits);
    Geometry(source, scope, draft, limits);
    Constraints(source, scope, canonical, draft, limits);
    ReadSources(draft, member, limits);
    ValidateCards(draft, limits);
    draft.data.startup_budget_bytes = budget;
    draft.data.owned_payload_bytes = OwnedPayload(draft.data, limits.host_bytes);
    return std::move(draft.data);
}
}
TiedShellDeclaration TiedShellDeclaration::Prepare(const source::CanonicalSource& source,
                                                  const std::string& member, Limits limits) {
    auto data = detail::Build(source.data(), member, limits);
    auto next = std::make_shared<Storage>(source);
    next->data = std::move(data);
    return TiedShellDeclaration(std::move(next));
}
std::size_t TiedShellDeclaration::Forecast(const source::CanonicalSource& source, Limits limits) {
    return detail::Preflight(source.data(), limits);
}
const source::CanonicalSource& TiedShellDeclaration::canonical() const noexcept { return storage_->canonical; }
const Data& TiedShellDeclaration::data() const noexcept { return storage_->data; }
} // namespace crash::modelio::tied_shell
