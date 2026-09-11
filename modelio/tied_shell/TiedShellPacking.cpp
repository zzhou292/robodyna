#include "TiedShellPacking.h"
#include "packing/Internal.h"

namespace crash::modelio::tied_shell {
struct TiedShellPacking::Storage {
    TiedShellDeclaration declaration;
    PackingData data;
    Storage(const TiedShellDeclaration& input, PackingData prepared)
        : declaration(input), data(std::move(prepared)) {}
};
std::size_t TiedShellPacking::Forecast(const TiedShellDeclaration& input, PackingLimits limits) {
    return packing_detail::Preflight(input.canonical().data(), input.data(), limits);
}
TiedShellPacking TiedShellPacking::Prepare(const TiedShellDeclaration& input, PackingLimits limits) {
    auto staged = packing_detail::Build(input.canonical().data(), input.data(), limits);
    return TiedShellPacking(std::make_shared<const Storage>(input, std::move(staged)));
}
const TiedShellDeclaration& TiedShellPacking::declaration() const noexcept {
    return storage_->declaration;
}
const PackingData& TiedShellPacking::data() const noexcept { return storage_->data; }
} // namespace crash::modelio::tied_shell
