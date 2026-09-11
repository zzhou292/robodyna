#include "Internal.h"
#include "modelio/source_assembly/AuxiliarySourceCards.h"

namespace crash::modelio::solid_source::detail {
void ReadCurveData(const tied_shell::SourceEvidence& source, std::uint64_t id,
                   std::size_t count, double scale, std::vector<double>& x, std::vector<double>& y) {
    Require(source.block.keyword == "*DEFINE_CURVE" && source.cards.size() == count + 1 &&
        tied_shell::detail::CardId(source.cards[0].second, 0) == id,
        "Original solid curve identity or point count changed");
    const auto& header = source.cards[0].second;
    Require(Required(header, 1) == 0 && Required(header, 2) == 1 && Required(header, 3) == 1,
            "Original solid curve scales or options changed");
    Blank(header, 4, 8);
    const bool shared = !x.empty();
    Require(x.size() == y.size() && (!shared || x.size() == count), "Incomplete owned solid curve");
    if (!shared) { x.reserve(count); y.reserve(count); }
    for (std::size_t i = 0; i < count; ++i) {
        const auto& row = source.cards[i+1].second;
        Require(assembly::reader::auxiliary::BlankTail(row, 40), "Extra solid curve values");
        const double xi = Required(row, 0, 20), yi = Required(row, 1, 20) * scale;
        if (shared) Require(output::Bits(x[i]) == output::Bits(xi) && output::Bits(y[i]) == output::Bits(yi),
                            "Shared original solid curve differs between parts");
        else { x.push_back(xi); y.push_back(yi); }
    }
}
} // namespace crash::modelio::solid_source::detail
