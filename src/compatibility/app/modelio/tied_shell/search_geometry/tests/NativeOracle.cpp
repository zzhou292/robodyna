#include "NativeOracle.h"
#include "../native/Packet.h"
#include <algorithm>
#include <map>
#include <set>

namespace crash::modelio::tied_shell::test {
NativeGeometryResult NativeGeometry(const std::array<SourceId,4>& master,
        const std::vector<std::array<SourceId,4>>& nodes,
        const std::vector<search_detail::RankedCoefficient>& coefficients,
        const std::vector<unsigned>& order) {
    output::Require(nodes.size() == coefficients.size() && nodes.size() == order.size() && nodes.size() <= 128,
                    "Invalid native search test packet extent");
    std::set<SourceId> ids(master.begin(), master.end());
    for (const auto& row : nodes) ids.insert(row.begin(), row.end());
    std::map<SourceId, int> native_node;
    for (auto id : ids) native_node.emplace(id, native_node.size()+1);
    std::array<int,4> native_master;
    for (unsigned i = 0; i < 4; ++i) native_master[i] = native_node.at(master[i]);
    std::vector<int> quad, triangle, qorder, torder, qsource, tsource;
    std::vector<double> geometry, modulus, part, element;
    for (const auto family : {SearchShellFamily::Q4, SearchShellFamily::T3}) {
        for (unsigned i = 0; i < nodes.size(); ++i) {
            if (coefficients[i].family != family) continue;
            auto& connectivity = family == SearchShellFamily::Q4 ? quad : triangle;
            auto& source = family == SearchShellFamily::Q4 ? qsource : tsource;
            source.push_back(i);
            for (unsigned j = 0; j < (family == SearchShellFamily::Q4 ? 4u : 3u); ++j)
                connectivity.push_back(native_node.at(nodes[i][j]));
            const auto& c = coefficients[i];
            geometry.push_back(c.geometry);
            modulus.push_back(c.modulus);
            part.push_back(c.part);
            element.push_back(c.element);
        }
    }
    for (auto i : order) {
        output::Require(i < nodes.size(), "Invalid native test order");
        const bool q4 = coefficients[i].family == SearchShellFamily::Q4;
        const auto& source = q4 ? qsource : tsource;
        auto& native_order = q4 ? qorder : torder;
        native_order.push_back(std::find(source.begin(), source.end(), int(i))-source.begin()+1);
    }
    const int unused = 1;
    NativeGeometryResult result;
    int status = -1;
    native_tied_search_geometry(native_node.size(), qsource.size(), tsource.size(), native_master.data(),
        quad.empty() ? &unused : quad.data(), triangle.empty() ? &unused : triangle.data(),
        qorder.empty() ? &unused : qorder.data(), torder.empty() ? &unused : torder.data(),
        geometry.data(), modulus.data(), part.data(), element.data(),
        result.selected.data(), result.consumed.data(), &status);
    output::Require(status == 0, "Independent native geometry packet rejected");
    // Translate only returned family indices to supplied packet rows. No
    // production rank/thickness helper participates in this oracle.
    result.selected[0] = result.selected[0] ? qsource.at(result.selected[0]-1) : -1;
    result.selected[1] = result.selected[1] ? tsource.at(result.selected[1]-1) : -1;
    return result;
}
} // namespace crash::modelio::tied_shell::test
