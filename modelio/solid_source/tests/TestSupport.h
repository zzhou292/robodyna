#pragma once
#include "../Internal.h"
#include "output/full_shell/static_bundle/MappingArrays.h"
#include <gtest/gtest.h>
#include <iomanip>
#include <sstream>

namespace crash::modelio::solid_source::test {
inline std::string Card(std::initializer_list<const char*> fields, unsigned width = 10) {
    std::ostringstream text;
    for (auto field : fields) text << std::setw(width) << field;
    return text.str();
}
inline tied_shell::SourceEvidence Evidence(const char* keyword, std::vector<std::string> cards) {
    tied_shell::SourceEvidence source;
    source.block.keyword = keyword;
    for (std::size_t i = 0; i < cards.size(); ++i) source.cards.emplace_back(10 + i, cards[i]);
    return source;
}
inline Data Rubber() {
    Data data;
    data.sources = {
        Evidence("*PART", {"Original rubber", Card({"2000477", "2000477", "2000477", "", "2000017"})}),
        Evidence("*SECTION_SOLID", {Card({"2000477"})}),
        Evidence("*MAT_BLATZ-KO_RUBBER", {Card({"2000477", "1.9800E-9", "24.000000"})}),
        Evidence("*HOURGLASS", {Card({"2000017", "2", ".1", "0", "1.5E-4", "6E-5"})})};
    Part part;
    part.id = part.section_id = part.material_id = 2000477;
    part.sources = {0, 1, 2};
    data.parts.push_back(part);
    return data;
}
inline Data Adhesive() {
    Data data;
    auto curve = Evidence("*DEFINE_CURVE", {Card({"2100010", "0", "1", "1"})});
    // Small numerical values exercise the parser/typed conversion independently
    // of the actual curve fixture used by the original-source gate.
    for (unsigned i = 0; i < 8; ++i)
        curve.cards.emplace_back(20 + i, Card({std::to_string(i * .001).c_str(),
            std::to_string(10.1 + i).c_str()}, 20));
    data.sources = {
        Evidence("*PART", {"Original adhesive", Card({"2000977", "2000977", "2000977"})}),
        Evidence("*SECTION_SOLID", {Card({"2000977", "2"})}),
        Evidence("*MAT_PIECEWISE_LINEAR_PLASTICITY", {
            Card({"2000977", "1.0700E-9", "1887.0000", ".417000", "10.041"}),
            Card({"", "", "2100010", "", "0.0"}), "", ""}), curve};
    Part part;
    part.id = part.section_id = part.material_id = 2000977;
    part.sources = {0, 1, 2};
    part.curve_source = 3;
    data.parts.push_back(part);
    return data;
}
inline void Resolve(Data& data) {
    detail::ReadPart(data.parts[0], data.sources, data);
    detail::PrepareMaterial(data.parts[0], data);
}
} // namespace crash::modelio::solid_source::test
