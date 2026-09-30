#pragma once
#include "../MidlayerDeclarations.h"
#include <algorithm>

namespace crash::modelio::vehicle::test {
inline assembly::SourceBlock Block(const std::string& text, std::size_t line) {
    assembly::SourceBlock block;
    block.filename="yaris-coarse-v1l.key";
    block.keyword=text.substr(0,text.find('\n'));
    block.raw_text=text;
    block.first_line=line;
    block.last_line=line+std::count(text.begin(),text.end(),'\n')-1;
    block.sha256=output::Sha256(text);
    return block;
}
// Original numeric fields, with short source-shaped comments. These value tests
// do not impersonate the full canonical source required by the public factory.
inline PartDisposition MidlayerFields() {
    PartDisposition part;
    part.part_id=part.material_id=part.section_id=2000524;
    part.shell_count=4251;
    part.unresolved_sources={
        Block("*PART\n$ title\n125-windshield-midlayer\n$ ids\n   2000524   2000524   2000524\n",10107),
        Block("*SECTION_SHELL\n$ section\n   2000524         9                   1\n$ thickness\n"
              "  0.500000  0.500000  0.500000  0.500000\n",10112),
        Block("*MAT_PIECEWISE_LINEAR_PLASTICITY\n$ first\n"
              "   2000524 1.0000E-9 250.00000  0.350000 10.000000  1.000000  2.500000\n"
              "$ second\n                             0         0\n$ eps\n\n$ stress\n\n$ end\n",10117)};
    return part;
}
inline void SetField(assembly::SourceBlock& block,std::size_t data_line,unsigned field,const std::string& value) {
    auto text=block.raw_text;
    std::size_t start=0;
    for(std::size_t i=block.first_line;i<data_line;++i) start=text.find('\n',start)+1;
    const auto end=text.find('\n',start);
    auto line=text.substr(start,end-start);
    line.resize(std::max(line.size(),std::size_t(field+1)*10),' ');
    output::Require(value.size()<=10,"Test field too wide");
    line.replace(field*10,10,std::string(10-value.size(),' ')+value);
    text.replace(start,end-start,line);
    block=Block(text,block.first_line);
}
} // namespace crash::modelio::vehicle::test
