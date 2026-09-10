#pragma once
#include "JsonReader.h"
#include <charconv>
#include <cmath>
#include <sstream>

namespace crash::modelio::assembly::reader::auxiliary {
inline std::string Trim(std::string value) {
    const auto last=value.find_last_not_of(" \t\r\n");
    if(last==std::string::npos)return {};
    value.resize(last+1);return value;
}
inline std::vector<std::string> RawCards(const Value& value,const SourceBlock& block) {
    const auto& cards=Array(value,"cards",4096,1);
    std::vector<std::string> result;std::istringstream stream(block.raw_text);std::string line;
    std::size_t number=block.first_line;
    Require(bool(std::getline(stream,line))&&Trim(line)==block.keyword,"Auxiliary source keyword differs from raw block");
    while(std::getline(stream,line)) {
        ++number;const auto first=line.find_first_not_of(" \t\r");
        if(first!=std::string::npos&&line[first]=='$')continue;
        const auto comment=line.find('$');if(comment!=std::string::npos)line.resize(comment);
        line=Trim(line);Require(result.size()<cards.Size(),"Auxiliary raw source has omitted cards");
        const auto& card=cards[static_cast<unsigned>(result.size())];
        Require(Unsigned(card,"source_line")==number&&Text(card,"text")==line,
                "Auxiliary source card changed or moved");
        result.push_back(line);
    }
    Require(number==block.last_line&&result.size()==cards.Size(),"Auxiliary source block/card extent differs");
    return result;
}
inline std::string Field(const std::string& row,std::size_t offset,std::size_t width) {
    Require(offset<row.size(),"Missing auxiliary source field");
    auto result=Trim(row.substr(offset,width));const auto first=result.find_first_not_of(" \t");
    Require(first!=std::string::npos,"Blank auxiliary source field");return result.substr(first);
}
inline SourceId Id(const std::string& row,std::size_t offset,std::size_t width) {
    const auto field=Field(row,offset,width);SourceId value=0;
    const auto parsed=std::from_chars(field.data(),field.data()+field.size(),value);
    Require(parsed.ec==std::errc{}&&parsed.ptr==field.data()+field.size()&&value&&value<=UINT32_MAX,
            "Invalid auxiliary source identity");return value;
}
inline double Number(const std::string& row,std::size_t offset,std::size_t width) {
    const auto field=Field(row,offset,width);double value=0;
    const auto parsed=std::from_chars(field.data(),field.data()+field.size(),value);
    Require(parsed.ec==std::errc{}&&parsed.ptr==field.data()+field.size()&&std::isfinite(value),
            "Invalid auxiliary source scalar");return value;
}
inline bool BlankTail(const std::string& row,std::size_t offset) {
    return offset>=row.size()||Trim(row.substr(offset)).empty();
}
} // namespace crash::modelio::assembly::reader::auxiliary
