#pragma once
#include "AcceptedReplayData.h"
#include <charconv>
#include <sstream>

namespace crash::output::replay_detail {
template<class T> T ReplayCsvNumber(const std::string& text) {
    T value{};const auto p=std::from_chars(text.data(),text.data()+text.size(),value);
    Require(p.ec==std::errc{}&&p.ptr==text.data()+text.size(),"Malformed replay CSV number");return value;
}
template<std::size_t N> std::array<std::string,N> ReplayCsvRow(const std::string& line) {
    std::array<std::string,N> columns;std::istringstream row(line);
    for(auto& text:columns)Require(bool(std::getline(row,text,','))&&!text.empty(),"Incomplete replay CSV row");
    Require(row.eof()&&!line.empty()&&line.back()!=',',"Extra replay CSV column");return columns;
}
} // namespace crash::output::replay_detail
