#pragma once
#include "AuxiliarySourceCards.h"
#include <algorithm>
namespace crash::modelio::assembly::reader {
// Requests must be strictly increasing one-based source line numbers. Original
// fixed columns survive trimming; callbacks see the enclosing keyword.
template<class Visit> void VisitSourceLines(const std::string& member,
    const std::vector<std::uint32_t>& requests,Visit&& visit) {
    Require(!requests.empty() && requests.front()>0 && std::is_sorted(requests.begin(),requests.end()) &&
        std::adjacent_find(requests.begin(),requests.end())==requests.end(),"Invalid requested source-line order");
    std::size_t cursor=0,line=1,next=0;std::string keyword;
    while(cursor<member.size() && next<requests.size()) {
        const auto end=member.find('\n',cursor),stop=end==std::string::npos ? member.size() : end;
        Require(stop-cursor<=4096,"Original source line exceeds bounded width");
        auto text=member.substr(cursor,stop-cursor);const auto comment=text.find('$');
        if(comment!=std::string::npos)text.resize(comment);
        text=auxiliary::Trim(std::move(text));
        if(!text.empty() && text.front()=='*')keyword=text;
        if(requests[next]==line){visit(next,keyword,text);++next;}
        cursor=end==std::string::npos ? member.size() : end+1;++line;
    }
    Require(next==requests.size(),"Original requested source-line coverage incomplete");
}
} // namespace crash::modelio::assembly::reader
