#include "Internal.h"
#include <charconv>
#include <algorithm>
#include <cmath>
#include <map>
#include <sstream>

namespace crash::modelio::vehicle::detail {
void CheckSource(const assembly::SourceBlock& source,const Value& expected,const char* hash_key) {
    Require(source.filename=="yaris-coarse-v1l.key"&&source.sha256==Text(expected,hash_key)&&
        output::Sha256(source.raw_text)==source.sha256,"Vehicle original declaration block association changed");
    if(expected.HasMember("keyword")) {
        Require(source.keyword==Text(expected,"keyword")&&source.filename==Text(expected,"file")&&
            source.first_line==Unsigned(expected,"first_line")&&source.last_line==Unsigned(expected,"last_line"),
            "Vehicle source block location changed");
    } else {
        Require(source.keyword=="*PART"&&source.first_line==Unsigned(expected,"source_line"),
                "Vehicle original PART location changed");
    }
    const auto end=source.raw_text.find('\n');auto keyword=source.raw_text.substr(0,end);
    if(!keyword.empty()&&keyword.back()=='\r')keyword.pop_back();
    const auto lines=std::count(source.raw_text.begin(),source.raw_text.end(),'\n')+
        (!source.raw_text.empty()&&source.raw_text.back()!='\n');
    Require(keyword==source.keyword&&lines&&source.first_line+lines-1==source.last_line,
            "Vehicle source raw block extent/keyword changed");
}
void CheckTypedCards(const assembly::SourceBlock& block,const std::vector<assembly::DeclarationCard>& cards,unsigned width) {
    std::map<std::size_t,std::string> original;
    std::istringstream stream(block.raw_text);std::string line;std::size_t number=block.first_line;
    while(std::getline(stream,line)) {
        if(!line.empty()&&line.back()=='\r')line.pop_back();
        const auto comment=line.find('$');if(comment!=std::string::npos)line.resize(comment);
        const auto last=line.find_last_not_of(" \t");line.resize(last==std::string::npos?0:last+1);
        original.emplace(number++,line);
    }
    Require(number==block.last_line+1,"Vehicle source block line extent changed");
    for(std::size_t i=0;i<cards.size();++i) {
        const auto& card=cards[i];const auto found=original.find(card.source_line);
        Require(found!=original.end()&&found->second==card.raw_text,"Typed card does not match original source text");
        const auto columns=(width==20&&i==0)?10:width;
        for(std::size_t f=0;f<card.values.size();++f) {
            const auto offset=f*columns;
            const auto field=offset<card.raw_text.size()?card.raw_text.substr(offset,columns):std::string{};
            const auto begin=field.find_first_not_of(" \t"),end=field.find_last_not_of(" \t");
            Require((begin==std::string::npos)==!card.values[f],"Typed source blank field changed");
            if(!card.values[f])continue;
            auto first=field.data()+begin;const auto last=field.data()+end+1;
            if(first!=last&&*first=='+')++first;
            double value=0;const auto parsed=std::from_chars(first,last,value);
            Require(parsed.ec==std::errc{}&&parsed.ptr==last&&std::isfinite(value),"Invalid original numerical card field");
            Same(value,*card.values[f]);
        }
    }
}
} // namespace crash::modelio::vehicle::detail
