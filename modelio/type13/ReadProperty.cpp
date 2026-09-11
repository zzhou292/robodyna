#include "ReadInternal.h"
#include <array>

namespace crash::modelio::type13::reader {
namespace {
std::vector<assembly::DeclarationCard> CheckedCards(const Value& object,assembly::SourceBlock& block,
    const char* keyword,const char* hash,std::size_t count) {
    Require(object.IsObject()&&object.MemberCount()==2,"TYPE13 property declaration shape changed");
    const auto& encoded=Member(object,"source");block=Block(encoded);
    Require(block.filename=="yaris-coarse-v1l.key"&&block.keyword==keyword&&block.sha256==hash,
            "TYPE13 original property source block identity changed");
    const auto raw=auxiliary::RawCards(encoded,block);Require(raw.size()==count,"TYPE13 property card count changed");
    auto cards=Cards(object,count);Require(cards.size()==count,"TYPE13 typed property card count changed");
    for(std::size_t k=0;k<count;++k) {
        auto& card=cards[k];Require(card.raw_text==raw[k]&&
            card.source_line==Unsigned(Member(encoded,"cards")[static_cast<unsigned>(k)],"source_line"),
            "TYPE13 typed/raw card text or line changed");
        Require(card.names.size()==8&&card.values.size()==8,"TYPE13 property card width changed");
        unsigned mask=0;
        for(unsigned f=0;f<8;++f) {
            const auto field=raw[k].substr(std::min<std::size_t>(10*f,raw[k].size()),10);
            const bool blank=auxiliary::Trim(field).empty();mask|=blank?(1u<<f):0;
            if(!blank){Require(bool(card.values[f]),"Missing TYPE13 supplied property field");
                Same(*card.values[f],auxiliary::Number(raw[k],10*f,10));}
        }
        Require(card.blank_mask==mask,"TYPE13 raw card blank mask changed");
        Require(auxiliary::BlankTail(raw[k],80),"TYPE13 property card has trailing fields");
    }
    return cards;
}
void Names(const assembly::DeclarationCard& card,std::initializer_list<const char*> names) {
    Require(card.names.size()==names.size(),"TYPE13 source field inventory changed");
    std::size_t i=0;for(const char* name:names)Require(card.names[i++]==name,"TYPE13 source field identity changed");
}
}
void ReadProperty(const Value& document,Data& data) {
    const auto& values=Member(document,"property");const auto& part=Member(values,"part");
    Require(values.IsObject()&&values.MemberCount()==3&&part.IsObject()&&part.MemberCount()==6,
            "TYPE13 part/property declaration shape changed");
    Require(Unsigned(part,"part_id")==2000486&&Unsigned(part,"section_id")==2000486&&
        Unsigned(part,"material_id")==2000486,"TYPE13 original PART association changed");
    const auto& encoded=Member(part,"source");data.part_source=Block(encoded);
    Require(data.part_source.keyword=="*PART"&&data.part_source.filename=="yaris-coarse-v1l.key"&&
        data.part_source.sha256=="f28e7571a05a69f52275a9dc13fa1e193018f0cffacf730c132c209b97f77259",
        "TYPE13 original PART block changed");
    const auto raw=auxiliary::RawCards(encoded,data.part_source);
    Require(raw.size()==2&&raw[0]==Text(part,"title"),"TYPE13 original PART title changed");
    for(unsigned i=0;i<3;++i)Require(auxiliary::Id(raw[1],10*i,10)==2000486,"TYPE13 raw PART association changed");
    Require(auxiliary::BlankTail(raw[1],30),"TYPE13 PART has unsupported source fields");
    const auto part_cards=Cards(part,1);Require(part_cards.size()==1&&part_cards[0].blank_mask==248&&
        part_cards[0].raw_text==raw[1]&&part_cards[0].source_line==Unsigned(Member(encoded,"cards")[1],"source_line"),
        "TYPE13 typed PART card changed");
    Names(part_cards[0],{"pid","secid","mid","eosid","hgid","grav","adpopt","tmid"});
    for(unsigned i=0;i<3;++i){Require(bool(part_cards[0].values[i]),"TYPE13 PART source identity is blank");Same(*part_cards[0].values[i],2000486);}
    const auto section=CheckedCards(Member(values,"section"),data.section_source,"*SECTION_BEAM",
        "dbbaad5f49c8af0c1b2a333dbc01f5cafc842c6c9a8188ae94571c6b21de1275",2);
    const auto material=CheckedCards(Member(values,"material"),data.material_source,"*MAT_SPOTWELD",
        "0bc085268ae4d90b2fad5d29a58cbb852feb99fbf447266dbbdcf46667b2c2b7",2);
    Names(section[0],{"secid","elform","shrf","qr_irid","cst","scoor","nsm","reserved8"});
    Names(section[1],{"ts1","ts2","tt1","tt2","nprint","reserved6","reserved7","reserved8"});
    Names(material[0],{"mid","ro","e","pr","sigy","et","dt","tfail"});
    Names(material[1],{"efail","nrr","nrs","nrt","mrr","mss","mtt","nf"});
    Require(section[0].blank_mask==224&&section[1].blank_mask==252&&
        material[0].blank_mask==64&&material[1].blank_mask==254,"TYPE13 source optional-field scope changed");
    const auto& a=material[0].values;const auto& b=section[1].values;
    data.source_property={*a[1],*a[2],*a[3],*a[4],*a[5],*b[0],*b[1],0,0,*material[1].values[0]};
    Require(ConvertProperty(data.source_property,data.converted)==native::Status::Success,
            "TYPE13 native source property conversion rejected");
    Require(native::InitializeProperty(data.converted.input(),data.property)==native::Status::Success,
            "TYPE13 native property initialization rejected");
}
}
