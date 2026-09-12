#include "SourceCurve.h"
#include "SourceFields.h"
#include "AuxiliarySourceCards.h"
namespace crash::modelio::assembly::reader {
void ReadSourceCurve(const SourceBlock& block,const std::vector<std::pair<std::size_t,std::string>>& cards,
    SourceId id,std::size_t count,double scale,std::vector<double>& x,std::vector<double>& y) {
    Require(count && count<4096 && std::isfinite(scale) && scale>0 &&
        block.keyword=="*DEFINE_CURVE" && cards.size()==count+1 &&
        auxiliary::Id(cards[0].second,0,10)==id,"Original curve identity or point count changed");
    const auto& header=cards[0].second;
    Require(RequiredScalar(header,1)==0 && RequiredScalar(header,2)==1 && RequiredScalar(header,3)==1,
        "Original curve scales or options changed");
    RequireBlankFields(header,4,8);
    const bool shared=!x.empty();
    Require(x.size()==y.size() && (!shared || x.size()==count),"Incomplete owned source curve");
    std::vector<double> next_x,next_y;
    if(!shared) {next_x.reserve(count);next_y.reserve(count);}
    for(std::size_t i=0;i<count;++i) {
        const auto& row=cards[i+1].second;
        Require(auxiliary::BlankTail(row,40),"Extra original curve values");
        const double xi=RequiredScalar(row,0,20),yi=RequiredScalar(row,1,20)*scale;
        Require(std::isfinite(yi),"Scaled source curve ordinate overflow");
        if(shared) Require(output::Bits(x[i])==output::Bits(xi) && output::Bits(y[i])==output::Bits(yi),
            "Shared original curve differs between parts");
        else {next_x.push_back(xi);next_y.push_back(yi);}
    }
    if(!shared) {x=std::move(next_x);y=std::move(next_y);}
}
} // namespace crash::modelio::assembly::reader
