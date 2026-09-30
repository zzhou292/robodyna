#pragma once
#include "output/ArtifactIO.h"
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

namespace crash::benchmarks::assembly_pilot {
// Nested integer refinements only. The serialized binary64 dt must reproduce
// the reference multiple exactly; close decimal timestamps never join frames.
inline std::uint64_t StepMultiple(double reference,double candidate) {
    output::Require(std::isfinite(reference)&&reference>0&&std::isfinite(candidate)&&candidate>0,
                    "Pilot steps must be finite and positive");
    const long double ratio=static_cast<long double>(candidate)/reference;
    output::Require(ratio>=1&&ratio<=1048576,"Reference must be the finest step; multiplier cap is 2^20");
    const auto multiple=static_cast<std::uint64_t>(std::round(ratio));
    int re=0,ce=0;
    const auto rm=static_cast<std::uint64_t>(std::ldexp(std::frexp(reference,&re),53));
    const auto cm=static_cast<std::uint64_t>(std::ldexp(std::frexp(candidate,&ce),53));
    // 53-bit significands times at most 2^20 fit exactly in this host integer.
    // Rounded floating multiplication would falsely admit e.g. 0.1 * 3.
    output::Require(ce>=re&&ce-re<=21&&
        static_cast<unsigned __int128>(rm)*multiple==(static_cast<unsigned __int128>(cm)<<(ce-re)),
        "Pilot dt is not an exact rational integer reference-step multiple");
    return multiple;
}
inline std::uint64_t Tick(std::uint64_t epoch,std::uint64_t multiple) {
    output::Require(multiple&&epoch<=UINT64_MAX/multiple,"Pilot physical-time tick overflow");
    return epoch*multiple;
}
inline std::vector<std::pair<std::size_t,std::size_t>> CommonFrames(
        const std::vector<std::uint64_t>& a,std::uint64_t am,
        const std::vector<std::uint64_t>& b,std::uint64_t bm) {
    for(const auto* sequence:{&a,&b}) {
        output::Require(!sequence->empty()&&sequence->size()<=1000&&sequence->front()==0,"Missing initial pilot sample or frame cap");
        for(std::size_t i=1;i<sequence->size();++i)
            output::Require((*sequence)[i]>(*sequence)[i-1],"Pilot epochs are duplicate or unordered");
    }
    for(auto e:a)Tick(e,am);for(auto e:b)Tick(e,bm);
    std::vector<std::pair<std::size_t,std::size_t>> result;
    std::size_t i=0,j=0;
    while(i<a.size()&&j<b.size()) {
        const auto x=Tick(a[i],am),y=Tick(b[j],bm);
        if(x==y)result.emplace_back(i++,j++);else if(x<y)++i;else ++j;
    }
    return result;
}
} // namespace crash::benchmarks::assembly_pilot
