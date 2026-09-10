#pragma once
#include "ArtifactIO.h"
namespace crash::output {
// Interleaved (plastic strain,yield stress Pa), IEEE754 binary64 big endian,
// from the original authenticated 46-pair readiness declaration. No formatting
// or host endianness affects this semantic source identity.
inline constexpr const char* SourcePartPlasticCurveSha256="9c759cc11a0662090dcc585ad69ee4f21ce1f899c3b554ba9bbabfda2745d9e4";
inline std::string PlasticCurveSha256(const double* strain,const double* stress,std::size_t count) {
    Require(strain&&stress&&count==46,"Source plastic curve requires46 points");
    std::string bytes;bytes.reserve(16*count);
    for(std::size_t p=0;p<count;++p)for(double x:{strain[p],stress[p]}) {
        const auto bits=Bits(x);for(unsigned b=0;b<8;++b)bytes.push_back(static_cast<char>((bits>>(8*(7-b)))&255));
    }
    return Sha256(bytes);
}
}
