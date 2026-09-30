#pragma once
#include "Source.h"
#include "modelio/tied_shell/Internal.h"
#include "modelio/source_assembly/SourceFields.h"
#include "modelio/source_assembly/SourceCurve.h"
#include "modelio/source_assembly/AuxiliarySourceCards.h"
namespace crash::modelio::beam18::detail {
using output::Require;
using namespace assembly::reader;
constexpr std::uint64_t Parts[]{2000514,2000520,2000948,2000950};
bool Selected(std::uint64_t);
void AddBytes(std::size_t&,std::size_t count,std::size_t width,std::size_t cap);
Forecast Budget(const source::CanonicalData&,Policy,Limits);
std::size_t OwnedPayload(const Data&,Limits);
void ReadDeclarations(const source::CanonicalData&,const std::string&,Data&,Limits);
void ReadPart(Part&,Data&);
void ReadGeometry(const source::CanonicalData&,const std::string&,Data&,Limits);
void ReadWorkingCards(const source::CanonicalData&,const std::string&,Data&);
void PrepareReferences(Data&);
template<class T> std::vector<T> Decode(const source::CanonicalData& source,const char* name) {
    const auto& a=source::FindArray(source,name);
    return output::arrays::Decode<T>(a.descriptor,a.bytes);
}
} // namespace crash::modelio::beam18::detail
