#pragma once
#include "VehicleSolidSource.h"
#include "SourcePolicy.h"
#include "modelio/tied_shell/Internal.h"

namespace crash::modelio::solid_source::detail {
using output::Require;
using namespace assembly::reader;
template<class T> std::vector<T> Decode(const source::CanonicalData& source, const char* name) {
    const auto& a = source::FindArray(source, name);
    return output::arrays::Decode<T>(a.descriptor, a.bytes);
}
void CheckOriginal(const source::CanonicalData&);
Forecast Budget(const source::CanonicalData&, Policy, Limits);
void ReadDeclarations(const source::CanonicalData&, const std::string&, Data&, Limits);
void ReadPart(Part&, const std::vector<tied_shell::SourceEvidence>&, Data&);
void PrepareMaterial(Part&, Data&);
void ReadRearMaterial(Part&, const tied_shell::SourceEvidence&, Data&);
void ReadGeometry(const source::CanonicalData&, const std::string&, Data&, Limits);
void PrepareReferences(const source::CanonicalData&, Data&, Limits);
std::size_t OwnedPayload(const Data&, Limits);
double Required(const std::string&, unsigned column, unsigned width = 10);
void Blank(const std::string&, unsigned first, unsigned last);
void ReadCurve(const tied_shell::SourceEvidence&, Data&);
} // namespace crash::modelio::solid_source::detail
