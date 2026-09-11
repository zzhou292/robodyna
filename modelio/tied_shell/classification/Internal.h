#pragma once
#include "../TiedClassificationContext.h"
#include "../Internal.h"

namespace crash::modelio::tied_shell::classification_detail {
using namespace assembly::reader;
void Add(std::size_t&, std::size_t count, std::size_t width, std::size_t cap);
void SourceRoles(const source::CanonicalData&, const Data&, const AuxiliaryData&,
    const vehicle::rigid_part::SourceData&, ClassificationSourceReceipt&, ClassificationSourceLimits);
void ReadSet(const Data&, const AuxiliaryData&, const vehicle::rigid_part::SourceData&,
    ClassificationSourceReceipt&);
OriginalWallAssemblyReceipt Wall(const source::CanonicalData&, const Data&,
    const std::string&, ClassificationSourceLimits);
std::size_t OwnedPayload(const ClassificationSourceReceipt&, std::size_t cap);
bool Observed(const Data&, SourceId);
}
