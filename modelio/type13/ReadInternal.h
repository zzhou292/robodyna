#pragma once
#include "SourceType13.h"
#include "modelio/source_assembly/JsonReader.h"
#include "modelio/source_assembly/AuxiliarySourceCards.h"

namespace crash::modelio::type13::reader {
using namespace assembly::reader;
void ReadScope(const Value&);
void ReadSourceArrays(const Value&);
void ReadProperty(const Value&,Data&);
void ReadGeometry(const Value&,ReadLimits,Data&);
std::size_t Preflight(const ArtifactIdentity&,ReadLimits);
std::size_t OwnedPayload(const Data&,std::size_t cap);
}
