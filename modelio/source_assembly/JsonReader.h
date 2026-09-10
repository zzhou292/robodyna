#pragma once
#include "SourceAssemblyData.h"
#include "output/ArtifactIO.h"
#include <string_view>

namespace crash::modelio::assembly::reader {
using crash::output::Value;
using crash::output::Require;
const Value& Member(const Value&, const char*);
const Value& Array(const Value&, const char*, std::size_t cap, std::size_t minimum = 0);
std::string Text(const Value&);
std::string Text(const Value&, const char*);
std::uint64_t Unsigned(const Value&, std::uint64_t cap = UINT64_MAX);
std::uint64_t Unsigned(const Value&, const char*, std::uint64_t cap = UINT64_MAX);
double Real(const Value&);
double Real(const Value&, const char*);
std::optional<double> OptionalReal(const Value&, const char*);
void Flag(const Value&, const char*, bool);
void TextIs(const Value&, const char*, std::string_view);
void Same(double, double);
void UniqueKeys(const Value&, unsigned depth = 0);
std::vector<SourceId> Ids(const Value&, std::size_t cap, bool ascending = false);
SourceBlock Block(const Value&);
std::vector<DeclarationCard> Cards(const Value&, std::size_t cap);
std::vector<RawAttachmentCard> AttachmentCards(const Value&, std::size_t cap);
std::size_t NodeIndex(const Data&, SourceId);
void ReadDeclarations(const Value&, const ReadLimits&, Data&);
Material ReadLaw44Material(const Value&, const Data&);
Material ReadLaw1Material(const Value&, const Data&);
Material ReadMaterial(const Value&, const Data&);
void ReadMaterialPolicy(const Value&, const Data&);
void ReadAuxiliaryFrontier(const Value&,const Value&,const ReadLimits&,Data&);
void ReadGeometry(const Value&, const ReadLimits&, Data&);
void ReadAttachments(const Value&, const ReadLimits&, Data&);
}  // namespace crash::modelio::assembly::reader
