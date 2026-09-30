#pragma once
#include "TiedShellDeclaration.h"
#include "modelio/source_assembly/JsonReader.h"
#include "modelio/vehicle_source/SourceCards.h"
#include <map>
#include <set>

namespace crash::modelio::tied_shell::detail {
using namespace assembly::reader;
using output::Document;
struct Request {
    SourceEvidence evidence;
    const Value* cards = nullptr;
};
using Requests = std::map<std::size_t, Request>;
struct Draft {
    Data data;
    Requests requests;
    std::map<SourceId, std::size_t> part_rows;
    std::map<std::size_t, const Value*> original_blocks;
    std::vector<SourceId> master_ids, slave_ids;
};
std::size_t Preflight(const source::CanonicalData&, Limits);
std::size_t OwnedPayload(const Data&, std::size_t cap);
std::size_t RequestSource(Draft&, const Value&, const Value& original_blocks, Limits);
void ReadSources(Draft&, const std::string&, Limits);
std::vector<SourceEvidence> ReadRequestedSources(Requests&, const std::string&, Limits);
std::vector<SourceId> ListIds(const SourceEvidence&, bool title, Limits);
SourceId CardId(const std::string&, unsigned column);
void CheckContact(const SourceEvidence&, SourceId slave, SourceId master);
Draft Declarations(const source::CanonicalData&, const Value& scope, const Value& canonical, Limits);
void Geometry(const source::CanonicalData&, const Value& scope, Draft&, Limits);
void Constraints(const source::CanonicalData&, const Value& scope, const Value& canonical, Draft&, Limits);
void ValidateCards(Draft&, Limits);
Data Build(const source::CanonicalData&, const std::string&, Limits);
} // namespace crash::modelio::tied_shell::detail
