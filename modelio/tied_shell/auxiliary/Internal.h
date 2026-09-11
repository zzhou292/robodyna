#pragma once
#include "../TiedAuxiliaryConstraints.h"
#include "../Internal.h"
#include <map>
#include <set>

namespace crash::modelio::tied_shell::auxiliary_detail {
using namespace assembly::reader;
using output::Document;
inline constexpr const char* MemberName = "set-yaris-coarse-v1l.key";
struct Draft {
    AuxiliaryData data;
    std::set<SourceId> members;
};
void Add(std::size_t&, std::size_t count, std::size_t width, std::size_t cap);
std::size_t Preflight(const source::CanonicalData&, const Data&, std::size_t, AuxiliaryLimits);
std::size_t OwnedPayload(const AuxiliaryData&, std::size_t);
std::vector<SourceEvidence> Sources(const Value&, const std::string&, AuxiliaryLimits);
void Groups(Draft&, const Data&, AuxiliaryLimits);
void Nodes(Draft&, const source::CanonicalData&, const Data&, AuxiliaryLimits);
void Boundary(Draft&, const Data&, const Value&, OriginalWallPolicy, AuxiliaryLimits);
AuxiliaryData Build(const source::CanonicalData&, const Data&, const std::string&,
                    OriginalWallPolicy, AuxiliaryLimits = {});
} // namespace crash::modelio::tied_shell::auxiliary_detail
