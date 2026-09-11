#pragma once
#include "VehicleShellReferences.h"
#include "ReferenceDeclarations.h"
#include "modelio/source_assembly/SourceShellReferenceInput.h"

namespace crash::cases::vehicle_startup::detail {
struct ReferenceStorage {
    std::vector<ReferenceRow> rows;
    std::vector<tl::fea::qeph::ReferenceData> qeph;
    std::vector<tl::fea::t3::ReferenceData> t3;
    ReferenceCounts counts;
    std::size_t first_error=SIZE_MAX;
};
// Called in source parent order. Only a successful native packet is appended;
// each attempt has its own zero-initialized temporary, never a reused output.
void Append(ReferenceStorage&,ReferenceRow,const tl::fea::qeph::ReferenceInput&);
void Append(ReferenceStorage&,ReferenceRow,const tl::fea::t3::ReferenceInput&);
void AppendUnresolved(ReferenceStorage&,ReferenceRow);
struct Geometry {
    std::vector<std::uint64_t> node_ids, records;
    std::vector<double> positions;
    std::vector<std::uint32_t> connections, lines;
    explicit Geometry(const modelio::vehicle::source::CanonicalData&);
};
void PrepareRows(const DeclarationView&,const Geometry&,ReferenceStorage&);
} // namespace crash::cases::vehicle_startup::detail
