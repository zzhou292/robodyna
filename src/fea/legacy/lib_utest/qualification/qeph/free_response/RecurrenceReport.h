#pragma once
#include "RecurrenceAudit.h"
#include "output/ArtifactIO.h"
namespace tl::qualification::qeph::recurrence {
// Qualification-only JSON boundary. Numerical targets do not include this file.
crash::output::Document DescribeAudit(const Audit&,const crash::output::Document& provenance,
    bool include_raw,const std::string& raw_hash="",std::size_t raw_bytes=0);
std::string EncodeReport(const crash::output::Document&); // Finite JSON, <=32 MiB.
}
