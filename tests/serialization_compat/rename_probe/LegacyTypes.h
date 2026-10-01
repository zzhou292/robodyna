// Test-only pre-rename identities. Keep these declarations unchanged as evidence.
#pragma once

#include "chrono/serialization/ChArchive.h"

namespace chrono {
// A nonempty first base makes the serializable second base pointer nontrivial.
struct ChRenameProbePrimary {
    virtual ~ChRenameProbePrimary() = default;
    double layout_marker = 1.25;
};

struct ChRenameProbeBase {
    virtual ~ChRenameProbeBase() = default;
    virtual int Kind() const = 0;
    virtual void ArchiveOut(ChArchiveOut& archive);
    virtual void ArchiveIn(ChArchiveIn& archive);
    int base_value = 17;
    int observed_base_version = -1;
};

struct ChRenameProbe : ChRenameProbePrimary, ChRenameProbeBase {
    int Kind() const override { return 41; }
    void ArchiveOut(ChArchiveOut& archive) override;
    void ArchiveIn(ChArchiveIn& archive) override;
    double value = 2.5;
    int observed_version = -1;
};

CH_CLASS_VERSION(ChRenameProbeBase, 3)
CH_CLASS_VERSION(ChRenameProbe, 5)
}  // namespace chrono
