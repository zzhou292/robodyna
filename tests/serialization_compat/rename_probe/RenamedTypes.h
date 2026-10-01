// A real type/namespace rename of the frozen probe, with unchanged object layout.
#pragma once

#include "chrono/serialization/ChArchive.h"

namespace robodyna::serialization_compat {
struct RbRenameProbePrimary {
    virtual ~RbRenameProbePrimary() = default;
    double layout_marker = 1.25;
};

struct RbRenameProbeBase {
    virtual ~RbRenameProbeBase() = default;
    virtual int Kind() const = 0;
    virtual void ArchiveOut(chrono::ChArchiveOut& archive);
    virtual void ArchiveIn(chrono::ChArchiveIn& archive);
    int base_value = 17;
    int observed_base_version = -1;
};

struct RbRenameProbe : RbRenameProbePrimary, RbRenameProbeBase {
    int Kind() const override { return 41; }
    void ArchiveOut(chrono::ChArchiveOut& archive) override;
    void ArchiveIn(chrono::ChArchiveIn& archive) override;
    double value = 2.5;
    int observed_version = -1;
};
}  // namespace robodyna::serialization_compat

namespace chrono {
CH_CLASS_VERSION(::robodyna::serialization_compat::RbRenameProbeBase, 3)
CH_CLASS_VERSION(::robodyna::serialization_compat::RbRenameProbe, 5)
}  // namespace chrono
