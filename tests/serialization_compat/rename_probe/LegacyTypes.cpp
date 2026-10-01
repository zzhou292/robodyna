#include "tests/serialization_compat/rename_probe/LegacyTypes.h"

namespace chrono {
CH_FACTORY_REGISTER(ChRenameProbe)
CH_UPCASTING(ChRenameProbe, ChRenameProbeBase)

void ChRenameProbeBase::ArchiveOut(ChArchiveOut& archive) {
    archive.VersionWrite<ChRenameProbeBase>();
    archive << CHNVP(base_value);
}

void ChRenameProbeBase::ArchiveIn(ChArchiveIn& archive) {
    observed_base_version = archive.VersionRead<ChRenameProbeBase>();
    archive >> CHNVP(base_value);
}

void ChRenameProbe::ArchiveOut(ChArchiveOut& archive) {
    archive.VersionWrite<ChRenameProbe>();
    ChRenameProbeBase::ArchiveOut(archive);
    archive << CHNVP(value);
}

void ChRenameProbe::ArchiveIn(ChArchiveIn& archive) {
    observed_version = archive.VersionRead<ChRenameProbe>();
    ChRenameProbeBase::ArchiveIn(archive);
    archive >> CHNVP(value);
}
}  // namespace chrono
