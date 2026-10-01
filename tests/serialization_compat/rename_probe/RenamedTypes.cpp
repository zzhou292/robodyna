#include "tests/serialization_compat/rename_probe/RenamedTypes.h"

namespace chrono {
CH_FACTORY_REGISTER_TAG(::robodyna::serialization_compat::RbRenameProbe, "ChRenameProbe", rb_rename_probe)
CH_UPCASTING_TAGGED(::robodyna::serialization_compat::RbRenameProbe,
                   ::robodyna::serialization_compat::RbRenameProbeBase,
                   "ChRenameProbe", "ChRenameProbeBase", rb_rename_probe_to_base)
}  // namespace chrono

namespace robodyna::serialization_compat {
using chrono::make_ChNameValue;

void RbRenameProbeBase::ArchiveOut(chrono::ChArchiveOut& archive) {
    // Authentic GCC/Itanium identity captured by the pre-change producer. This
    // unregistered abstract base must not inherit the new C++ RTTI spelling.
    archive.VersionWrite<RbRenameProbeBase>("N6chrono17ChRenameProbeBaseE");
    archive << CHNVP(base_value);
}

void RbRenameProbeBase::ArchiveIn(chrono::ChArchiveIn& archive) {
    observed_base_version = archive.VersionRead<RbRenameProbeBase>("N6chrono17ChRenameProbeBaseE");
    archive >> CHNVP(base_value);
}

void RbRenameProbe::ArchiveOut(chrono::ChArchiveOut& archive) {
    // A registered type already gets its version identity from the canonical
    // factory tag; no extra version-name override or second registration is used.
    archive.VersionWrite<RbRenameProbe>();
    RbRenameProbeBase::ArchiveOut(archive);
    archive << CHNVP(value);
}

void RbRenameProbe::ArchiveIn(chrono::ChArchiveIn& archive) {
    observed_version = archive.VersionRead<RbRenameProbe>();
    RbRenameProbeBase::ArchiveIn(archive);
    archive >> CHNVP(value);
}
}  // namespace robodyna::serialization_compat
