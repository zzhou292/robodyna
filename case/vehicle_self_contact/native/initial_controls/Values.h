#pragma once
#include "../InitializerControlsSource.h"
#include "lib_src/collision/radioss_type25/source_shells/Values.h"
namespace crash::cases::vehicle_self_contact::native::initial_controls::detail {
// Numeric/source-table seam for focused qualification. Preparation authenticates
// the complete immutable graph before using these borrowed tables.
GapScalars ResolveGaps(tl::util::ConstView<post_gapm::PhysicalOwner>,
    tl::util::ConstView<gap_operands::Binding>,tl::util::ConstView<n::source_gaps::PhysicalShell>,
    const n::source_gaps::Profile&,const n::source_gaps::Report&);
n::TransactionConfig ResolveSelfLaw(const modelio::self_contact::Data&,n::UnitScale);
RawControls ResolveOriginalControls(const modelio::self_contact::Data&);
}
