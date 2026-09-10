#pragma once
#include "SourcePartWallSetup.h"
namespace crash::cases::source_part_wall::detail {
bool ValidWallSettings(const SourcePartWallSettings&) noexcept;
SourcePartWallReport CertifyWallPenalty(const tl::fea::ShellBatchBinding&,const contact::NodalWallWeights&,
    const SourcePartWallSettings&,SourcePartWallCertificate*);
} // namespace crash::cases::source_part_wall::detail
