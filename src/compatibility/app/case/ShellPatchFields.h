#pragma once

#include "chrono/ElasticCouponModel.h"
#include "chrono/AcceptedSurfaceMesh.h"
#include "output/ArtifactIO.h"

namespace crash::case_data {
// Shared accepted-output serialization only. Schema/association admission is
// case-owned. Original coupon member order and numeric values are preserved.
void AppendShellElements(output::Document&,const tl::fea::reissner::ShellResult*,
                         std::size_t count,const std::uint64_t* parent_ids);
void AppendShellReference(output::Document&,const reference::ElasticCouponData&);
void AppendSurfaceBinding(output::Document&,const visual::Binding&);
} // namespace crash::case_data
