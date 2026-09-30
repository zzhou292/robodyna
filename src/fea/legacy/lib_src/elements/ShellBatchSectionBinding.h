#pragma once
#include "ShellBatchPlasticityBinding.h"

namespace tl::fea {
// Neutral names for new heterogeneous callers; existing public names remain.
using ShellBatchSectionBinding=ShellBatchPlasticityBinding;
using ShellSectionMaterialInput=ShellPlasticityMaterialInput;
using ShellSectionCurveInput=ShellPlasticityCurveInput;
using ShellSectionInput=ShellPlasticitySectionInput;
using ShellSectionParentInput=ShellPlasticityParentInput;
using ShellBatchSectionBindingInput=ShellBatchPlasticityBindingInput;
using ShellSectionBindingStatus=ShellPlasticityBindingStatus;
using ShellSectionBindingReport=ShellPlasticityBindingReport;
using ShellSectionCatalogLimits=ShellPlasticityCatalogLimits;
} // namespace tl::fea
