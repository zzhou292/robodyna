// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "radioss_type25/source_nodal/CorrectionTypes.h"
namespace tlfea::contact::radioss_type25::source_nodal::correction {
// Native-storage-order EightSlot STIFINT_ICONTROL, then ordered TYPE24 tail.
// Order, material/property operands and complete source coverage belong to the
// source binder. Empty secondary span declares no applicable TYPE24 occurrences.
Report Preflight(const Input&, Limits, Forecast&) noexcept;
// Startup-only bounded host composition. Public output is unchanged on failure;
// scratch may be partial. No sorting, allocations or independent state owner.
Report Apply(const Input&, Limits, void* scratch, std::size_t scratch_bytes, Output) noexcept;
// Pure native-pressure factor. Inactive rows do not consume material fields;
// active invalid/nonfinite inputs leave the output unchanged.
Status EvaluateFactor(const Solid&, FactorResult*) noexcept;
// A value-level permutation-equivalence certificate, not source authority or
// evidence that a supplied roster is complete. Every controlled incidence of
// each node must have the same factor bits, including the sign of zero. Only
// under successful source-bound certification may a binder substitute another
// complete solid order for the native IXS order. TYPE24 order is not certified.
Report PreflightOrderCertificate(const OrderInput&, Limits, Forecast&) noexcept;
OrderReport CertifyOrder(const OrderInput&, Limits, void* scratch,
                        std::size_t scratch_bytes, OrderCertificate*) noexcept;

}
