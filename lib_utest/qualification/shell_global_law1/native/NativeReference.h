// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_utest/qualification/native/qeph/QephForceReference.h"
#include "lib_utest/qualification/native/t3/T3ForceReference.h"
namespace tl::qualification::global_law1_native {
namespace q=tl::qualification::qeph;namespace t=tl::qualification::t3;
// Qualification only. Full existing NPT0 native sequences with explicit ITHK
// 0/1. All operands/results are in one consistent RAW native working system;
// these adapters never read a production policy or perform SI conversion.
q::Status Evaluate(const q::Reference&,const q::History&,const q::PrescribedInterval&,int ithk,q::ForceTrial&) noexcept;
t::Status Evaluate(const t::Reference&,const t::History&,const t::PrescribedInterval&,int ithk,t::ForceTrial&) noexcept;
// Actual complete native coefficient calls with benign fixed material/area.
// Observe only the defined THK0 channel; this is not full-force admission.
// Reference thickness>0, accepted thickness>=0, finite, ITHK exactly0 or1.
double QephThickness(double reference_thickness,double accepted_thickness,int ithk);
double T3Thickness(double reference_thickness,double accepted_thickness,int ithk);
double NativeEm20();
}
