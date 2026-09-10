#pragma once
#include "../../native/qeph/QephForceReference.h"
#include "../../native/t3/T3ForceReference.h"
#include <array>
namespace tl::qualification::layered_law1_native {
// Distinct native elastic sidecars: no plastic strain/rate history or law control.
using Points=std::array<std::array<double,5>,3>;
static_assert(sizeof(Points)==15*sizeof(double));
struct QephHistory {qeph::History shell;Points points{};};
struct T3History {t3::History shell;Points points{};};
struct QephTrial {qeph::ForceTrial shell;Points points{};};
struct T3Trial {t3::ForceTrial shell;Points points{};};
// Independent caller-owned native histories. No production evaluator, integrator,
// global solver publication or constitutive failure/plastic state is involved.
qeph::Status Evaluate(const qeph::Reference&,const QephHistory&,const qeph::PrescribedInterval&,QephTrial&) noexcept;
t3::Status Evaluate(const t3::Reference&,const T3History&,const t3::PrescribedInterval&,T3Trial&) noexcept;
} // namespace tl::qualification::layered_law1_native
