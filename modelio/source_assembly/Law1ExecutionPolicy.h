#pragma once
#include "SourceLaw1Driver.h"
#include "SourceReferenceMetric.h"
#include "lib_src/elements/ShellParentExecution.h"
#include "lib_src/elements/ShellBatchPlasticityBinding.h"

namespace crash::modelio::assembly {
enum class Law1ExecutionProfile { LegacyLayered, NativeA62OrdinaryNpt0 };
// Chosen numerical execution domain, not an observation of unspecified Engine
// controls: explicit/default shell context, virgin OFFG1, no element-freeze
// transition, no implicit/thermal/XFEM/restart. Native default internal drilling
// is0 here; overrides require a separate policy/qualification.
class Law1ExecutionPolicy {
  public:
    static Law1ExecutionPolicy Resolve(Law1ExecutionProfile profile, const SourceUnits& units,
                                       const QephReferenceMetric& metric) {
        if (profile == Law1ExecutionProfile::LegacyLayered) return {profile, 1};
        output::Require(profile == Law1ExecutionProfile::NativeA62OrdinaryNpt0,
                        "Unknown LAW1 source execution profile");
        const auto source = QephReferenceMetric::Resolve(QephMetricProfile::AuthenticatedSourceLength, units);
        output::Require(metric.profile() == QephMetricProfile::AuthenticatedSourceLength &&
            output::Bits(metric.working_length_m()) == output::Bits(source.working_length_m()),
            "Native LAW1 requires independently authenticated QEPH and coefficient source lengths");
        return {profile, source.working_length_m()};
    }
    Law1ExecutionProfile profile() const noexcept { return profile_; }
    double coefficient_working_length_m() const noexcept { return length_; }
    bool requires_ordinary_explicit_defaults() const noexcept {
        return profile_ == Law1ExecutionProfile::NativeA62OrdinaryNpt0;
    }
    tl::fea::ShellParentExecution Parent(const SourceLaw1Driver& driver,
        tl::fea::ShellBindingFamily family, tl::fea::ShellSectionFormulation formulation) const {
        tl::fea::ShellParentExecution result;
        if (profile_ == Law1ExecutionProfile::LegacyLayered || driver.status() == Law1DriverStatus::NotElastic) {
            return result;
        }
        output::Require(driver.available() &&
            (family == tl::fea::ShellBindingFamily::Qeph || family == tl::fea::ShellBindingFamily::T3) &&
            formulation == tl::fea::ShellSectionFormulation::LayeredNip3,
            "Native LAW1 requires resolved centered ordinary QEPH/T3 source declarations");
        result.policy = tl::fea::ShellParentExecutionPolicy::GlobalLaw1Npt0;
        result.global_law1 = {tl::fea::ShellLaw1Thickness::Accepted, length_};
        return result;
    }
  private:
    Law1ExecutionPolicy(Law1ExecutionProfile profile, double length) : profile_(profile), length_(length) {}
    Law1ExecutionProfile profile_;
    double length_;
};
} // namespace crash::modelio::assembly
