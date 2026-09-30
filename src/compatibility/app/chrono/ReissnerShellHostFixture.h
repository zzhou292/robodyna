#pragma once

// Host-only integration utilities shared by numerical parity and measurement
// executables. No CUDA kernel or runtime allocation is defined in this header.
#include "ReissnerReferenceFixture.h"
#include "ReissnerShellSetup.h"
#include "lib_src/elements/ReissnerShellForce.h"

#ifndef CH_REISSNER_CONSISTENT_FRAME_REFERENCE
#error "Complete CUDA forces must be checked against the qualified coherent Chrono reference"
#endif

namespace crash::qualification {
namespace tl_shell = tl::fea::reissner;

inline tl_shell::ShellConfiguration ToShell(const Frames& frames) {
    tl_shell::ShellConfiguration result;
    for (unsigned n = 0; n < 4; ++n) {
        result.position[n] = {frames.x[n].x(), frames.x[n].y(), frames.x[n].z()};
        result.rotation[n] = {frames.q[n].e0(), frames.q[n].e1(), frames.q[n].e2(), frames.q[n].e3()};
    }
    return result;
}

inline Evaluation ToEvaluation(const tl_shell::ShellResult& result) {
    Evaluation value;
    value.energy = result.energy;
    value.bending_energy = result.bending_energy;
    for (unsigned n = 0; n < 4; ++n) {
        value.force[n] = Vec(result.force[n].x, result.force[n].y, result.force[n].z);
        value.couple[n] = Vec(result.couple[n].x, result.couple[n].y, result.couple[n].z);
        for (unsigned c = 0; c < 12; ++c) {
            value.strain[n][c] = result.strain[n][c];
            value.stress[n][c] = result.resultant[n][c];
        }
    }
    return value;
}

inline Frames GeneralShellDeformation() {
    auto frames = Bending(.12, .002);
    for (unsigned n = 0; n < 4; ++n) {
        const auto p = Neutral().x[n];
        frames.x[n].y() += .008 * p.x() * p.y();
        frames.x[n].z() += .015 * p.x() * p.y();
        frames.q[n] = frames.q[n] * Rotation(-.08 * p.y(), Vec(1, 0, 0)) *
                      Rotation(.025 * p.x() * p.y(), Vec(0, 0, 1));
    }
    return frames;
}

inline void ExpectShellAgreement(const tl_shell::ShellResult& actual, const Evaluation& expected) {
    // Declared before execution. Direct arithmetic budgets are separate from
    // independent objectivity/energy derivative gates in the owning fixture.
    constexpr double direct = 3e-12;
    for (unsigned n = 0; n < 4; ++n) {
        for (unsigned c = 0; c < 3; ++c) {
            EXPECT_NEAR(tl_shell::detail::Component(actual.force[n], c), expected.force[n][c],
                        direct * (kForceScale + std::abs(expected.force[n][c])));
            EXPECT_NEAR(tl_shell::detail::Component(actual.couple[n], c), expected.couple[n][c],
                        direct * (kMomentScale + std::abs(expected.couple[n][c])));
        }
        for (unsigned c = 0; c < 12; ++c) {
            EXPECT_NEAR(actual.strain[n][c], expected.strain[n][c], direct * (1 + std::abs(expected.strain[n][c])));
            const double section_scale = c < 6 ? kC : kD;
            EXPECT_NEAR(actual.resultant[n][c], expected.stress[n][c], direct * (section_scale + std::abs(expected.stress[n][c])));
        }
    }
    EXPECT_NEAR(actual.energy, expected.energy, direct * (1 + std::abs(expected.energy)));
    EXPECT_NEAR(actual.bending_energy, expected.bending_energy, direct * (1 + std::abs(expected.bending_energy)));
}

// Reused unchanged dimensional gates and perturbation order from the original
// CUDA fixture. The caller supplies the actual operation under qualification.
template <class Sample>
inline void CheckShellEnergyDerivatives(const Frames& frames, Sample&& sample, unsigned first_dof = 0) {
    const auto base = sample(frames);
    for (unsigned node = 0; node < 4; ++node) for (unsigned dof = first_dof; dof < 6; ++dof) {
        SCOPED_TRACE(node);
        SCOPED_TRACE(dof);
        Vec axis(0, 0, 0); axis[dof % 3] = 1;
        const double restoring = tl_shell::detail::Component(dof < 3 ? base.force[node] : base.couple[node], dof % 3);
        double error[2]; unsigned level = 0;
        for (double h : {2e-5, 1e-5}) {
            double energy[2];
            for (unsigned sign = 0; sign < 2; ++sign) {
                const double delta = sign ? h : -h;
                auto changed = frames;
                if (dof < 3) changed.x[node][dof] += delta;
                else changed.q[node] = Rotation(delta, axis) * frames.q[node];
                energy[sign] = sample(changed).energy;
            }
            const double derivative = (energy[1] - energy[0]) / (2 * h);
            error[level++] = std::abs(derivative + restoring);
            EXPECT_LE(error[level-1], 1e-7 + 2e-6 * std::max(std::abs(derivative), std::abs(restoring)));
        }
        EXPECT_LE(error[1], .35 * error[0] + 2e-7 * (1 + std::abs(restoring)));
    }
    ExpectShellAgreement(sample(frames), ToEvaluation(base));
}
}  // namespace crash::qualification

