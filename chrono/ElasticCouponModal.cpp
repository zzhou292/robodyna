#include "ElasticCouponModal.h"

#include <Eigen/Eigenvalues>
#include <Eigen/SVD>

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

namespace crash::reference {
namespace {
using Matrix = Eigen::Matrix<double, kCouponFreeDofs, kCouponFreeDofs>;
using Vector = Eigen::Matrix<double, kCouponFreeDofs, 1>;
using Increment = std::array<double, kCouponFreeDofs>;
using Status = ElasticCouponStatus;
enum class ForceSource { kChrono, kTL };

Status Reject(const char* gate, double measured, double limit, std::string& diagnostic) {
    std::ostringstream explanation;
    explanation.precision(17);
    explanation << gate << ": measured " << measured << ", required limit " << limit;
    diagnostic = explanation.str();
    return Status::kAuditRejected;
}

Vector InverseRootMass(const ElasticCouponData& data) {
    Vector result;
    for (std::size_t free = 0; free < kCouponFreeNodes.size(); ++free) {
        const auto& mass = data.nodal_mass[kCouponFreeNodes[free]];
        for (std::size_t c = 0; c < 6; ++c)
            result(6 * free + c) = 1 / std::sqrt(c < 3 ? mass.mass : mass.physical_tangential_inertia);
    }
    return result;
}

Vector FreeForces(const ElasticCouponEvaluation& evaluation) {
    Vector result;
    for (std::size_t free = 0; free < kCouponFreeNodes.size(); ++free) {
        const auto n = kCouponFreeNodes[free];
        for (std::size_t c = 0; c < 6; ++c)
            result(6 * free + c) = tl::fea::reissner::detail::Component(
                c < 3 ? evaluation.force[n] : evaluation.couple[n], c % 3);
    }
    return result;
}

Status Forces(const ElasticCouponModel& model, const ElasticCouponConfiguration& configuration,
              ForceSource source, Vector& output, std::string& diagnostic) {
    ElasticCouponEvaluation evaluation;
    const auto status = source == ForceSource::kChrono
                            ? model.EvaluateChrono(configuration, evaluation, diagnostic)
                            : model.EvaluateTL(configuration, evaluation, diagnostic);
    if (status == Status::kSuccess) output = FreeForces(evaluation);
    return status;
}

// Positive stiffness sign K=-dF/dq; coordinates are independent WORLD virtual
// displacements/spins about this base, including at deformed configurations.
Status DifferenceJacobian(const ElasticCouponModel& model, const ElasticCouponConfiguration& base,
                          ForceSource source, double difference_scale, Matrix& output,
                          std::string& diagnostic) {
    Matrix candidate;
    for (std::size_t column = 0; column < kCouponFreeDofs; ++column) {
        Increment increment{};
        const double delta = difference_scale * (column % 6 < 3 ? kCouponTranslationDifference
                                                               : kCouponRotationDifference);
        increment[column] = delta;
        ElasticCouponConfiguration plus, minus;
        auto status = ApplyElasticCouponIncrement(base, increment, 1, plus, diagnostic);
        if (status != Status::kSuccess) return status;
        status = ApplyElasticCouponIncrement(base, increment, -1, minus, diagnostic);
        if (status != Status::kSuccess) return status;
        Vector force_plus, force_minus;
        status = Forces(model, plus, source, force_plus, diagnostic);
        if (status != Status::kSuccess) return status;
        status = Forces(model, minus, source, force_minus, diagnostic);
        if (status != Status::kSuccess) return status;
        candidate.col(column) = -(force_plus - force_minus) / (2 * delta);
    }
    if (!candidate.allFinite()) {
        diagnostic = "Coupon finite-difference Jacobian is nonfinite";
        return Status::kNonfiniteResult;
    }
    output = candidate;
    return Status::kSuccess;
}

Matrix MassScale(const Matrix& matrix, const Vector& inverse_root_mass) {
    return inverse_root_mass.asDiagonal() * matrix * inverse_root_mass.asDiagonal();
}

double OperatorNorm(const Matrix& matrix) {
    const Eigen::JacobiSVD<Matrix> decomposition(matrix);
    if (decomposition.info() != Eigen::Success || !decomposition.singularValues().allFinite())
        return std::numeric_limits<double>::quiet_NaN();
    return decomposition.singularValues()(0);
}

double RelativeError(const Matrix& a, const Matrix& b) {
    return (a - b).norm() / std::max(a.norm(), b.norm());
}

// Check each column: a large rotary/shear block must not hide a bad weaker
// translation column in one global norm. The positive clamped matrix ensures
// nonzero columns; unexpected zero denominators reject below.
double ColumnRelativeError(const Matrix& a, const Matrix& b) {
    double worst = 0;
    for (std::size_t column = 0; column < kCouponFreeDofs; ++column) {
        const double denominator = std::max(a.col(column).norm(), b.col(column).norm());
        if (!std::isfinite(denominator) || !(denominator > 0)) return std::numeric_limits<double>::infinity();
        const double error = (a.col(column) - b.col(column)).norm() / denominator;
        if (!std::isfinite(error)) return std::numeric_limits<double>::infinity();
        worst = std::max(worst, error);
    }
    return worst;
}

Status DirectionalCrossCheck(const ElasticCouponModel& model, const ElasticCouponConfiguration& base,
                             const Matrix& chrono_stiffness, const Vector& inverse_root_mass,
                             double& worst, std::string& diagnostic) {
    for (std::size_t direction = 0; direction < 3; ++direction) {
        Increment increment{};
        Vector delta;
        for (std::size_t c = 0; c < kCouponFreeDofs; ++c) {
            // Fixed mixed directions exercise noncoaxial rotations and every
            // translation; no random seed, eigenvector reuse or zero blocks.
            const double sign = ((c * (2 * direction + 1) + direction) % 7 < 3) ? -1 : 1;
            increment[c] = sign * (1 + .1 * ((c + direction) % 3)) *
                           (c % 6 < 3 ? kCouponTranslationDifference : kCouponRotationDifference) * .5;
            delta(c) = increment[c];
        }
        ElasticCouponConfiguration plus, minus;
        auto status = ApplyElasticCouponIncrement(base, increment, 1, plus, diagnostic);
        if (status != Status::kSuccess) return status;
        status = ApplyElasticCouponIncrement(base, increment, -1, minus, diagnostic);
        if (status != Status::kSuccess) return status;
        Vector force_plus, force_minus;
        status = Forces(model, plus, ForceSource::kTL, force_plus, diagnostic);
        if (status != Status::kSuccess) return status;
        status = Forces(model, minus, ForceSource::kTL, force_minus, diagnostic);
        if (status != Status::kSuccess) return status;
        const Vector independent = inverse_root_mass.asDiagonal() * (-(force_plus - force_minus) / 2);
        const Vector predicted = inverse_root_mass.asDiagonal() * (chrono_stiffness * delta);
        const double error = (independent - predicted).norm() / std::max(independent.norm(), predicted.norm());
        if (!std::isfinite(error) || error > kCouponDerivativeTolerance)
            return Reject("TL/Chrono mixed directional derivative", error, kCouponDerivativeTolerance, diagnostic);
        worst = std::max(worst, error);
    }
    return Status::kSuccess;
}

Status SelectInitialMode(const ElasticCouponModel& model,
                         const Eigen::SelfAdjointEigenSolver<Matrix>& spectrum,
                         const Vector& inverse_root_mass, ElasticCouponModalReport& report,
                         std::string& diagnostic) {
    // This fixture is intended to release its lowest elastic bending mode.
    // A lower nonbending mode is diagnosed, not silently skipped or filtered.
    report.selected_mode = 0;
    const Vector mass_mode = spectrum.eigenvectors().col(0);
    const Vector mode = inverse_root_mass.asDiagonal() * mass_mode;
    for (std::size_t free = 0; free < kCouponFreeNodes.size(); ++free) {
        report.transverse_translation_mass_fraction += std::pow(mass_mode(6 * free + 2), 2);
        for (const auto c : {2, 3, 4}) report.out_of_plane_mass_fraction += std::pow(mass_mode(6 * free + c), 2);
    }
    if (report.out_of_plane_mass_fraction < .9)
        return Reject("Lowest mode is not predominantly out-of-plane bending (required fraction >=0.9)",
                      report.out_of_plane_mass_fraction, .9, diagnostic);
    if (report.transverse_translation_mass_fraction < .5)
        return Reject("Lowest mode lacks transverse translational motion (required fraction >=0.5)",
                      report.transverse_translation_mass_fraction, .5, diagnostic);
    // Free entries 2 and 3 are the physical tip nodes 4 and 5. A twist mode has
    // opposite tip signs and cannot be normalized by the positive mean tip z.
    const double tip_a = mode(14), tip_b = mode(20);
    const double mean_tip = .5 * (tip_a + tip_b);
    if (!std::isfinite(mean_tip) || std::abs(mean_tip) <= .5 * std::max(std::abs(tip_a), std::abs(tip_b)))
        return Reject("Lowest mode lacks coherent nonzero transverse tip motion", mean_tip, 0, diagnostic);
    const double scale = ElasticCouponData::initial_tip_displacement / mean_tip;
    for (std::size_t c = 0; c < kCouponFreeDofs; ++c) report.initial_mode_increment[c] = scale * mode(c);
    return ApplyElasticCouponIncrement(model.data().reference_configuration, report.initial_mode_increment,
                                      1, report.initial_configuration, diagnostic);
}
}  // namespace

ElasticCouponStatus MeasureElasticCouponOperatorNorm(const ElasticCouponModel& model,
                                                      const ElasticCouponConfiguration& configuration,
                                                      double& output, std::string& diagnostic) {
    Matrix stiffness;
    const auto status = DifferenceJacobian(model, configuration, ForceSource::kChrono, .5, stiffness, diagnostic);
    if (status != Status::kSuccess) return status;
    const double norm = OperatorNorm(MassScale(stiffness, InverseRootMass(model.data())));
    if (!std::isfinite(norm) || norm <= 0) return Reject("Invalid coupon operator norm", norm, 0, diagnostic);
    output = norm;
    diagnostic.clear();
    return Status::kSuccess;
}

ElasticCouponStatus AuditElasticCoupon(const ElasticCouponModel& model, ElasticCouponModalReport& output,
                                       std::string& diagnostic) {
    const auto& data = model.data();
    const auto& neutral = data.reference_configuration;
    const Vector inverse_root_mass = InverseRootMass(data);
    ElasticCouponModalReport report;
    Matrix coarse, fine, tl_fine;
    auto status = DifferenceJacobian(model, neutral, ForceSource::kChrono, 1, coarse, diagnostic);
    if (status != Status::kSuccess) return status;
    status = DifferenceJacobian(model, neutral, ForceSource::kChrono, .5, fine, diagnostic);
    if (status != Status::kSuccess) return status;
    status = DifferenceJacobian(model, neutral, ForceSource::kTL, .5, tl_fine, diagnostic);
    if (status != Status::kSuccess) return status;
    const Matrix a_coarse = MassScale(coarse, inverse_root_mass);
    const Matrix a_fine = MassScale(fine, inverse_root_mass);
    const Matrix a_tl = MassScale(tl_fine, inverse_root_mass);
    report.reference_symmetry_error = std::max(RelativeError(a_fine, a_fine.transpose()),
                                               RelativeError(a_coarse, a_coarse.transpose()));
    if (!std::isfinite(report.reference_symmetry_error) || report.reference_symmetry_error > kCouponDerivativeTolerance)
        return Reject("Stress-free mass-scaled symmetry", report.reference_symmetry_error,
                      kCouponDerivativeTolerance, diagnostic);
    report.mass_scaled_derivative_refinement_error = ColumnRelativeError(a_coarse, a_fine);
    if (report.mass_scaled_derivative_refinement_error > kCouponDerivativeTolerance)
        return Reject("FD halving column disagreement", report.mass_scaled_derivative_refinement_error,
                      kCouponDerivativeTolerance, diagnostic);
    report.tl_derivative_relative_error = ColumnRelativeError(a_tl, a_fine);
    if (report.tl_derivative_relative_error > kCouponDerivativeTolerance)
        return Reject("TL/Chrono all-DOF derivative disagreement", report.tl_derivative_relative_error,
                      kCouponDerivativeTolerance, diagnostic);
    // Only the stress-free matrix is symmetrized, after the independent skew
    // gate above. Off-reference operator norms below retain every entry.
    const Matrix symmetric = .5 * (a_fine + a_fine.transpose());
    const Eigen::SelfAdjointEigenSolver<Matrix> spectrum(symmetric);
    const Eigen::SelfAdjointEigenSolver<Matrix> coarse_spectrum(.5 * (a_coarse + a_coarse.transpose()));
    if (spectrum.info() != Eigen::Success || coarse_spectrum.info() != Eigen::Success ||
        !spectrum.eigenvalues().allFinite() || !coarse_spectrum.eigenvalues().allFinite()) {
        diagnostic = "Clamped coupon eigensolver failed";
        return Status::kModalFailure;
    }
    const double largest = spectrum.eigenvalues()(kCouponFreeDofs - 1);
    const double smallest = spectrum.eigenvalues()(0);
    const double uncertainty = OperatorNorm(a_coarse - a_fine);
    const double positive_floor = std::max(1e-10 * largest, 10 * uncertainty);
    if (!std::isfinite(positive_floor) || !(smallest > positive_floor) || coarse_spectrum.eigenvalues()(0) <= 0)
        return Reject("Unresolved zero/negative clamped mode (minimum must exceed FD uncertainty)",
                      smallest, positive_floor, diagnostic);
    report.maximum_frequency_refinement_error =
        std::abs(std::sqrt(largest / coarse_spectrum.eigenvalues()(kCouponFreeDofs - 1)) - 1);
    if (report.maximum_frequency_refinement_error > kCouponFrequencyRefinementTolerance)
        return Reject("Maximum frequency FD-halving change", report.maximum_frequency_refinement_error,
                      kCouponFrequencyRefinementTolerance, diagnostic);
    report.eigen_residual = (symmetric * spectrum.eigenvectors() -
                             spectrum.eigenvectors() * spectrum.eigenvalues().asDiagonal()).norm() / symmetric.norm();
    if (!std::isfinite(report.eigen_residual) || report.eigen_residual > 1e-10)
        return Reject("Mass-normalized eigen residual", report.eigen_residual, 1e-10, diagnostic);
    for (std::size_t c = 0; c < kCouponFreeDofs; ++c) report.squared_frequency[c] = spectrum.eigenvalues()(c);
    status = SelectInitialMode(model, spectrum, inverse_root_mass, report, diagnostic);
    if (status != Status::kSuccess) return status;
    for (std::size_t sample = 0; sample < report.sampled_amplitude.size(); ++sample) {
        ElasticCouponConfiguration configuration;
        status = ApplyElasticCouponIncrement(neutral, report.initial_mode_increment, report.sampled_amplitude[sample],
                                            configuration, diagnostic);
        if (status != Status::kSuccess) return status;
        Matrix stiffness;
        if (sample == 0) stiffness = fine;
        else {
            status = DifferenceJacobian(model, configuration, ForceSource::kChrono, .5, stiffness, diagnostic);
            if (status != Status::kSuccess) return status;
        }
        const double norm = OperatorNorm(MassScale(stiffness, inverse_root_mass));
        if (!std::isfinite(norm) || norm <= 0) return Reject("Sampled operator norm", norm, 0, diagnostic);
        report.sampled_operator_norm[sample] = norm;
        report.sampled_norm_maximum = std::max(report.sampled_norm_maximum, norm);
        status = DirectionalCrossCheck(model, configuration, stiffness, inverse_root_mass,
                                       report.tl_directional_relative_error, diagnostic);
        if (status != Status::kSuccess) return status;
    }
    report.monitored_norm_limit = 2 * report.sampled_norm_maximum;
    report.membrane_wave_speed = std::sqrt(data.young_modulus / (data.density * (1 - data.poisson_ratio * data.poisson_ratio)));
    report.shortest_reference_edge = std::numeric_limits<double>::infinity();
    for (const auto& connectivity : data.connectivity)
        for (std::size_t n = 0; n < 4; ++n) {
            const auto edge = tl::fea::reissner::detail::Subtract(neutral.position[connectivity[n]],
                                                                neutral.position[connectivity[(n + 1) % 4]]);
            report.shortest_reference_edge = std::min(report.shortest_reference_edge,
                                                       std::sqrt(tl::fea::reissner::detail::Dot(edge, edge)));
        }
    const double shear_modulus = data.young_modulus / (2 * (1 + data.poisson_ratio));
    report.spectral_step_limit = .1 / std::sqrt(report.monitored_norm_limit);
    report.wave_step_limit = .1 * report.shortest_reference_edge / report.membrane_wave_speed;
    report.rotary_step_limit = .1 * data.thickness / std::sqrt(12 * shear_modulus / data.density);
    report.proposed_step_limit = std::min({report.spectral_step_limit, report.wave_step_limit, report.rotary_step_limit});
    report.first_mode_angular_frequency = std::sqrt(smallest);
    const double half_period = std::acos(-1.0) / report.first_mode_angular_frequency;
    const double steps = std::ceil(half_period / report.proposed_step_limit);
    if (!std::isfinite(steps) || steps < 1 || steps > 1000000)
        return Reject("Coupon half-period exceeds one-million-step startup cap", steps, 1000000, diagnostic);
    report.step_count = static_cast<std::uint64_t>(steps);
    report.time_step = half_period / report.step_count;
    if (report.time_step > report.proposed_step_limit) report.time_step = half_period / ++report.step_count;
    report.horizon = report.step_count * report.time_step;
    ElasticCouponEvaluation initial;
    status = model.EvaluateTL(report.initial_configuration, initial, diagnostic);
    if (status != Status::kSuccess) return status;
    if (!std::isfinite(initial.energy) || initial.energy <= 0)
        return Reject("Initial bending strain energy must be positive", initial.energy, 0, diagnostic);
    report.initial_energy = initial.energy;
    output = report;
    diagnostic.clear();
    return Status::kSuccess;
}

}  // namespace crash::reference
