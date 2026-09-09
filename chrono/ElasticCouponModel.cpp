#include "ElasticCouponModel.h"

#include "ReissnerShellSetup.h"
#include "elements/ReissnerShellForce.h"
#include "math/Quaternion.h"
#include "chrono/core/ChTypes.h"
#include "chrono/fea/ChElementShellReissner4.h"
#include "chrono/fea/ChMaterialShellReissner.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChNodeFEAxyzrot.h"
#include "chrono/physics/ChSystemSMC.h"

#include <cmath>
#include <stdexcept>

#ifndef CH_REISSNER_CONSISTENT_FRAME_REFERENCE
#error "Elastic coupon requires the owning coherent-force Chrono core target."
#endif

namespace crash::reference {
namespace {
namespace tlr = tl::fea::reissner;
using Node = chrono::fea::ChNodeFEAxyzrot;
using Element = chrono::fea::ChElementShellReissner4;

chrono::ChVector3d ChronoVector(tlr::Vec3 value) { return {value.x, value.y, value.z}; }
chrono::ChQuaterniond ChronoRotation(tlr::Quaternion value) { return {value.w, value.x, value.y, value.z}; }
tlr::Vec3 CopyVector(const chrono::ChVector3d& value) { return {value.x(), value.y(), value.z()}; }

bool ValidConfiguration(const ElasticCouponConfiguration& configuration) {
    for (std::size_t n = 0; n < kCouponNodes; ++n)
        if (!tlr::detail::Finite(configuration.position[n]) ||
            !tl::math::UnitQuaternion(configuration.rotation[n]))
            return false;
    return true;
}

// Assembly uses one contribution per element, including shared physical nodes
// and fixed-node reactions. No Chrono mass/inertia enters this operation.
bool Assemble(const ElasticCouponData& data, ElasticCouponEvaluation& result) {
    for (std::size_t e = 0; e < kCouponElements; ++e) {
        const auto& element = result.element[e];
        for (std::size_t local = 0; local < 4; ++local) {
            const auto n = data.connectivity[e][local];
            result.force[n] = tlr::detail::Add(result.force[n], element.force[local]);
            result.couple[n] = tlr::detail::Add(result.couple[n], element.couple[local]);
        }
        result.energy += element.energy;
        result.bending_energy += element.bending_energy;
    }
    for (std::size_t n = 0; n < kCouponNodes; ++n)
        if (!tlr::detail::Finite(result.force[n]) || !tlr::detail::Finite(result.couple[n])) return false;
    return std::isfinite(result.energy) && std::isfinite(result.bending_energy);
}

tlr::ShellResult ReadChronoElement(Element& element, const ElasticCouponConfiguration& configuration,
                                  const std::array<std::size_t, 4>& connectivity) {
    chrono::ChVectorDynamic<> force(24);
    element.ComputeInternalForces(force);
    if (!force.allFinite()) throw std::runtime_error("Chrono coupon force is nonfinite");
    tlr::ShellResult result;
    for (std::size_t n = 0; n < 4; ++n) {
        result.force[n] = {force(6 * n), force(6 * n + 1), force(6 * n + 2)};
        const chrono::ChVector3d local(force(6 * n + 3), force(6 * n + 4), force(6 * n + 5));
        // Chrono returns body couples; all modal rows and TL results are WORLD.
        result.couple[n] = CopyVector(ChronoRotation(configuration.rotation[connectivity[n]]).Rotate(local));
    }
    // Same implementation-sensitive section diagnostic used by the qualified
    // Reissner reference checks; centered linear elasticity U=1/2 integral(e.C.e).
    for (std::size_t p = 0; p < 4; ++p) {
        const std::array<chrono::ChVector3d, 4> strain{{element.eps_tilde_1_i[p], element.eps_tilde_2_i[p],
                                                     element.k_tilde_1_i[p], element.k_tilde_2_i[p]}};
        const double weight = element.alpha_i[p] * Element::w_i[p];
        if (!std::isfinite(weight) || weight <= 0) throw std::runtime_error("Invalid Chrono reference area");
        for (std::size_t c = 0; c < 12; ++c) {
            const double deformation = strain[c / 3][c % 3];
            const double resultant = element.stress_i[p](c);
            if (!std::isfinite(deformation) || !std::isfinite(resultant))
                throw std::runtime_error("Nonfinite Chrono section response");
            result.strain[p][c] = deformation;
            result.resultant[p][c] = resultant;
            const double energy = .5 * weight * deformation * resultant;
            result.energy += energy;
            if (c >= 6) result.bending_energy += energy;
        }
    }
    return result;
}
}  // namespace

struct ElasticCouponModel::Impl {
    ElasticCouponData data;
    chrono::ChSystemSMC system;
    std::shared_ptr<chrono::fea::ChMesh> mesh;
    std::array<std::shared_ptr<Node>, kCouponNodes> nodes;
    std::array<std::shared_ptr<Element>, kCouponElements> elements;

    Impl() {
        // Follow Chrono demo_FEA_shellsReissner.cpp's mesh/node/layer ownership.
        // This system never advances; it exists only to initialize the donor.
        system.SetNumThreads(1, 1, 1);
        mesh = chrono_types::make_shared<chrono::fea::ChMesh>();
        mesh->SetAutomaticGravity(false);
        system.Add(mesh);
        data.reference_configuration.position = {{{.1, .05, 0}, {0, .05, 0}, {0, -.05, 0},
                                                  {.1, -.05, 0}, {.2, .05, 0}, {.2, -.05, 0}}};
        data.connectivity = {{{0, 1, 2, 3}, {4, 0, 3, 5}}};
        auto elasticity = chrono_types::make_shared<chrono::fea::ChElasticityReissnerIsothropic>(
            data.young_modulus, data.poisson_ratio, 5.0 / 6.0, .01);
        auto material = chrono_types::make_shared<chrono::fea::ChMaterialShellReissner>(elasticity);
        material->SetDensity(data.density);
        for (std::size_t n = 0; n < kCouponNodes; ++n) {
            nodes[n] = chrono_types::make_shared<Node>(chrono::ChFrame<>(
                ChronoVector(data.reference_configuration.position[n]),
                ChronoRotation(data.reference_configuration.rotation[n])));
            nodes[n]->SetMass(0);
            nodes[n]->GetInertia().fillDiagonal(0);
            nodes[n]->SetFixed(data.fixed[n]);
            mesh->AddNode(nodes[n]);
        }
        for (std::size_t e = 0; e < kCouponElements; ++e) {
            const auto& c = data.connectivity[e];
            elements[e] = chrono_types::make_shared<Element>();
            elements[e]->SetNodes(nodes[c[0]], nodes[c[1]], nodes[c[2]], nodes[c[3]]);
            elements[e]->AddLayer(data.thickness, 0, material);
            mesh->AddElement(elements[e]);
        }
        system.Setup();  // Exactly once, before any current-configuration change.
        for (std::size_t e = 0; e < kCouponElements; ++e) {
            CopyReissnerShellSetup(*elements[e], data.reference[e], data.section[e]);
            if (tlr::ComputeShellMass(data.reference[e], data.section[e],
                                     tlr::ShellDrillingInertiaPolicy::kEqualPhysicalTangential,
                                     data.element_mass[e]) != tlr::ShellMassStatus::kSuccess)
                throw std::runtime_error("Coupon physical shell mass preparation failed");
            for (std::size_t local = 0; local < 4; ++local) {
                auto& sum = data.nodal_mass[data.connectivity[e][local]];
                const auto& contribution = data.element_mass[e].node[local];
                sum.area += contribution.area;
                sum.mass += contribution.mass;
                sum.physical_tangential_inertia += contribution.physical_tangential_inertia;
                sum.artificial_drilling_inertia += contribution.artificial_drilling_inertia;
            }
        }
        for (std::size_t n = 0; n < kCouponNodes; ++n) {
            const auto& mass = data.nodal_mass[n];
            if (!tlr::mass_detail::ValidMass(mass) ||
                mass.artificial_drilling_inertia != mass.physical_tangential_inertia)
                throw std::runtime_error("Coupon assembled inertia is not the declared positive J*I");
            if (!data.fixed[n]) {
                data.inverse_mass[n] = 1 / mass.mass;
                data.inverse_isotropic_inertia[n] = 1 / mass.physical_tangential_inertia;
            }
        }
    }
};

ElasticCouponModel::ElasticCouponModel() : impl_(std::make_unique<Impl>()) {}
ElasticCouponModel::~ElasticCouponModel() = default;
const ElasticCouponData& ElasticCouponModel::data() const { return impl_->data; }

ElasticCouponStatus ElasticCouponModel::EvaluateChrono(const ElasticCouponConfiguration& configuration,
                                                       ElasticCouponEvaluation& output,
                                                       std::string& diagnostic) const {
    if (!ValidConfiguration(configuration)) {
        diagnostic = "Chrono coupon evaluation requires finite positions and unit quaternions";
        return ElasticCouponStatus::kInvalidConfiguration;
    }
    ElasticCouponEvaluation candidate;
    try {
        for (std::size_t n = 0; n < kCouponNodes; ++n) {
            impl_->nodes[n]->SetPos(ChronoVector(configuration.position[n]));
            impl_->nodes[n]->SetRot(ChronoRotation(configuration.rotation[n]));
        }
        for (std::size_t e = 0; e < kCouponElements; ++e)
            candidate.element[e] = ReadChronoElement(*impl_->elements[e], configuration, data().connectivity[e]);
    } catch (const std::exception& error) {
        diagnostic = std::string("Chrono coupon force rejected: ") + error.what();
        return ElasticCouponStatus::kForceFailure;
    }
    if (!Assemble(data(), candidate)) {
        diagnostic = "Chrono coupon assembly produced a nonfinite result";
        return ElasticCouponStatus::kNonfiniteResult;
    }
    output = candidate;
    diagnostic.clear();
    return ElasticCouponStatus::kSuccess;
}

ElasticCouponStatus ElasticCouponModel::EvaluateTL(const ElasticCouponConfiguration& configuration,
                                                   ElasticCouponEvaluation& output,
                                                   std::string& diagnostic) const {
    if (!ValidConfiguration(configuration)) {
        diagnostic = "TL coupon evaluation requires finite positions and unit quaternions";
        return ElasticCouponStatus::kInvalidConfiguration;
    }
    ElasticCouponEvaluation candidate;
    for (std::size_t e = 0; e < kCouponElements; ++e) {
        tlr::ShellConfiguration local;
        for (std::size_t n = 0; n < 4; ++n) {
            const auto physical = data().connectivity[e][n];
            local.position[n] = configuration.position[physical];
            local.rotation[n] = configuration.rotation[physical];
        }
        const auto status = tlr::ComputeShellForce(data().reference[e], data().section[e], local, candidate.element[e]);
        if (status != tlr::ShellStatus::kSuccess) {
            diagnostic = "TL coupon element " + std::to_string(e) + " force status " +
                         std::to_string(static_cast<int>(status));
            return ElasticCouponStatus::kForceFailure;
        }
    }
    if (!Assemble(data(), candidate)) {
        diagnostic = "TL coupon assembly produced a nonfinite result";
        return ElasticCouponStatus::kNonfiniteResult;
    }
    output = candidate;
    diagnostic.clear();
    return ElasticCouponStatus::kSuccess;
}

ElasticCouponStatus ApplyElasticCouponIncrement(const ElasticCouponConfiguration& base,
                                               const std::array<double, kCouponFreeDofs>& increment,
                                               double scale, ElasticCouponConfiguration& output,
                                               std::string& diagnostic) {
    if (!ValidConfiguration(base) || !std::isfinite(scale)) {
        diagnostic = "Coupon increment requires a valid base and finite scale";
        return ElasticCouponStatus::kInvalidConfiguration;
    }
    auto candidate = base;
    for (std::size_t free = 0; free < kCouponFreeNodes.size(); ++free) {
        const auto n = kCouponFreeNodes[free];
        const tlr::Vec3 shift{scale * increment[6 * free], scale * increment[6 * free + 1],
                             scale * increment[6 * free + 2]};
        const double spin[3] = {scale * increment[6 * free + 3], scale * increment[6 * free + 4],
                               scale * increment[6 * free + 5]};
        candidate.position[n] = tlr::detail::Add(base.position[n], shift);
        if (!tlr::detail::Finite(candidate.position[n]) ||
            !tl::math::IncrementWorldRotation(base.rotation[n], spin, candidate.rotation[n])) {
            diagnostic = "Coupon increment produced invalid translation or world rotation";
            return ElasticCouponStatus::kInvalidConfiguration;
        }
    }
    output = candidate;
    diagnostic.clear();
    return ElasticCouponStatus::kSuccess;
}

}  // namespace crash::reference
