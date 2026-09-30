#include "ReissnerShellSetup.h"

#include "chrono/fea/ChElementShellReissner4.h"
#include "chrono/fea/ChMaterialShellReissner.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <typeinfo>

namespace crash::reference {
namespace {
using Element = chrono::fea::ChElementShellReissner4;
using Vector = chrono::ChVector3d;
using Matrix = chrono::ChMatrix33<>;
using namespace tl::fea::reissner;

constexpr double kFrameTolerance = 1e-12;
constexpr double kGeometryTolerance = 1e-11;
constexpr double kMaximumAspectRatio = 100;

void Require(bool condition, const char* explanation) {
    if (!condition)
        throw std::invalid_argument(explanation);
}

Vec3 CopyVector(const Vector& value) {
    Require(std::isfinite(value.x()) && std::isfinite(value.y()) && std::isfinite(value.z()),
            "Reissner setup contains a nonfinite reference vector");
    return {value.x(), value.y(), value.z()};
}

Quaternion CopyQuaternion(const chrono::ChQuaternion<>& value) {
    const Quaternion result{value.e0(), value.e1(), value.e2(), value.e3()};
    Require(detail::Finite(result) && std::abs(detail::Dot(result, result) - 1) <= kFrameTolerance,
            "Reissner setup requires finite unit quaternions");
    return result;
}

Matrix3 CopyFrame(const Matrix& value) {
    const Matrix defect = value.transpose() * value - Matrix(1);
    Require(value.allFinite() && defect.allFinite() &&
                defect.cwiseAbs().maxCoeff() <= kFrameTolerance &&
                std::abs(value.determinant() - 1) <= kFrameTolerance,
            "Reissner setup requires proper reference frames");
    Matrix3 result;
    for (unsigned row = 0; row < 3; ++row)
        for (unsigned col = 0; col < 3; ++col)
            result.v[3 * row + col] = value(row, col);
    return result;
}

double RequireRectangle(const Element& element) {
    const auto& x = element.xa_0;
    for (unsigned n = 0; n < 4; ++n)
        CopyVector(x[n]);
    const Vector edge_u = x[0] - x[1];
    const Vector edge_v = x[0] - x[3];
    const double length_u = edge_u.Length();
    const double length_v = edge_v.Length();
    const double longest = std::max(length_u, length_v);
    const double shortest = std::min(length_u, length_v);
    Require(std::isfinite(longest) && shortest > 0 && longest / shortest <= kMaximumAspectRatio,
            "Reissner setup requires a positive rectangle with aspect ratio at most 100");
    Require(((x[2] - x[1]) - (x[3] - x[0])).Length() <= kGeometryTolerance * shortest &&
                std::abs(edge_u.Dot(edge_v)) <= kGeometryTolerance * length_u * length_v,
            "Reissner setup currently admits planar rectangles only; do not flatten or remesh input");
    const double area = length_u * length_v;
    Require(std::isfinite(area) && area > 0, "Reissner setup has invalid rectangle area");
    return area;
}

void CopyPoint(Element& element, const double natural[2], const Matrix& offset,
               const chrono::ChMatrixNM<double, 4, 2>& gradient,
               const Vector& strain1, const Vector& strain2, ShellPointReference& result) {
    result.natural[0] = natural[0];
    result.natural[1] = natural[1];
    Element::ShapeVector shape;
    element.ShapeFunctions(shape, natural[0], natural[1]);
    Require(shape.allFinite() && std::abs(shape.sum() - 1) <= kFrameTolerance && gradient.allFinite(),
            "Reissner setup has invalid shape functions or reference gradients");
    for (unsigned n = 0; n < 4; ++n) {
        result.shape[n] = shape(n);
        for (unsigned axis = 0; axis < 2; ++axis)
            result.gradient[n][axis] = gradient(n, axis);
    }
    result.frame_offset = CopyFrame(offset);
    result.strain0[0] = CopyVector(strain1);
    result.strain0[1] = CopyVector(strain2);
}

void RequireNaturalOrder() {
    // The ANS force operation depends on these source point identities. Guard
    // the currently-public mutable Chrono tables instead of silently remapping.
    constexpr double node[4][2] = {{1, 1}, {-1, 1}, {-1, -1}, {1, -1}};
    constexpr double ans[4][2] = {{0, 1}, {-1, 0}, {0, -1}, {1, 0}};
    const double a = 1 / std::sqrt(3.0);
    const double gauss[4][2] = {{-a, -a}, {a, -a}, {a, a}, {-a, a}};
    for (unsigned n = 0; n < 4; ++n) {
        for (unsigned axis = 0; axis < 2; ++axis)
            Require(Element::xi_n[n][axis] == node[n][axis] &&
                        Element::xi_A[n][axis] == ans[n][axis] &&
                        Element::xi_i[n][axis] == gauss[n][axis],
                    "Reissner setup uses unsupported natural node or integration-point order");
        Require(Element::w_i[n] == 1, "Reissner setup uses unsupported quadrature weights");
    }
}

ElasticSection CopySection(Element& element) {
    Require(element.GetNumLayers() == 1 && element.m_layers_z.size() == 2,
            "Reissner setup requires exactly one centered elastic layer");
    const auto& layer = element.GetLayer(0);
    const auto material = layer.GetMaterial();
    Require(material && typeid(*material) == typeid(chrono::fea::ChMaterialShellReissner),
            "Reissner setup requires the supported Chrono shell material type");
    Require(!material->GetPlasticity(), "Reissner setup does not admit plasticity; null history is not an elastic override");
    Require(!material->GetDamping(), "Reissner setup does not admit material damping");
    const auto elasticity = material->GetElasticity();
    Require(elasticity && typeid(*elasticity) == typeid(chrono::fea::ChElasticityReissnerIsothropic),
            "Reissner setup requires the supported isotropic linear elasticity type");
    const auto& isotropic = static_cast<const chrono::fea::ChElasticityReissnerIsothropic&>(*elasticity);
    ElasticSection result;
    result.thickness = layer.GetThickness();
    result.density = material->GetDensity();
    Require(std::isfinite(result.thickness) && result.thickness > 0 &&
                std::isfinite(result.density) && result.density > 0,
            "Reissner setup requires finite positive thickness and density");
    const double lower = element.m_layers_z[0], upper = element.m_layers_z[1];
    Require(lower == -0.5 * result.thickness && upper == 0.5 * result.thickness &&
                element.GetThickness() == result.thickness && std::isfinite(layer.GetFiberAngle()),
            "Reissner setup does not admit section offsets or inconsistent layer thickness");
    const double young = isotropic.GetYoungModulus(), poisson = isotropic.GetPoissonRatio();
    Require(std::isfinite(young) && young > 0 && std::isfinite(poisson) && poisson > -1 && poisson < .5 &&
                std::isfinite(isotropic.GetShearFactor()) && isotropic.GetShearFactor() > 0 &&
                std::isfinite(isotropic.GetTorqueFactor()) && isotropic.GetTorqueFactor() > 0,
            "Reissner setup requires positive elastic, shear and drilling stiffness with -1 < nu < 0.5");
    chrono::ChMatrixNM<double, 12, 12> stiffness;
    const Vector zero(0, 0, 0);
    material->ComputeStiffnessMatrix(stiffness, zero, zero, zero, zero, lower, upper, layer.GetFiberAngle());
    Require(stiffness.allFinite(), "Reissner setup produced a nonfinite section matrix");
    for (unsigned row = 0; row < 12; ++row) {
        Require(stiffness(row, row) > 0, "Reissner setup has nonpositive section stiffness");
        for (unsigned col = 0; col < 12; ++col)
            result.stiffness[12 * row + col] = stiffness(row, col);
    }
    // Verify all basis strains and a mixed state using the actual material's
    // stress operation. This catches matrix ordering/API drift at this boundary.
    for (unsigned sample = 0; sample < 13; ++sample) {
        Vector strain[4] = {zero, zero, zero, zero};
        for (unsigned col = 0; col < 12; ++col)
            strain[col / 3][col % 3] = sample < 12 ? (sample == col ? 1 : 0) : (static_cast<double>(col) - 5.5) / 7;
        Vector stress[4];
        material->ComputeStress(stress[0], stress[1], stress[2], stress[3],
                                strain[0], strain[1], strain[2], strain[3], lower, upper, layer.GetFiberAngle());
        for (unsigned row = 0; row < 12; ++row) {
            double predicted = 0, scale = 0;
            for (unsigned col = 0; col < 12; ++col) {
                const double term = stiffness(row, col) * strain[col / 3][col % 3];
                predicted += term;
                scale += std::abs(term);
            }
            const double actual = stress[row / 3][row % 3];
            Require(std::isfinite(predicted) && std::isfinite(scale) && std::isfinite(actual) && std::abs(predicted - actual) <=
                        kFrameTolerance * std::max(scale, std::abs(actual)),
                    "Reissner setup section matrix disagrees with actual stress operation");
        }
    }
    result.prepared = true;
    return result;
}

}  // namespace

void CopyReissnerShellSetup(Element& element, ShellReference& reference, ElasticSection& section) {
    Require(element.m_nodes.size() == 4, "Reissner setup requires four nodes");
    for (const auto& node : element.m_nodes)
        Require(static_cast<bool>(node), "Reissner setup requires initialized nodes");
    // These gradients are initialized to zero by the element constructor;
    // reject missing Setup before reading other, uninitialized rest caches.
    for (unsigned p = 0; p < 4; ++p)
        Require(element.L_alpha_beta_i[p].allFinite() && element.L_alpha_beta_i[p].squaredNorm() > 0 &&
                    element.L_alpha_beta_A[p].allFinite() && element.L_alpha_beta_A[p].squaredNorm() > 0,
                "Reissner setup requires completed normal Chrono Setup");
    RequireNaturalOrder();
    const double area = RequireRectangle(element);
    ShellReference candidate;
    Quaternion physical_rotation[4];
    for (unsigned n = 0; n < 4; ++n) {
        const auto& node = *element.m_nodes[n];
        candidate.initial_position[n] = CopyVector(element.xa_0[n]);
        candidate.initial_rotation[n] = CopyQuaternion(node.GetX0().GetRot());
        const auto current = CopyQuaternion(node.GetRot());
        const double sign = detail::Dot(current, candidate.initial_rotation[n]) < 0 ? -1 : 1;
        const auto difference = detail::Subtract(detail::Scale(current, sign), candidate.initial_rotation[n]);
        Require(node.GetPos() == element.xa_0[n] && node.GetX0().GetPos() == element.xa_0[n] &&
                    std::abs(difference.w) <= kFrameTolerance && std::abs(difference.x) <= kFrameTolerance &&
                    std::abs(difference.y) <= kFrameTolerance && std::abs(difference.z) <= kFrameTolerance,
                "Reissner setup must be copied before changing current reference frames");
        CopyFrame(element.iTa[n]);
        candidate.node_frame_offset[n] = CopyQuaternion(element.iTa[n].GetQuaternion());
        physical_rotation[n] = detail::Product(candidate.initial_rotation[n], candidate.node_frame_offset[n]);
    }
    MeanFrame initial_mean;
    Require(ComputeMeanFrame(physical_rotation, initial_mean) == Status::kSuccess,
            "Reissner setup physical directors are outside the admitted mean-frame chart");
    double integrated_area = 0;
    for (unsigned p = 0; p < 4; ++p) {
        auto& gauss = candidate.gauss[p];
        CopyPoint(element, Element::xi_i[p], element.iTa_i[p], element.L_alpha_beta_i[p],
                  element.eps_tilde_1_0_i[p], element.eps_tilde_2_0_i[p], gauss);
        gauss.curvature0[0] = CopyVector(element.k_tilde_1_0_i[p]);
        gauss.curvature0[1] = CopyVector(element.k_tilde_2_0_i[p]);
        gauss.area_weight = element.alpha_i[p] * Element::w_i[p];
        Require(std::isfinite(gauss.area_weight) && gauss.area_weight > 0 &&
                    std::abs(4 * gauss.area_weight - area) <= kGeometryTolerance * area,
                "Reissner setup has an invalid or inconsistent positive reference mapping");
        integrated_area += gauss.area_weight;
        CopyPoint(element, Element::xi_A[p], element.iTa_A[p], element.L_alpha_beta_A[p],
                  element.eps_tilde_1_0_A[p], element.eps_tilde_2_0_A[p], candidate.ans[p]);
    }
    Require(std::abs(integrated_area - area) <= kGeometryTolerance * area,
            "Reissner setup quadrature area disagrees with rectangle geometry");
    ElasticSection material_candidate = CopySection(element);
    candidate.prepared = true;
    reference = candidate;
    section = material_candidate;
}

}  // namespace crash::reference
