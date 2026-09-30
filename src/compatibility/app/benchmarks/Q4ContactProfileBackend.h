#pragma once

#include "Q4ContactProfile.h"
#include "lib_src/collision/Q4RectangularIntegration.h"

namespace crash::profile::detail {
// Measurement adapters only. No copied integration, force, law or allocation
// policy: each operation dispatches directly to its owning TL implementation.
struct ScalarContactBackend {
    using Cell=contact::Q4IntegrationCell;
    using Result=contact::Q4IntegrationResult;
    using Scratch=contact::Q4IntegrationScratch;
    static constexpr auto kind=ContactProfileBackend::Scalar;
    static constexpr auto scratch_bytes=contact::MaxQ4IntegrationScratchBytes;
    TL_SURFACE_HD static const Result& Base(const Result& r) { return r; }
    TL_SURFACE_HD static unsigned U(const Result& r) { return r.deepest_leaf; }
    TL_SURFACE_HD static unsigned V(const Result& r) { return r.deepest_leaf; }
    static contact::Q4IntegrationCellKind Kind(const Cell& cell) { return cell.kind; }
    TL_SURFACE_HD static contact::Q4IntegrationReport Integrate(const contact::Q4NormalIntegrationInput& input,
        const contact::Q4IntegrationLimits& limits,Scratch scratch,Result* result) {
        return contact::IntegrateQ4NormalContact(input,limits,scratch,result);
    }
    TL_SURFACE_HD static contact::Q4IntegrationReport Finish(const contact::Q4NormalIntegrationInput& input,
        const contact::Q4IntegrationLimits& limits,Scratch scratch,const contact::Q4IntegralInterval gap[4],
        const double nominal[4],Result* result) {
        return contact::q4_integration::Finish(input,limits,scratch,gap,nominal,result->leaf_count,result->visited,result);
    }
};
struct RectangularContactBackend {
    using Cell=contact::Q4RectangularCell;
    using Result=contact::Q4RectangularResult;
    using Scratch=contact::Q4RectangularScratch;
    static constexpr auto kind=ContactProfileBackend::Rectangular;
    static constexpr auto scratch_bytes=contact::MaxQ4RectangularScratchBytes;
    TL_SURFACE_HD static const contact::Q4IntegrationResult& Base(const Result& r) { return r.integration; }
    TL_SURFACE_HD static unsigned U(const Result& r) { return r.deepest_u; }
    TL_SURFACE_HD static unsigned V(const Result& r) { return r.deepest_v; }
    static contact::Q4IntegrationCellKind Kind(const Cell& cell) { return cell.bounds.kind; }
    TL_SURFACE_HD static contact::Q4IntegrationReport Integrate(const contact::Q4NormalIntegrationInput& input,
        const contact::Q4IntegrationLimits& limits,Scratch scratch,Result* result) {
        return contact::IntegrateQ4NormalContactRectangular(input,limits,scratch,result);
    }
    TL_SURFACE_HD static contact::Q4IntegrationReport Finish(const contact::Q4NormalIntegrationInput& input,
        const contact::Q4IntegrationLimits& limits,Scratch scratch,const contact::Q4IntegralInterval gap[4],
        const double nominal[4],Result* result) {
        return contact::q4_rectangular::Finish(input,limits,scratch,gap,nominal,
            result->integration.leaf_count,result->integration.visited,result);
    }
};
} // namespace crash::profile::detail
