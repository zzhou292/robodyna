#pragma once
#include "lib_src/collision/Q4ContactIntegration.h"
#include <array>
#include <string>

namespace crash::profile {
namespace contact=tlfea::contact;
// Prescribed diagnostic inputs, never an imported physical state or restart.
// Reuse C3 host preparation and explicitly selected prescribed C2 operation.
// The scalar backend remains the default; no production C4 selection changes.
enum class ContactProfileBackend { Scalar,Rectangular };
struct ContactProfileInput {
    double position[18]{},velocity[18]{},inverse_mass[6]{};
    std::uint8_t fixed[6]{};
    contact::SurfaceQ4 parents[2];
    double area[2]{},wall_x=0,stiffness=0,maximum_penetration=0;
    std::uint64_t epoch=0,attempt=0;
    contact::Q4IntegrationLimits limits;
    TL_SURFACE_HD contact::Q4NormalIntegrationInput View(unsigned parent) const {
        return {{{position,6,3,1},{velocity,6,3,1},parents,2},
                {inverse_mass,fixed,6,epoch},parent,attempt,wall_x,area[parent],stiffness,maximum_penetration};
    }
};
struct ContactProfileSource {
    ContactProfileInput input;
    std::string frame_sha256,wall_sha256;
    double accepted_time=0;
};
struct ContactParentProfile {
    contact::Q4IntegrationReport report;
    contact::Q4IntegrationResult result;
    double host_ms=0,host_finish_ms=0;
    std::array<float,3> gpu_ms{},gpu_finish_ms{};
    std::uint32_t deepest_u=0,deepest_v=0;
    std::array<std::uint32_t,3> leaf_kinds{}; // inactive, active, mixed; selected host partition.
    bool scalar_comparison=false;
    contact::Q4IntegrationReport scalar_report;
    contact::Q4IntegrationResult scalar_result;
    double scalar_host_ms=0;
    // Nodal magnitudes, resultant, potential. Outward diagnostic differences
    // and sums of the two retained scalar/candidate certificate radii.
    std::array<double,6> scalar_difference_upper{},scalar_error_sum_upper{};
    std::array<std::uint32_t,3> scalar_leaf_kinds{};
};
struct ContactProfileResult {
    std::array<ContactParentProfile,2> parents;
    ContactProfileBackend backend=ContactProfileBackend::Scalar;
    std::size_t device_bytes=0,scratch_bytes=0;
};
ContactProfileSource ReadContactProfileSource(const std::string& wall,const std::string& frame,const std::string& expected_sha256);
ContactProfileResult MeasureContactProfile(const ContactProfileInput&,ContactProfileBackend=ContactProfileBackend::Scalar);
void WriteContactProfile(const std::string& new_path,const ContactProfileSource&,const ContactProfileResult&);
} // namespace crash::profile
