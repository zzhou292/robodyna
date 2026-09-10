#pragma once
#include "qualification/source_contact/SourcePartContactFixture.h"
#include "lib_src/elements/ShellBatchPlasticity.h"
#include <array>

namespace crash::cases::source_part_plastic {
namespace source = crash::qualification::source_contact;
// Application observations of accepted TL section state. No constitutive
// update, device allocation, independent clock or publication lives here.
struct PlasticSummary {
    bool enabled = false;
    double maximum_plastic_strain = 0;
    double mean_plastic_strain = 0; // Native-area/reference-thickness weighted.
    double cumulative_plastic_work_J = 0; // Diagnostic; not extra internal energy.
    unsigned yielded_points = 0, yielded_parents = 0;
};
struct SourcePartPlasticState {
    std::array<tl::fea::ShellBatchSectionState,source::Q4Count> qeph{};
    std::array<tl::fea::ShellBatchSectionState,source::T3Count> t3{};
    std::array<double,source::Q4Count> qeph_reported_thickness{};
    std::array<double,source::T3Count> t3_reported_thickness{};
};
}
