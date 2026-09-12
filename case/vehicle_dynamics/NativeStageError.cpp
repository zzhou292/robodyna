#include "NativeStageError.h"
#include <string>

namespace crash::cases::vehicle_dynamics {
namespace {

std::string FamilyName(tl::fea::solids::Family family) {
    using Family = tl::fea::solids::Family;
    switch (family) {
        case Family::Solid18: return "Solid18";
        case Family::Solid24: return "Solid24";
        case Family::Solid6z: return "Solid6z";
        case Family::Solid18Law44: return "Solid18Law44";
        case Family::Solid18Law90: return "Solid18Law90";
    }
    return "unknown(" + std::to_string(static_cast<int>(family)) + ")";
}

std::string Index(std::size_t index) {
    return index == SIZE_MAX ? "unavailable" : std::to_string(index);
}

template<class Report>
std::string Message(const Report& report, const char* stage, const std::string& participant) {
    std::string result = stage ? stage : "Physical stage";
    result += ": ";
    result += report.message ? report.message : "Native operation rejected";
    result += " [" + participant;
    result += " parent_index=" + Index(report.parent);
    result += " node_index=" + Index(report.node);
    result += " batch_status=" + std::to_string(static_cast<int>(report.status));
    result += " element_status=" + std::to_string(report.element_status);
    result += " nodal_status=" + std::to_string(static_cast<int>(report.nodal_status));
    result += "]";
    return result;
}

} // namespace

NativeStageError::NativeStageError(const tl::fea::solids::BatchReport& report, const char* stage)
    : std::runtime_error(Message(report, stage, "participant=solids family=" + FamilyName(report.family))),
      failure_(SolidFailure{report.status, report.family, report.parent, report.node,
                            report.element_status, report.nodal_status}) {}

NativeStageError::NativeStageError(const tl::fea::beam18::BatchReport& report, const char* stage)
    : std::runtime_error(Message(report, stage, "participant=beam18")),
      failure_(StructuralBeamFailure{report.status, report.parent, report.node,
                                     report.element_status, report.nodal_status}) {}

} // namespace crash::cases::vehicle_dynamics
