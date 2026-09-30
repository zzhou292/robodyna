#include "CanonicalSpecs.h"
#include <sstream>

namespace crash::output::full_shell::source::detail {
const std::array<CanonicalSpec, CanonicalArrayCount>& CanonicalSpecs() {
    using S = arrays::Scalar;
    static const std::array<CanonicalSpec, CanonicalArrayCount> specs{{
        {"node_ids", S::UInt64, 1, "source_node_id"},
        {"node_positions", S::Float64, 3, "x_m,y_m,z_m"},
        {"node_codes", S::Int32, 2, "tc_raw,rc_raw"},
        {"node_blank_masks", S::UInt16, 1, "blank_field_mask"},
        {"node_source_lines", S::UInt32, 1, "source_line"},
        {"shells_records", S::UInt64, 6, "element_id,part_id,n1,n2,n3,n4"},
        {"shells_node_indices", S::UInt32, 4, ""},
        {"shells_source_lines", S::UInt32, 1, "source_line"},
        {"shells_blank_masks", S::UInt16, 1, "blank_field_mask"},
        {"solids_records", S::UInt64, 10, "element_id,part_id,n1,n2,n3,n4,n5,n6,n7,n8"},
        {"solids_node_indices", S::UInt32, 8, ""},
        {"solids_source_lines", S::UInt32, 1, "source_line"},
        {"solids_blank_masks", S::UInt16, 1, "blank_field_mask"},
        {"beams_records", S::UInt64, 10, "element_id,part_id,n1,n2,n3,rt1,rr1,rt2,rr2,local"},
        {"beams_node_indices", S::UInt32, 2, ""},
        {"beams_source_lines", S::UInt32, 1, "source_line"},
        {"beams_blank_masks", S::UInt16, 1, "blank_field_mask"}
    }};
    return specs;
}
std::vector<std::string> FieldNames(const char* text) {
    std::istringstream stream(text);
    std::vector<std::string> result;
    std::string field;
    while (std::getline(stream, field, ',')) result.push_back(field);
    return result;
}
} // namespace crash::output::full_shell::source::detail
