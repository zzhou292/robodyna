#include "SourceShellCollection.h"
#include "SourceShellReferenceInput.h"
#include <exception>

namespace crash::qualification::source_contact {
FixtureReport SourceShellCollection::Initialize(const SourcePartContactFixture& source) {
    if (prepared_) return {FixtureStatus::InvalidArgument, "Source shell input is immutable after initialization"};
    if (!source.prepared()) return {FixtureStatus::InvalidFixture, "Source geometry is not authenticated"};
    SourceShellCollection staged;
    std::size_t q = 0, t = 0;
    try {
        for (std::size_t p = 0; p < ParentCount; ++p) {
            const auto& parent = source.parents()[p];
            if (parent.arity == 4 && q < Q4Count) {
                auto& entry = staged.qeph_[q];
                entry.reference = QephReferenceInput(source, p);
                for (unsigned n = 0; n < 4; ++n) entry.nodes[n] = parent.local_node_indices[n];
                entry.source_parent_id = parent.source_id;
                staged.qeph_parent_[q++] = p;
            } else if (parent.arity == 3 && t < T3Count) {
                auto& entry = staged.t3_[t];
                entry.reference = T3PortReferenceInput(source, p);
                for (unsigned n = 0; n < 3; ++n) entry.nodes[n] = parent.local_node_indices[n];
                entry.source_parent_id = parent.source_id;
                staged.t3_parent_[t++] = p;
            } else return {FixtureStatus::InvalidFixture, "Unexpected source parent family or capacity"};
        }
    } catch (const std::exception& e) {
        return {FixtureStatus::InvalidFixture, e.what()};
    }
    if (q != Q4Count || t != T3Count)
        return {FixtureStatus::InvalidFixture, "Incomplete original source shell collection"};
    staged.prepared_ = true;
    *this = staged;
    return {FixtureStatus::Ok, "Complete original source geometry mapped to explicit elastic QEPH/T3 inputs"};
}
}  // namespace crash::qualification::source_contact
