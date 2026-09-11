#pragma once
#include "../Internal.h"
#include <gtest/gtest.h>

namespace crash::modelio::tied_shell::classification_test {
struct Fixture {
    source::CanonicalData canonical;
    Data declaration;
    AuxiliaryData auxiliary;
    vehicle::rigid_part::SourceData rigid;
    std::vector<SourceEvidence> wall_sources;
    std::string wall;
    Fixture();
    void Metadata();
    ClassificationSourceReceipt Check() const;
};
std::string Card(std::initializer_list<unsigned>, unsigned width = 10);
}
