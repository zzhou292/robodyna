#pragma once
#include "AcceptedReplay.h"
#include "SourcePartWallComparisonEvents.h"
#include <filesystem>

namespace crash::output::wall_comparison {
const Value& Field(const Value&,const char*);
double Real(const Value&,const char*);
std::uint64_t Unsigned(const Value&,const char*);
std::string Encode(const Value&);
Document ReadDocument(const std::filesystem::path&);
Document PhysicalConfiguration(const Value&);
void ValidatePilot(const Value& configuration,unsigned refinement);

struct Run {
    std::filesystem::path directory;
    AcceptedReplay replay;
    Document configuration,manifest;
    unsigned refinement=0;
    std::array<double,117> mass{};
    long double initial_momentum=0;
    Scales scales{};
    std::string physical_configuration,placement,placed_mesh,original_wall_manifest,manifest_sha256;
    Events events;
    double maximum_chord=0,maximum_strain=0,maximum_thickness_curvature=0;
    void Open(const std::filesystem::path&,unsigned refinement);
    Document CommonFrame(std::uint64_t base_epoch);
};
ContactSample Contact(const Value&);
Events ReadEvents(const Run&);
void CheckFrameMomentum(const Run&,const Value&);
} // namespace crash::output::wall_comparison
