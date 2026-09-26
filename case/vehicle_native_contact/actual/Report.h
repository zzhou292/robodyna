#pragma once
#include "../VehicleContactStartup.h"
#include "output/ArtifactIO.h"
#include <chrono>
#include <filesystem>
namespace crash::cases::vehicle_native_contact::test {
constexpr std::size_t GuardBytes = std::size_t{18} << 30;
constexpr std::size_t ExportBytes = 2u << 20;
using Clock = std::chrono::steady_clock;
double Seconds(Clock::time_point);
std::filesystem::path Destination();
output::Document Document(const char* stage);
void Sources(output::Document&, const detail::SourceInputs&);
void Plan(output::Document&, const Forecast&, bool complete);
void InitialValues(output::Document&, const InitialCensus&);
void Failure(output::Document&, const std::exception&);
} // namespace crash::cases::vehicle_native_contact::test
