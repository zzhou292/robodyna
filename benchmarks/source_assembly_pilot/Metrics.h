#pragma once
#include "output/AcceptedReplayValues.h"
#include <array>
#include <map>

namespace crash::benchmarks::assembly_pilot {
struct Metric {
    std::string units;
    std::size_t count=0,nonzero=0,worst_index=0;
    std::uint64_t worst_source_id=0;
    double maximum=0,reference_maximum=0;
    long double square=0,reference_square=0;
    void Add(double reference,double candidate,std::size_t index=0,std::uint64_t source=0);
    void Distance(double reference_magnitude,double distance,std::size_t index,std::uint64_t source);
};
using Metrics=std::map<std::string,Metric>;
double RotationDistance(const std::array<double,4>&,const std::array<double,4>&);
// Format-only fixtures may exercise these value utilities. Archive authority
// comes from Run::Open and hash-bound frames, never from this math entry point.
Metrics Difference(const output::Value&,const output::Value&);
output::Value MetricDocument(output::Document&,const Metrics&);
} // namespace crash::benchmarks::assembly_pilot
