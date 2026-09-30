// Existing quadrature donors for an offline surface-mass audit. Chrono's
// unchanged order-16 rule failed the fixed gate; retain that diagnostic and
// qualify a coherent Boost 4/8/16 sequence. No new root solver or mechanics.
#include "chrono/core/ChQuadrature.h"
#include "output/ArtifactIO.h"
#include <boost/math/quadrature/gauss.hpp>
#include <boost/version.hpp>

#include <gtest/gtest.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <iomanip>
#include <limits>
#include <string>
#include <sstream>
#include <utility>
#include <vector>

namespace {
namespace io = crash::output;
constexpr double kMomentTolerance = 2e-12;  // Declared before any donor run.
struct Rule {
    int order;
    std::vector<double> nodes, weights;
};

Rule GetChronoRule(int order) {
    chrono::ChQuadratureTables tables(order, order);
    return {order, tables.Lroots.at(0), tables.Weight.at(0)};
}

template <unsigned N>
Rule GetBoostRule() {
    using Donor = boost::math::quadrature::gauss<double, N>;
    const auto& x = Donor::abscissa();
    const auto& w = Donor::weights();
    io::Require(x.size() == N / 2 && w.size() == x.size(), "unexpected even Boost half-rule shape");
    std::vector<std::pair<double, double>> pairs;
    for (std::size_t i = 0; i < x.size(); ++i) {
        io::Require(x[i] > 0, "expected positive half-rule abscissae");
        pairs.emplace_back(x[i], w[i]);
        pairs.emplace_back(-x[i], w[i]);
    }
    std::sort(pairs.begin(), pairs.end(), [](auto a, auto b) { return a.first > b.first; });
    Rule result{static_cast<int>(N), {}, {}};
    for (auto pair : pairs) { result.nodes.push_back(pair.first); result.weights.push_back(pair.second); }
    return result;
}

struct Errors { double root = 0, moment = 0; };

std::string Precise(double value) {
    std::ostringstream stream;
    stream << std::setprecision(17) << value;
    return stream.str();
}

Errors Measure(const Rule& r) {
    io::Require(r.order == 4 || r.order == 8 || r.order == 16, "unsupported quadrature order");
    io::Require(r.nodes.size() == static_cast<std::size_t>(r.order) &&
                    r.weights.size() == r.nodes.size(), "quadrature shape mismatch");
    Errors errors;
    for (int i = 0; i < r.order; ++i) {
        io::Require(std::isfinite(r.nodes[i]) && std::abs(r.nodes[i]) < 1 &&
                        std::isfinite(r.weights[i]) && r.weights[i] > 0,
                    "invalid quadrature node/weight");
        if (i) io::Require(r.nodes[i] < r.nodes[i - 1], "donor nodes must be distinct and descending");
        io::Require(std::abs(r.nodes[i] + r.nodes[r.order - 1 - i]) <= kMomentTolerance &&
                        std::abs(r.weights[i] - r.weights[r.order - 1 - i]) <= kMomentTolerance,
                    "quadrature symmetry mismatch");
        // Independent three-term VALUE recurrence, not Chrono's power-basis
        // evaluator and not a new root solver. Moments alone can obscure a
        // larger high-order orthogonal-polynomial residual.
        long double p0 = 1, p1 = r.nodes[i];
        for (int degree = 2; degree <= r.order; ++degree) {
            const long double p = ((2 * degree - 1) * static_cast<long double>(r.nodes[i]) * p1 -
                                   (degree - 1) * p0) / degree;
            p0 = p1;
            p1 = p;
        }
        errors.root = std::max(errors.root, static_cast<double>(std::abs(p1)));
    }
    // Independent analytic integral of x^degree on [-1,1], evaluated in long
    // double by powers and sums, without calling Chrono's polynomial evaluator.
    for (int degree = 0; degree < 2 * r.order; ++degree) {
        long double sum = 0;
        for (int i = 0; i < r.order; ++i) {
            long double power = 1;
            for (int p = 0; p < degree; ++p) power *= static_cast<long double>(r.nodes[i]);
            sum += static_cast<long double>(r.weights[i]) * power;
        }
        const long double exact = degree % 2 ? 0 : 2.L / (degree + 1);
        const double error = static_cast<double>(std::abs(sum - exact));
        io::Require(std::isfinite(error), "nonfinite polynomial-moment residual");
        errors.moment = std::max(errors.moment, error);
    }
    return errors;
}

Errors Check(const Rule& r) {
    const auto errors = Measure(r);
    io::Require(errors.root <= kMomentTolerance, "quadrature failed independent Legendre root residual");
    io::Require(errors.moment <= kMomentTolerance, "quadrature failed the predeclared polynomial-moment gate");
    return errors;
}

TEST(ShellSurfaceQuadrature, ActualBoostOrderFourPolynomialAndRootGates) { EXPECT_NO_THROW(Check(GetBoostRule<4>())); }
TEST(ShellSurfaceQuadrature, ActualBoostOrderEightPolynomialAndRootGates) { EXPECT_NO_THROW(Check(GetBoostRule<8>())); }
TEST(ShellSurfaceQuadrature, ActualBoostOrderSixteenPolynomialAndRootGates) { EXPECT_NO_THROW(Check(GetBoostRule<16>())); }
TEST(ShellSurfaceQuadrature, UnchangedChronoFourAndEightRemainQualified) {
    EXPECT_NO_THROW(Check(GetChronoRule(4)));
    EXPECT_NO_THROW(Check(GetChronoRule(8)));
}
TEST(ShellSurfaceQuadrature, UnchangedChronoSixteenRetainsMeasuredQualificationFailure) {
    const auto errors = Measure(GetChronoRule(16));
    RecordProperty("chrono16_max_legendre_root_residual", Precise(errors.root));
    RecordProperty("chrono16_max_monomial_moment_error", Precise(errors.moment));
    EXPECT_GT(errors.root, kMomentTolerance);
    EXPECT_THROW(Check(GetChronoRule(16)), std::exception);
}
TEST(ShellSurfaceQuadrature, InvalidRulesCannotBeQualified) {
    auto bad = GetBoostRule<4>();
    bad.weights[0] += 1e-5;
    EXPECT_THROW(Check(bad), std::exception);
    bad = GetBoostRule<4>();
    bad.nodes[0] = bad.nodes[1];
    EXPECT_THROW(Check(bad), std::exception);
    bad = GetBoostRule<4>();
    bad.weights[0] = std::numeric_limits<double>::infinity();
    EXPECT_THROW(Check(bad), std::exception);
}

void Export(const std::filesystem::path& output, const std::filesystem::path& source,
            const std::filesystem::path& boost_headers, const std::filesystem::path& boost_license) {
    io::Document doc;
    doc.SetObject();
    io::String(doc, "schema", "robo-dyna.shell-surface-quadrature.v1");
    io::String(doc, "donor", "boost::math::quadrature::gauss<double,N>");
    io::String(doc, "donor_version", BOOST_LIB_VERSION);
    io::Integer(doc, "donor_version_integer", BOOST_VERSION);
    io::String(doc, "donor_license", "Boost Software License 1.0");
    io::String(doc, "donor_header_root", boost_headers.string());
    io::String(doc, "donor_license_file", boost_license.string());
    io::String(doc, "scope", "Fixed offline integration rules; no shell formulation or mass equivalence");
    io::Boolean(doc, "polynomial_moments_qualified", true);
    io::Boolean(doc, "legendre_root_residuals_qualified", true);
    io::Number(doc, "moment_absolute_tolerance", kMomentTolerance);
    auto& alloc = doc.GetAllocator();
    io::Value rules(rapidjson::kArrayType), sources(rapidjson::kArrayType);
    for (const auto& r : {GetBoostRule<4>(), GetBoostRule<8>(), GetBoostRule<16>()}) {
        const auto error = Check(r);  // Cannot bypass by filtering GTests.
        io::Value row(rapidjson::kObjectType), nodes(rapidjson::kArrayType), weights(rapidjson::kArrayType);
        row.AddMember("order", r.order, alloc);
        row.AddMember("max_polynomial_moment_absolute_error", error.moment, alloc);
        row.AddMember("max_legendre_root_residual", error.root, alloc);
        for (double x : r.nodes) nodes.PushBack(x, alloc);
        for (double w : r.weights) weights.PushBack(w, alloc);
        row.AddMember("nodes", nodes, alloc);
        row.AddMember("weights", weights, alloc);
        rules.PushBack(row, alloc);
    }
    for (const char* relative : {"boost/math/quadrature/gauss.hpp", "boost/math/special_functions/legendre.hpp",
                                 "boost/math/tools/roots.hpp", "boost/math/policies/policy.hpp", "boost/version.hpp"}) {
        const auto bytes = io::ReadBounded(boost_headers / relative, 1024 * 1024);
        io::Value row(rapidjson::kObjectType);
        row.AddMember("path", io::Value(relative, alloc), alloc);
        const auto hash = io::Sha256(bytes);
        row.AddMember("sha256", io::Value(hash.c_str(), alloc), alloc);
        sources.PushBack(row, alloc);
    }
    io::Value license(rapidjson::kObjectType);
    const auto license_bytes = io::ReadBounded(boost_license, 4 * 1024 * 1024);
    io::Require(license_bytes.find("Boost Software License") != std::string::npos,
                "missing packaged Boost license evidence");
    const auto license_hash = io::Sha256(license_bytes);
    license.AddMember("path", "package-copyright", alloc);
    license.AddMember("sha256", io::Value(license_hash.c_str(), alloc), alloc);
    sources.PushBack(license, alloc);
    io::Value references(rapidjson::kArrayType), diagnostics(rapidjson::kObjectType);
    for (const char* relative : {"src/chrono/core/ChQuadrature.h", "src/chrono/core/ChQuadrature.cpp", "LICENSE"}) {
        const auto hash = io::Sha256(io::ReadBounded(source / relative, 1024 * 1024));
        io::Value row(rapidjson::kObjectType);
        row.AddMember("path", io::Value(relative, alloc), alloc);
        row.AddMember("sha256", io::Value(hash.c_str(), alloc), alloc);
        references.PushBack(row, alloc);
    }
    const auto rejected = Measure(GetChronoRule(16));
    diagnostics.AddMember("donor", "chrono::ChQuadratureTables", alloc);
    diagnostics.AddMember("order", 16, alloc);
    diagnostics.AddMember("qualified", rejected.root <= kMomentTolerance && rejected.moment <= kMomentTolerance, alloc);
    diagnostics.AddMember("max_legendre_root_residual", rejected.root, alloc);
    diagnostics.AddMember("max_polynomial_moment_absolute_error", rejected.moment, alloc);
    diagnostics.AddMember("source_files", references, alloc);
    doc.AddMember("rejected_reference_diagnostic", diagnostics, alloc);
    doc.AddMember("rules", rules, alloc);
    doc.AddMember("reported_source_files", sources, alloc);
    io::WriteJson(output, doc);  // Existing absent-destination/checked-I/O policy.
}
}  // namespace

int main(int argc, char** argv) {
    std::filesystem::path output, source;
    std::filesystem::path boost_headers = "/usr/include";
    std::filesystem::path boost_license = "/usr/share/doc/libboost1.74-dev/copyright";
    std::vector<char*> remaining{argv[0]};
    for (int i = 1; i < argc; ++i) {
        const std::string flag = argv[i];
        if (flag == "--export" || flag == "--chrono-source-root" || flag == "--boost-include-root" || flag == "--boost-license") {
            if (++i == argc) { std::cerr << "missing custom argument value\n"; return 2; }
            if (flag == "--boost-include-root") { boost_headers = argv[i]; continue; }
            if (flag == "--boost-license") { boost_license = argv[i]; continue; }
            auto& destination = flag == "--export" ? output : source;
            if (!destination.empty()) { std::cerr << "duplicate custom argument\n"; return 2; }
            destination = argv[i];
        } else remaining.push_back(argv[i]);
    }
    if (output.empty() != source.empty()) { std::cerr << "export requires source root\n"; return 2; }
    int count = static_cast<int>(remaining.size());
    remaining.push_back(nullptr);
    ::testing::InitGoogleTest(&count, remaining.data());
    const int status = RUN_ALL_TESTS();
    if (status || output.empty()) return status;
    try { Export(output, source, boost_headers, boost_license); }
    catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
    return 0;
}
