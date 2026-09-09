#include "CouponProfile.h"

#include <charconv>
#include <iostream>
#include <stdexcept>

namespace {
using crash::output::Require;
unsigned Count(const std::string& text) {
    unsigned value = 0;
    const auto parsed = std::from_chars(text.data(), text.data() + text.size(), value);
    Require(parsed.ec == std::errc{} && parsed.ptr == text.data() + text.size(),
            "Profile counts must be unsigned decimal integers");
    return value;
}
struct Options {
    std::filesystem::path destination;
    crash::benchmarks::CouponProfileConfig config;
};
Options Parse(int argc, char** argv) {
    Require(argc >= 2,
            "usage: robo-dyna-coupon-profile NEW_RESULT.json [--warmup 1..100] "
            "[--steps-per-repeat 1..300] [--repetitions 1..5]; total steps <=1000");
    Options options;
    options.destination = argv[1];
    unsigned seen = 0;
    for (int i = 2; i < argc; ++i) {
        const std::string arg = argv[i];
        const unsigned bit = arg == "--warmup" ? 1 : arg == "--steps-per-repeat" ? 2 :
                             arg == "--repetitions" ? 4 : 0;
        Require(bit != 0, "Unknown coupon profile option");
        Require(!(seen & bit), "Duplicate coupon profile option");
        Require(i + 1 < argc, "Missing coupon profile option value");
        seen |= bit;
        const unsigned value = Count(argv[++i]);
        if (bit == 1) options.config.warmup_steps = value;
        if (bit == 2) options.config.steps_per_repeat = value;
        if (bit == 4) options.config.repetitions = value;
    }
    crash::benchmarks::ValidateCouponProfileConfig(options.config);
    const auto parent = options.destination.has_parent_path() ? options.destination.parent_path() : ".";
    Require(!options.destination.filename().empty() &&
                !std::filesystem::exists(options.destination) && std::filesystem::is_directory(parent),
            "Profile result must be a new file with an existing parent directory");
    Require(!std::filesystem::is_symlink(std::filesystem::symlink_status(options.destination)),
            "Profile result must not be a symlink");
    return options;
}
}  // namespace

int main(int argc, char** argv) {
    try {
        const auto options = Parse(argc, argv);
        auto result = crash::benchmarks::ProfileElasticCoupon(options.config);
        crash::output::WriteJson(options.destination, result);
        std::cout << "Saved bounded coupon profile to " << options.destination << '\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "robo-dyna coupon profile: " << error.what() << '\n';
        return 1;
    }
}
