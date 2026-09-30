#include "PreparedCensusReplay.h"
#include "chrono_thirdparty/rapidjson/stringbuffer.h"
#include "chrono_thirdparty/rapidjson/writer.h"

#include <fstream>
#include <iostream>

int main(int argc, char** argv) {
    namespace replay = crash::cases::vehicle_startup::shell_execution::self_contact_test::prepared_replay;
    namespace output = crash::output;
    try {
        output::Require(argc == 4, "Usage: prepared_census_replay census.json EXPECTED_SHA256 absent-report.jsonl");
        const std::filesystem::path destination = argv[3];
        output::Require(!std::filesystem::exists(destination), "Prepared replay report already exists");
        std::ofstream stream(destination, std::ios::binary);
        output::Require(bool(stream), "Cannot create prepared replay report");
        std::size_t bytes = 0;
        const auto write = [&](const output::Document& d) {
            rapidjson::StringBuffer buffer;
            rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
            output::Require(d.Accept(writer), "Cannot encode prepared replay report");
            output::Require(buffer.GetSize() + 1 <= replay::ArchiveByteCap - bytes,
                "Prepared replay report exceeds 2 GiB; incomplete diagnostic retained");
            stream.write(buffer.GetString(), static_cast<std::streamsize>(buffer.GetSize())); stream.put('\n');
            output::Require(bool(stream), "Prepared replay report write failed"); bytes += buffer.GetSize() + 1;
        };
        output::Document opening; opening.SetObject();
        output::String(opening, "schema", "robo_dyna.prepared_census_replay_start.v1");
        output::String(opening, "manifest_sha256", argv[2]); output::Boolean(opening, "complete", false);
        write(opening);
        const auto result = replay::Replay(argv[1], argv[2], [&](const replay::PairResult& pair) {
            if (!replay::Certified(pair.policy.status)) write(replay::PairDocument(pair));
        });
        write(replay::SummaryDocument(result, argv[2]));
        stream.flush(); output::Require(bool(stream), "Prepared replay report flush failed");
        stream.close(); output::Require(!stream.fail(), "Prepared replay report close failed");
        std::cout << "Diagnostic replay complete: pairs=" << result.pairs << " noncertified=" << result.noncertified
                  << " result_digest=" << result.result_digest << "; no physical acceptance claim\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Prepared census replay incomplete: " << error.what() << '\n'; return 1;
    }
}
