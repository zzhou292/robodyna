#pragma once

#include "lib_src/collision/self_contact_transaction/Storage.h"

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vector>

#ifndef ROBO_COVERAGE_FIXTURE_NAMESPACE
#define ROBO_COVERAGE_FIXTURE_NAMESPACE nonlinear_fixture
#endif
#ifndef ROBO_COVERAGE_EXPECTED_PAIRS
#define ROBO_COVERAGE_EXPECTED_PAIRS 826
#endif
#ifndef ROBO_COVERAGE_EXPECTED_ROSTER_DIGEST
#define ROBO_COVERAGE_EXPECTED_ROSTER_DIGEST 2928523903779679127ull
#endif
#ifndef ROBO_COVERAGE_EXPECTED_SOURCE_DIGEST
#define ROBO_COVERAGE_EXPECTED_SOURCE_DIGEST 15183149279991149367ull
#endif
#ifndef ROBO_COVERAGE_EXPECTED_SCHEMA_HASH
#define ROBO_COVERAGE_EXPECTED_SCHEMA_HASH 1586894878486140084ull
#endif
#ifndef ROBO_COVERAGE_EXPECTED_SOURCE_HASH
#define ROBO_COVERAGE_EXPECTED_SOURCE_HASH 15426552270166476538ull
#endif
#ifndef ROBO_COVERAGE_EXPECTED_PROFILE_HASH
#define ROBO_COVERAGE_EXPECTED_PROFILE_HASH 5356721435267868170ull
#endif
#ifndef ROBO_COVERAGE_EXPECTED_DT_HASH
#define ROBO_COVERAGE_EXPECTED_DT_HASH 16186267176476366842ull
#endif
#ifndef ROBO_COVERAGE_EXPECTED_PAYLOAD_HASH
#define ROBO_COVERAGE_EXPECTED_PAYLOAD_HASH 6300036658283995026ull
#endif
#ifndef ROBO_COVERAGE_EXPECTED_PAYLOAD_BYTES
#define ROBO_COVERAGE_EXPECTED_PAYLOAD_BYTES 12043040ull
#endif

namespace crash::cases::vehicle_startup::shell_execution::
    self_contact_test::ROBO_COVERAGE_FIXTURE_NAMESPACE {

namespace contact = tlfea::contact;
namespace sct = tlfea::contact::self_contact_transaction;

inline constexpr std::uint64_t Magic = 0x3158464c4e4353ull;
inline constexpr std::uint64_t Version = 2;
inline constexpr std::uint64_t Endian = 0x0102030405060708ull;
inline constexpr std::size_t ExpectedPairs =
    ROBO_COVERAGE_EXPECTED_PAIRS;
inline constexpr std::size_t MaximumBytes = 64u << 20;
inline constexpr std::size_t MaximumFeatures = 15;
inline constexpr std::size_t MaximumIntersections = 1;
inline constexpr std::size_t MaximumOwners = 64;
inline constexpr std::uint64_t ExpectedRosterDigest =
    ROBO_COVERAGE_EXPECTED_ROSTER_DIGEST;
inline constexpr std::uint64_t ExpectedNonlinearRosterDigest =
    ROBO_COVERAGE_EXPECTED_SOURCE_DIGEST;
inline constexpr std::uint64_t ExpectedSchemaHash =
    ROBO_COVERAGE_EXPECTED_SCHEMA_HASH;
inline constexpr std::uint64_t ExpectedSourceHash =
    ROBO_COVERAGE_EXPECTED_SOURCE_HASH;
inline constexpr std::uint64_t ExpectedProfileHash =
    ROBO_COVERAGE_EXPECTED_PROFILE_HASH;
inline constexpr std::uint64_t ExpectedDtHash =
    ROBO_COVERAGE_EXPECTED_DT_HASH;
inline constexpr std::uint64_t ExpectedPayloadHash =
    ROBO_COVERAGE_EXPECTED_PAYLOAD_HASH;
inline constexpr std::uint64_t ExpectedPayloadBytes =
    ROBO_COVERAGE_EXPECTED_PAYLOAD_BYTES;
inline constexpr std::uint64_t ExpectedPolicyResultDigest =
    940115079017839607ull;

enum class PhaseLabel : std::uint64_t {
    AcceptedOwner = 1,
    PreparedCandidate = 2,
};

struct PhaseIdentity {
    PhaseLabel accepted_label = PhaseLabel::AcceptedOwner;
    PhaseLabel prepared_label = PhaseLabel::PreparedCandidate;
    std::uint64_t accepted_epoch = 0;
    std::uint64_t prepared_base_epoch = 0;
    std::uint64_t accepted_time_bits = 0;
    std::uint64_t prepared_base_time_bits = 0;
    std::uint64_t prepared_time_bits = 0;
    std::uint64_t accepted_velocity_time_bits = 0;
    std::uint64_t prepared_velocity_time_bits = 0;
    std::uint64_t accepted_temporal_scheme = 0;
    std::uint64_t prepared_temporal_scheme = 0;
    std::uint64_t accepted_velocity_phase = 0;
    std::uint64_t prepared_velocity_phase = 0;
    std::uint64_t prepared_trajectory = 0;
};

struct Pair {
    std::uint32_t facet[2]{};
    sct::NonlinearSeparationStatus baseline_status =
        sct::NonlinearSeparationStatus::InvalidInput;
    std::size_t baseline_work = 0;
    unsigned baseline_depth = 0;
    contact::CurrentFixedTriangle accepted[2];
    contact::CurrentFixedTriangle prepared[2];
    sct::FacetQuadraticCoefficients quadratic[2];
    double half_thickness[2]{};
    std::uint16_t accepted_mask = 0;
    std::uint16_t prepared_mask = 0;
    std::vector<contact::FixedTriangleFeatureCandidate>
        accepted_features;
    std::vector<sct::AcceptedFeaturePolicyEvidence>
        accepted_policy;
    std::vector<contact::FixedTriangleFeatureCandidate>
        prepared_features;
    std::vector<contact::FixedTriangleIntersection>
        accepted_intersections;
    std::vector<contact::FixedTriangleIntersection>
        prepared_intersections;
    std::vector<sct::AcceptedEventCertificate> accepted_owners;
};

struct File {
    std::uint64_t schema_hash = 0;
    std::uint64_t source_hash = 0;
    std::uint64_t profile_hash = 0;
    std::uint64_t dt_hash = 0;
    std::uint64_t payload_hash = 0;
    std::uint64_t payload_bytes = 0;
    std::uint64_t roster_digest = 0;
    std::uint64_t nonlinear_roster_digest = 0;
    PhaseIdentity phase;
    std::vector<Pair> pairs;
};

inline void HashByte(
    std::uint8_t value, std::uint64_t* hash) noexcept {
    *hash ^= value;
    *hash *= 1099511628211ull;
}

inline void HashUnsigned(
    std::uint64_t value, std::uint64_t* hash) noexcept {
    for (unsigned byte = 0; byte < 8; ++byte)
        HashByte(
            static_cast<std::uint8_t>(value >> (8 * byte)), hash);
}

inline void HashBytes(
    const void* data, std::size_t size,
    std::uint64_t* hash) noexcept {
    const auto* bytes = static_cast<const std::uint8_t*>(data);
    for (std::size_t i = 0; i < size; ++i)
        HashByte(bytes[i], hash);
}

inline std::uint64_t SchemaHash() noexcept {
    std::uint64_t hash = 1469598103934665603ull;
    HashUnsigned(sizeof(void*), &hash);
    HashUnsigned(sizeof(std::size_t), &hash);
    HashUnsigned(sizeof(contact::FixedTriangleKey), &hash);
    HashUnsigned(sizeof(contact::FacetVertexKey), &hash);
    HashUnsigned(sizeof(contact::FacetEdgeKey), &hash);
    HashUnsigned(sizeof(contact::CurrentFixedTriangle), &hash);
    HashUnsigned(
        alignof(contact::CurrentFixedTriangle), &hash);
    HashUnsigned(
        sizeof(contact::FixedTriangleFeatureCandidate), &hash);
    HashUnsigned(
        alignof(contact::FixedTriangleFeatureCandidate), &hash);
    HashUnsigned(
        sizeof(contact::FixedTriangleIntersection), &hash);
    HashUnsigned(sizeof(sct::FacetQuadraticCoefficients), &hash);
    HashUnsigned(sizeof(sct::AcceptedEventCertificate), &hash);
    HashUnsigned(
        alignof(sct::AcceptedEventCertificate), &hash);
    HashUnsigned(
        sizeof(sct::AcceptedFeaturePolicyEvidence), &hash);
    HashUnsigned(
        alignof(sct::AcceptedFeaturePolicyEvidence), &hash);
    HashUnsigned(sizeof(PhaseIdentity), &hash);
    HashUnsigned(
        offsetof(
            contact::FixedTriangleFeatureCandidate,
            representation_error_m),
        &hash);
    HashUnsigned(
        offsetof(sct::AcceptedEventCertificate, discovery), &hash);
    HashUnsigned(
        offsetof(sct::AcceptedEventCertificate, target_facet),
        &hash);
    return hash;
}

inline void HashPath(
    const contact::FixedTriangleKey& key,
    std::uint64_t* hash) noexcept {
    HashUnsigned(key.source_instance_id, hash);
    HashUnsigned(key.parent_eid, hash);
    HashUnsigned(key.level, hash);
    HashUnsigned(key.local_facet, hash);
}

inline std::uint64_t RosterDigest(
    const std::vector<Pair>& pairs) noexcept {
    std::uint64_t hash = 1469598103934665603ull;
    for (const auto& pair : pairs) {
        HashPath(pair.prepared[0].key, &hash);
        HashPath(pair.prepared[1].key, &hash);
        HashUnsigned(
            static_cast<unsigned>(pair.baseline_status), &hash);
        HashUnsigned(pair.baseline_work, &hash);
        HashUnsigned(pair.baseline_depth, &hash);
    }
    return hash;
}

inline std::uint64_t SourceHash(
    const std::vector<Pair>& pairs) noexcept {
    std::uint64_t hash = 1469598103934665603ull;
    for (const auto& pair : pairs) {
        HashUnsigned(pair.facet[0], &hash);
        HashUnsigned(pair.facet[1], &hash);
        HashBytes(pair.accepted, sizeof(pair.accepted), &hash);
        HashBytes(pair.prepared, sizeof(pair.prepared), &hash);
        HashBytes(pair.quadratic, sizeof(pair.quadratic), &hash);
        HashBytes(
            pair.half_thickness,
            sizeof(pair.half_thickness), &hash);
    }
    return hash;
}

class Writer {
  public:
    void U16(std::uint16_t value) {
        for (unsigned byte = 0; byte < 2; ++byte)
            bytes_.push_back(
                static_cast<std::uint8_t>(
                    value >> (8 * byte)));
    }
    void U32(std::uint32_t value) {
        for (unsigned byte = 0; byte < 4; ++byte)
            bytes_.push_back(
                static_cast<std::uint8_t>(
                    value >> (8 * byte)));
    }
    void U64(std::uint64_t value) {
        for (unsigned byte = 0; byte < 8; ++byte)
            bytes_.push_back(
                static_cast<std::uint8_t>(
                    value >> (8 * byte)));
    }
    template <class T>
    void Object(const T& value) {
        static_assert(std::is_trivially_copyable_v<T>);
        const auto* begin =
            reinterpret_cast<const std::uint8_t*>(&value);
        bytes_.insert(bytes_.end(), begin, begin + sizeof(T));
    }
    template <class T>
    void Objects(const std::vector<T>& values) {
        static_assert(std::is_trivially_copyable_v<T>);
        if (values.empty()) return;
        const auto* begin =
            reinterpret_cast<const std::uint8_t*>(
                values.data());
        bytes_.insert(
            bytes_.end(), begin,
            begin + values.size() * sizeof(T));
    }
    const std::vector<std::uint8_t>& bytes() const noexcept {
        return bytes_;
    }

  private:
    std::vector<std::uint8_t> bytes_;
};

class Reader {
  public:
    Reader(const std::uint8_t* data, std::size_t size)
        : data_(data), size_(size) {}
    std::uint16_t U16() {
        std::uint16_t result = 0;
        for (unsigned byte = 0; byte < 2; ++byte)
            result |= static_cast<std::uint16_t>(
                Byte()) << (8 * byte);
        return result;
    }
    std::uint32_t U32() {
        std::uint32_t result = 0;
        for (unsigned byte = 0; byte < 4; ++byte)
            result |= static_cast<std::uint32_t>(
                Byte()) << (8 * byte);
        return result;
    }
    std::uint64_t U64() {
        std::uint64_t result = 0;
        for (unsigned byte = 0; byte < 8; ++byte)
            result |= static_cast<std::uint64_t>(
                Byte()) << (8 * byte);
        return result;
    }
    template <class T>
    T Object() {
        static_assert(std::is_trivially_copyable_v<T>);
        Require(sizeof(T));
        T result;
        std::memcpy(&result, data_ + offset_, sizeof(T));
        offset_ += sizeof(T);
        return result;
    }
    template <class T>
    std::vector<T> Objects(std::size_t count) {
        static_assert(std::is_trivially_copyable_v<T>);
        if (count > MaximumBytes / sizeof(T))
            throw std::runtime_error(
                "Nonlinear fixture range overflows");
        const std::size_t bytes = count * sizeof(T);
        Require(bytes);
        std::vector<T> result(count);
        if (bytes)
            std::memcpy(
                result.data(), data_ + offset_, bytes);
        offset_ += bytes;
        return result;
    }
    std::size_t remaining() const noexcept {
        return size_ - offset_;
    }

  private:
    std::uint8_t Byte() {
        Require(1);
        return data_[offset_++];
    }
    void Require(std::size_t count) {
        if (count > size_ - offset_)
            throw std::runtime_error(
                "Nonlinear fixture is truncated");
    }
    const std::uint8_t* data_ = nullptr;
    std::size_t size_ = 0;
    std::size_t offset_ = 0;
};

inline void AppendPair(Writer* writer, const Pair& pair) {
    if (!writer ||
        pair.accepted_features.size() > MaximumFeatures ||
        pair.accepted_policy.size() !=
            pair.accepted_features.size() ||
        pair.prepared_features.size() > MaximumFeatures ||
        pair.accepted_intersections.size() >
            MaximumIntersections ||
        pair.prepared_intersections.size() >
            MaximumIntersections ||
        pair.accepted_owners.size() > MaximumOwners)
        throw std::runtime_error(
            "Nonlinear fixture pair exceeds schema bounds");
    writer->U32(pair.facet[0]);
    writer->U32(pair.facet[1]);
    writer->U32(
        static_cast<unsigned>(pair.baseline_status));
    writer->U64(pair.baseline_work);
    writer->U32(pair.baseline_depth);
    writer->U16(pair.accepted_mask);
    writer->U16(pair.prepared_mask);
    writer->U32(pair.accepted_features.size());
    writer->U32(pair.prepared_features.size());
    writer->U32(pair.accepted_intersections.size());
    writer->U32(pair.prepared_intersections.size());
    writer->U32(pair.accepted_owners.size());
    for (const auto& value : pair.accepted)
        writer->Object(value);
    for (const auto& value : pair.prepared)
        writer->Object(value);
    for (const auto& value : pair.quadratic)
        writer->Object(value);
    writer->Object(pair.half_thickness[0]);
    writer->Object(pair.half_thickness[1]);
    writer->Objects(pair.accepted_features);
    writer->Objects(pair.accepted_policy);
    writer->Objects(pair.prepared_features);
    writer->Objects(pair.accepted_intersections);
    writer->Objects(pair.prepared_intersections);
    writer->Objects(pair.accepted_owners);
}

inline Pair ReadPair(Reader* reader) {
    if (!reader)
        throw std::runtime_error(
            "Nonlinear fixture reader is absent");
    Pair pair;
    pair.facet[0] = reader->U32();
    pair.facet[1] = reader->U32();
    pair.baseline_status =
        static_cast<sct::NonlinearSeparationStatus>(
            reader->U32());
    pair.baseline_work = reader->U64();
    pair.baseline_depth = reader->U32();
    pair.accepted_mask = reader->U16();
    pair.prepared_mask = reader->U16();
    const auto accepted_features = reader->U32();
    const auto prepared_features = reader->U32();
    const auto accepted_intersections = reader->U32();
    const auto prepared_intersections = reader->U32();
    const auto accepted_owners = reader->U32();
    if (accepted_features > MaximumFeatures ||
        prepared_features > MaximumFeatures ||
        accepted_intersections > MaximumIntersections ||
        prepared_intersections > MaximumIntersections ||
        accepted_owners > MaximumOwners)
        throw std::runtime_error(
            "Nonlinear fixture count exceeds schema bounds");
    for (auto& value : pair.accepted)
        value = reader->Object<
            contact::CurrentFixedTriangle>();
    for (auto& value : pair.prepared)
        value = reader->Object<
            contact::CurrentFixedTriangle>();
    for (auto& value : pair.quadratic)
        value = reader->Object<
            sct::FacetQuadraticCoefficients>();
    pair.half_thickness[0] = reader->Object<double>();
    pair.half_thickness[1] = reader->Object<double>();
    pair.accepted_features = reader->Objects<
        contact::FixedTriangleFeatureCandidate>(
            accepted_features);
    pair.accepted_policy = reader->Objects<
        sct::AcceptedFeaturePolicyEvidence>(
            accepted_features);
    pair.prepared_features = reader->Objects<
        contact::FixedTriangleFeatureCandidate>(
            prepared_features);
    pair.accepted_intersections = reader->Objects<
        contact::FixedTriangleIntersection>(
            accepted_intersections);
    pair.prepared_intersections = reader->Objects<
        contact::FixedTriangleIntersection>(
            prepared_intersections);
    pair.accepted_owners = reader->Objects<
        sct::AcceptedEventCertificate>(accepted_owners);
    return pair;
}

inline void Write(
    const std::string& path, const std::vector<Pair>& pairs,
    std::uint64_t profile_hash, std::uint64_t dt_hash,
    std::uint64_t nonlinear_roster_digest,
    const PhaseIdentity& phase, bool enforce_pins = true) {
    const auto roster_digest = RosterDigest(pairs);
    if (enforce_pins &&
        ((ExpectedPairs && pairs.size() != ExpectedPairs) ||
        (ExpectedRosterDigest &&
         roster_digest != ExpectedRosterDigest) ||
        (ExpectedNonlinearRosterDigest &&
         nonlinear_roster_digest !=
             ExpectedNonlinearRosterDigest)))
        throw std::runtime_error(
            "Nonlinear fixture source roster is not pinned");
    Writer payload;
    for (const auto& pair : pairs)
        AppendPair(&payload, pair);
    if (payload.bytes().size() > MaximumBytes)
        throw std::runtime_error(
            "Nonlinear fixture payload exceeds hard cap");
    std::uint64_t payload_hash = 1469598103934665603ull;
    HashBytes(
        payload.bytes().data(), payload.bytes().size(),
        &payload_hash);
    Writer header;
    header.U64(Magic);
    header.U64(Version);
    header.U64(Endian);
    header.U64(SchemaHash());
    header.U64(SourceHash(pairs));
    header.U64(profile_hash);
    header.U64(dt_hash);
    header.U64(roster_digest);
    header.U64(nonlinear_roster_digest);
    header.U64(pairs.size());
    header.U64(payload.bytes().size());
    header.U64(payload_hash);
    header.U64(static_cast<std::uint64_t>(phase.accepted_label));
    header.U64(static_cast<std::uint64_t>(phase.prepared_label));
    header.U64(phase.accepted_epoch);
    header.U64(phase.prepared_base_epoch);
    header.U64(phase.accepted_time_bits);
    header.U64(phase.prepared_base_time_bits);
    header.U64(phase.prepared_time_bits);
    header.U64(phase.accepted_velocity_time_bits);
    header.U64(phase.prepared_velocity_time_bits);
    header.U64(phase.accepted_temporal_scheme);
    header.U64(phase.prepared_temporal_scheme);
    header.U64(phase.accepted_velocity_phase);
    header.U64(phase.prepared_velocity_phase);
    header.U64(phase.prepared_trajectory);
    std::ofstream output(
        path, std::ios::binary | std::ios::trunc);
    if (!output)
        throw std::runtime_error(
            "Cannot open nonlinear fixture output");
    output.write(
        reinterpret_cast<const char*>(
            header.bytes().data()),
        static_cast<std::streamsize>(
            header.bytes().size()));
    output.write(
        reinterpret_cast<const char*>(
            payload.bytes().data()),
        static_cast<std::streamsize>(
            payload.bytes().size()));
    if (!output)
        throw std::runtime_error(
            "Cannot write nonlinear fixture output");
}

inline File Read(
    const std::string& path, bool enforce_pins = true) {
    std::ifstream input(path, std::ios::binary | std::ios::ate);
    if (!input)
        throw std::runtime_error(
            "Cannot open nonlinear fixture");
    const auto end = input.tellg();
    if (end < 0 ||
        static_cast<std::uint64_t>(end) >
            MaximumBytes + 208)
        throw std::runtime_error(
            "Nonlinear fixture size is invalid");
    std::vector<std::uint8_t> bytes(
        static_cast<std::size_t>(end));
    input.seekg(0);
    input.read(
        reinterpret_cast<char*>(bytes.data()),
        static_cast<std::streamsize>(bytes.size()));
    if (!input)
        throw std::runtime_error(
            "Cannot read nonlinear fixture");
    Reader reader(bytes.data(), bytes.size());
    if (reader.U64() != Magic ||
        reader.U64() != Version ||
        reader.U64() != Endian)
        throw std::runtime_error(
            "Nonlinear fixture header is invalid");
    File result;
    result.schema_hash = reader.U64();
    result.source_hash = reader.U64();
    result.profile_hash = reader.U64();
    result.dt_hash = reader.U64();
    result.roster_digest = reader.U64();
    if (enforce_pins && ExpectedRosterDigest &&
        result.roster_digest != ExpectedRosterDigest)
        throw std::runtime_error(
            "Nonlinear fixture roster digest changed");
    result.nonlinear_roster_digest = reader.U64();
    const auto pair_count = reader.U64();
    result.payload_bytes = reader.U64();
    result.payload_hash = reader.U64();
    result.phase.accepted_label =
        static_cast<PhaseLabel>(reader.U64());
    result.phase.prepared_label =
        static_cast<PhaseLabel>(reader.U64());
    result.phase.accepted_epoch = reader.U64();
    result.phase.prepared_base_epoch = reader.U64();
    result.phase.accepted_time_bits = reader.U64();
    result.phase.prepared_base_time_bits = reader.U64();
    result.phase.prepared_time_bits = reader.U64();
    result.phase.accepted_velocity_time_bits = reader.U64();
    result.phase.prepared_velocity_time_bits = reader.U64();
    result.phase.accepted_temporal_scheme = reader.U64();
    result.phase.prepared_temporal_scheme = reader.U64();
    result.phase.accepted_velocity_phase = reader.U64();
    result.phase.prepared_velocity_phase = reader.U64();
    result.phase.prepared_trajectory = reader.U64();
    if (result.schema_hash != SchemaHash() ||
        (enforce_pins && ExpectedSchemaHash &&
         result.schema_hash != ExpectedSchemaHash) ||
        (enforce_pins && ExpectedSourceHash &&
         result.source_hash != ExpectedSourceHash) ||
        (enforce_pins && ExpectedProfileHash &&
         result.profile_hash != ExpectedProfileHash) ||
        (enforce_pins && ExpectedDtHash &&
         result.dt_hash != ExpectedDtHash) ||
        (enforce_pins && ExpectedPayloadHash &&
         result.payload_hash != ExpectedPayloadHash) ||
        (enforce_pins && ExpectedPayloadBytes &&
         result.payload_bytes != ExpectedPayloadBytes) ||
        result.phase.accepted_label !=
            PhaseLabel::AcceptedOwner ||
        result.phase.prepared_label !=
            PhaseLabel::PreparedCandidate ||
        result.phase.accepted_epoch !=
            result.phase.prepared_base_epoch ||
        result.phase.accepted_time_bits !=
            result.phase.prepared_base_time_bits ||
        (enforce_pins && ExpectedPairs &&
         pair_count != ExpectedPairs) ||
        (enforce_pins && ExpectedNonlinearRosterDigest &&
         result.nonlinear_roster_digest !=
             ExpectedNonlinearRosterDigest) ||
        result.payload_bytes != reader.remaining())
        throw std::runtime_error(
            "Nonlinear fixture schema or count changed");
    const auto* payload =
        bytes.data() + (bytes.size() - reader.remaining());
    std::uint64_t payload_hash = 1469598103934665603ull;
    HashBytes(payload, reader.remaining(), &payload_hash);
    if (payload_hash != result.payload_hash)
        throw std::runtime_error(
            "Nonlinear fixture payload hash changed");
    result.pairs.reserve(pair_count);
    for (std::size_t pair = 0; pair < pair_count; ++pair)
        result.pairs.push_back(ReadPair(&reader));
    if (reader.remaining() ||
        RosterDigest(result.pairs) != result.roster_digest ||
        SourceHash(result.pairs) != result.source_hash)
        throw std::runtime_error(
            "Nonlinear fixture payload verification failed");
    return result;
}

}  // namespace crash::cases::vehicle_startup::shell_execution::
   // self_contact_test::nonlinear_fixture
