#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <istream>
#include <memory>
#include <string>
#include <vector>

namespace crash::case_data {
struct WallVertex {
    std::array<double,3> position_m{};
    std::uint32_t vertex_index = 0;
    std::uint64_t source_node_id = 0, assembled_source_node_id = 0;
};
struct WallTriangle {
    std::array<std::uint32_t,3> vertex_indices{};
    std::uint64_t triangle_id = 0, source_quad_id = 0, assembled_source_quad_id = 0;
    std::array<std::uint64_t,3> source_node_ids{};
};
struct WallSourceQuad {
    std::uint64_t source_quad_id = 0, assembled_source_quad_id = 0;
    std::uint64_t source_part_id = 0, assembled_source_part_id = 0;
    std::array<std::uint64_t,4> source_node_ids{};
};
struct WallStitch {
    std::uint64_t source_quad_id = 0;
    std::array<std::uint64_t,2> source_edge{};
    std::vector<std::uint64_t> inserted_source_node_ids;
};
struct WallReactionGroups {
    std::vector<std::uint64_t> whole_wall_triangle_ids;
    std::vector<std::uint64_t> source_segment_set_1001_triangle_ids;
};
struct WallProvenance {
    std::string wall_file, combine_file, wall_sha256, combine_sha256;
    std::string model_archive_reference_sha256, generator_sha256, obj_sha256;
};
struct WallLimits {
    std::size_t max_vertices = 64, max_triangles = 100, max_source_quads = 46;
    std::size_t max_json_bytes = 1024*1024;
};
enum class WallStatus { Ok, IoError, ParseError, InvalidSchema, InvalidData, ResourceLimit, AlreadyLoaded };
struct WallReport {
    WallStatus status = WallStatus::InvalidData;
    const char* message = "Invalid wall";
    std::size_t parse_offset = 0;
};

// Application-owned immutable canonical fixed-wall geometry, in SI. Full-
// precision bundled RapidJSON preserves binary64 coordinates and uint64 IDs.
// The schema describes a fixed -X facing plane at X=.05 m; no analytic plane
// force generation, mechanics import, offsets or runtime motion is performed.
// Declared source hashes are retained and syntax-checked, not recomputed from
// source files. The case driver owns pinned-asset selection/authentication.
// Load publishes once after all checks; failure leaves an unloaded object empty.
// Loading an already published object is rejected and preserves all its views.
// Small schema-conforming test assets are allowed under explicit caps; the real
// Yaris case must additionally require the canonical 62/100/46 counts/pins.
class CanonicalWall {
  public:
    CanonicalWall();
    ~CanonicalWall();
    CanonicalWall(const CanonicalWall&) = delete;
    CanonicalWall& operator=(const CanonicalWall&) = delete;
    WallReport LoadFile(const std::string& path, WallLimits limits = {});
    WallReport Load(std::istream&, WallLimits limits = {});
    bool loaded() const noexcept;
    const std::vector<WallVertex>& vertices() const noexcept;
    const std::vector<WallTriangle>& triangles() const noexcept;
    const std::vector<WallSourceQuad>& source_quads() const noexcept;
    const std::vector<WallStitch>& stitching() const noexcept;
    const WallReactionGroups& reaction_groups() const noexcept;
    const WallProvenance& provenance() const noexcept;
    std::array<double,3> front_normal() const noexcept;
    std::array<std::array<double,3>,2> bounds_m() const noexcept;
    double area_m2() const noexcept;
  private:
    struct Data;
    std::unique_ptr<const Data> data_;
};
}  // namespace crash::case_data
