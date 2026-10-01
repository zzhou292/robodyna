// Robodyna public IO API. Storage and filesystem behavior remain in the foundation backend.
#ifndef ROBODYNA_IO_RBPATHS_H
#define ROBODYNA_IO_RBPATHS_H

#include "chrono/core/ChDataPath.h"

namespace robodyna::io {

/// Configure the shared data directory. Configure paths before concurrent use.
inline void SetDataPath(const std::string& path) { ::chrono::SetChronoDataPath(path); }

/// Return the existing shared data path by reference.
inline const std::string& GetDataPath() { return ::chrono::GetChronoDataPath(); }

/// Concatenate the current data path and filename without inserting a separator.
inline std::string GetDataFile(const std::string& filename) { return ::chrono::GetChronoDataFile(filename); }

/// Assign the demo output path without creating a directory.
inline void SetOutputPath(const std::string& path) { ::chrono::SetChronoOutputPath(path); }

/// Return the shared demo output path, creating its directory as in the inherited API.
inline const std::string& GetOutputPath() { return ::chrono::GetChronoOutputPath(); }

/// Assign the test output path without creating a directory.
inline void SetTestOutputPath(const std::string& path) { ::chrono::SetChronoTestOutputPath(path); }

/// Return the shared test output path, creating its directory as in the inherited API.
inline const std::string& GetTestOutputPath() { return ::chrono::GetChronoTestOutputPath(); }

// Preserve both existing std::filesystem::path and std::string overloads.
using ::chrono::CreateOutputDirectory;

}  // namespace robodyna::io

#endif
