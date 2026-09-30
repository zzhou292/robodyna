#include "Timing.h"
#include "Runtime.h"
#include <cerrno>
#include <cinttypes>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>

namespace robo_dyna::cuda_kernel_timing {
namespace {
class Json {
public:
    explicit Json(int fd) : fd_(fd) {}
    void Text(const char* value) noexcept {
        const auto length = std::strlen(value);
        if (!valid_ || length > ReportCap-size_) { valid_ = false; return; }
        std::size_t written = 0;
        while (written < length) {
            const auto count = write(fd_, value+written, length-written);
            if (count < 0 && errno == EINTR) continue;
            if (count <= 0) { valid_ = false; return; }
            written += static_cast<std::size_t>(count);
        }
        size_ += length;
    }
    void Format(const char* format, ...) noexcept {
        char buffer[1024];
        va_list args;
        va_start(args, format);
        const int count = std::vsnprintf(buffer, sizeof(buffer), format, args);
        va_end(args);
        if (count < 0 || static_cast<std::size_t>(count) >= sizeof(buffer)) { valid_ = false; return; }
        Text(buffer);
    }
    void String(const char* value) noexcept {
        Text("\"");
        for (const unsigned char* p = reinterpret_cast<const unsigned char*>(value); *p; ++p) {
            if (*p == '"' || *p == '\\') Format("\\%c", *p);
            else if (*p < 32 || *p >= 127) Format("\\u%04x", unsigned(*p));
            else { char character[2]{char(*p), 0}; Text(character); }
        }
        Text("\"");
    }
    bool valid() const noexcept { return valid_; }
private:
    int fd_;
    std::size_t size_ = 0;
    bool valid_ = true;
};
void Counts(Json& json, const Counters& counters) {
    json.Format("\"calls\":%" PRIu64 ",\"launch_failures\":%" PRIu64 ",\"timed_calls\":%" PRIu64
                ",\"total_device_ns\":%" PRIu64 ",\"maximum_device_ns\":%" PRIu64
                ",\"first_timed_device_ns\":%" PRIu64 ",\"unmeasured_calls\":%" PRIu64 ",",
                counters.calls, counters.launch_failures, counters.timed_calls, counters.total_ns,
                counters.maximum_ns, counters.first_timed_ns, counters.unmeasured);
    json.Format("\"first_grid\":[%u,%u,%u],\"first_block\":[%u,%u,%u],\"first_shared_bytes\":%zu,"
                "\"dimensions_changed_calls\":%" PRIu64, counters.grid[0], counters.grid[1], counters.grid[2],
                counters.block[0], counters.block[1], counters.block[2], counters.shared, counters.dimensions_changed);
}
}
void EmitReport() noexcept {
    const auto& state = Data(); // Shutdown holds state mutex: bounded coherent snapshot.
    const int fd = open(state.output, O_WRONLY|O_CREAT|O_EXCL|O_CLOEXEC|O_NOFOLLOW, 0600);
    if (fd < 0) { Warning("create-only report could not be completed"); return; }
    Json json(fd);
    json.Text("{\"schema\":\"robo_dyna.cuda_kernel_timing.v1\",\"diagnostic_serialization\":true,");
    json.Text("\"scope\":\"Same-stream CUDA event elapsed regions around intercepted outer launches. "
              "End events are synchronized; submission gaps and concurrent non-intercepted work may affect intervals. "
              "Not production throughput, an instruction timer, or graph/driver launch coverage.\",");
    json.Format("\"pid\":%ld,\"registry_capacity\":%zu,\"handle_capacity\":%zu,\"name_byte_cap\":%zu,"
                "\"fixed_state_bytes\":%zu,\"report_byte_cap\":%zu,\"active_at_shutdown\":%" PRIu64 ",",
                static_cast<long>(getpid()), KernelCap, HandleCap, NameCap, sizeof(State), ReportCap, state.active);
    json.Format("\"public_launch_calls\":%" PRIu64 ",\"cuda13_handle_launch_calls\":%" PRIu64
                ",\"nested_calls_skipped\":%" PRIu64 ",\"capture_calls_skipped\":%" PRIu64 ",",
                state.public_calls, state.handle_calls, state.nested, state.capture_skipped);
    json.Format("\"instrumentation_failures\":%" PRIu64 ",\"first_instrumentation_error\":%d,"
                "\"invalid_event_times\":%" PRIu64 ",\"registrations_dropped\":%" PRIu64
                ",\"handles_dropped\":%" PRIu64 ",\"counter_saturated\":%s,",
                state.instrumentation_failures, state.first_instrumentation_error, state.invalid_times,
                state.registrations_dropped, state.handles_dropped, state.saturated ? "true" : "false");
    json.Text("\"unknown\":{");
    Counts(json, state.unknown);
    json.Text("},\"kernels\":[");
    for (std::size_t i = 0; i < state.kernel_count; ++i) {
        const auto& kernel = state.kernels[i];
        json.Format("%s{\"registration\":%zu,\"name\":", i ? "," : "", i);
        json.String(kernel.name);
        json.Format(",\"name_truncated\":%s,\"active_registration\":%s,",
                    kernel.truncated ? "true" : "false", kernel.active ? "true" : "false");
        Counts(json, kernel.counters);
        json.Text("}");
    }
    json.Text("]}\n");
    const bool complete = json.valid();
    const int closed = close(fd);
    if (!complete || closed != 0) Warning("create-only report could not be completed");
}
} // namespace robo_dyna::cuda_kernel_timing
