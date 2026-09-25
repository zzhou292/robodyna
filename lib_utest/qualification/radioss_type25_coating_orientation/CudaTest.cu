// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Assertions.h"
#include "../radioss_type25_friction/CudaFixture.h"
#include <stdexcept>
namespace coating_orientation_test {
namespace {
struct Record {
    s::Status status = s::Status::InvalidInput;
    s::NativeCoatingOrientationResult value{s::CoatingOrientation::Reversed, 91.25};
};
__global__ void Evaluate(const s::NativeCoatingOrientationInput* inputs, Record* outputs,
                         std::size_t count, bool reverse) {
    for (std::size_t thread = blockIdx.x * blockDim.x + threadIdx.x;
         thread < count; thread += blockDim.x * gridDim.x) {
        const auto row = reverse ? count - 1 - thread : thread;
        auto result = outputs[row];
        result.status = s::EvaluateNativeCoatingOrientation(inputs[row], &result.value);
        outputs[row] = result;
    }
}
void Check(cudaError_t status) {
    if (status != cudaSuccess) throw std::runtime_error(cudaGetErrorString(status));
}
class CoatingOrientationCuda : public type25_friction_test::PacketCuda<> {
  protected:
    std::vector<Record> Run(const std::vector<s::NativeCoatingOrientationInput>& packets,
                            unsigned width, bool reverse) {
        static_assert(sizeof(s::NativeCoatingOrientationInput) <= RowBytes);
        static_assert(sizeof(Record) <= RowBytes);
        if (packets.empty() || packets.size() > Capacity) throw std::invalid_argument("Coating test packet cap");
        std::vector<Record> result(packets.size());
        std::vector<s::NativeCoatingOrientationInput> unchanged(packets.size());
        type25_friction_test::Drain drain{stream};
        Check(cudaMemcpyAsync(input, packets.data(), packets.size()*sizeof(packets[0]), cudaMemcpyHostToDevice, stream));
        Check(cudaMemcpyAsync(output, result.data(), result.size()*sizeof(result[0]), cudaMemcpyHostToDevice, stream));
        Evaluate<<<3, width, 0, stream>>>(static_cast<const s::NativeCoatingOrientationInput*>(input),
            static_cast<Record*>(output), packets.size(), reverse);
        Check(cudaGetLastError());
        Check(cudaMemcpyAsync(result.data(), output, result.size()*sizeof(result[0]), cudaMemcpyDeviceToHost, stream));
        Check(cudaMemcpyAsync(unchanged.data(), input, unchanged.size()*sizeof(unchanged[0]), cudaMemcpyDeviceToHost, stream));
        Check(cudaStreamSynchronize(stream));
        for (std::size_t i = 0; i < packets.size(); ++i) SameInput(unchanged[i], packets[i]);
        return result;
    }
};
TEST_F(CoatingOrientationCuda, NativeBitsAndSignsAgreeAcrossLaunchOrders) {
    auto packets = Cases();
    for (unsigned width : {1u, 7u, 32u, 128u}) {
        for (bool reverse : {false, true}) {
            SCOPED_TRACE(width);
            SCOPED_TRACE(reverse);
            std::reverse(packets.begin(), packets.end());
            const auto result = Run(packets, width, reverse);
            for (std::size_t i = 0; i < packets.size(); ++i) {
                SCOPED_TRACE(i);
                ASSERT_EQ(result[i].status, s::Status::Ok);
                Same(result[i].value, Oracle(packets[i]));
                s::NativeCoatingOrientationResult host;
                ASSERT_EQ(s::EvaluateNativeCoatingOrientation(packets[i], &host), s::Status::Ok);
                Same(result[i].value, host);
            }
        }
    }
}
TEST_F(CoatingOrientationCuda, DeviceRejectionPreservesOutputAndValidRetry) {
    std::vector<s::NativeCoatingOrientationInput> packets(3, Packet());
    packets[0].node_count = 6;
    packets[1].positions[7].x = NAN;
    for (unsigned i = 0; i < packets[2].node_count; ++i)
        packets[2].positions[i].x = std::numeric_limits<double>::max();
    const auto result = Run(packets, 7, true);
    const s::Status expected[]{s::Status::UnsupportedProfile, s::Status::InvalidInput, s::Status::NonfiniteResult};
    for (unsigned i = 0; i < result.size(); ++i) {
        EXPECT_EQ(result[i].status, expected[i]);
        Same(result[i].value, Record{}.value);
    }
    const auto retry = Run({Packet()}, 32, false);
    ASSERT_EQ(retry[0].status, s::Status::Ok);
    Same(retry[0].value, Oracle(Packet()));
}
}
} // namespace coating_orientation_test
