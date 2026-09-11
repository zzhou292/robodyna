#pragma once
#include <cstdint>
extern "C" void native_post_kinchk(std::int32_t nodes, const std::int32_t* ids,
    const std::int32_t* five_blocks, const std::int32_t* interface_decode,
    std::int32_t body_count, const std::int32_t* bodies, std::int32_t member_count,
    const std::int32_t* members, std::int32_t wall_node, std::int32_t rbe2_node,
    std::int32_t rbe3_node, std::int32_t cyclic_node, std::int32_t print_level,
    std::int32_t* output_blocks, std::int32_t* output_decode, std::int32_t* kinet,
    std::int32_t* statistics, std::int32_t* status);
// Bodies: five columns {main node, zero-based member offset, member count,
// hierarchy flag, sensor}. All node indices are one-based. Zero optional node
// disables that excluded-role control. statistics: warnings/errors/last message/
// native no-true-conflict summary/KINET completed. Invalid packets preserve outputs.
