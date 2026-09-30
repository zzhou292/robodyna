/*==============================================================
 *==============================================================
 * Project: RoboDyna
 * Author:  Json Zhou
 * Email:   zzhou292@wisc.edu
 * File:    FEStateBuffer.h
 * Brief:   Unified state buffer abstraction for multi-element problems.
 *          Provides a consistent view of coordinates, velocities, and forces
 *          in buffers managed by FEMultiElementProblem.
 *==============================================================
 *==============================================================*/

#pragma once

#include <vector>

// Unified view of nodal state across all element blocks.
//
// This struct holds device pointers to coordinate/velocity/force buffers that
// span multiple element blocks. Each block is assigned a contiguous slice
// of the global buffers via BlockRange.
//
// FEMultiElementProblem owns these allocations. Existing element blocks keep
// separate coordinate allocations; SyncPositionsToElements/FromElements copy
// between those allocations and the ranges below. UpdateCollisionNodeBuffer
// also copies positions into the collision layout. This struct is a pointer
// view, not an owning allocation or a guarantee of zero-copy element binding.
// Block concatenation does not identify shared physical nodes across blocks.
struct FEStateBuffer {
  // Device pointers to unified coordinate buffers.
  // Layout: [block0_coefs, block1_coefs, ...]
  // Size: total_coef each
  double* d_x12 = nullptr;
  double* d_y12 = nullptr;
  double* d_z12 = nullptr;

  // Device pointer to unified velocity buffer.
  // Layout: [vx0, vy0, vz0, vx1, vy1, vz1, ...]
  // Size: total_coef * 3
  double* d_velocity = nullptr;

  // Device pointer to unified external force buffer.
  // Layout: [fx0, fy0, fz0, fx1, fy1, fz1, ...]
  // Size: total_coef * 3
  double* d_f_ext = nullptr;

  // Device pointer to column-major node buffer for collision.
  // Layout: [x0, x1, ..., y0, y1, ..., z0, z1, ...]
  // Size: total_coef * 3
  // Note: This is a separate buffer from d_x12/d_y12/d_z12 because
  // collision systems expect column-major layout.
  double* d_nodes_collision = nullptr;

  // Total number of coefficients (DOF indices / 3) across all blocks.
  int total_coef = 0;

  // Range of coefficients owned by each element block.
  struct BlockRange {
    int coef_offset;  // Starting coefficient index in global buffer
    int coef_count;   // Number of coefficients in this block
  };
  std::vector<BlockRange> blocks;

  // Convenience accessors
  int GetTotalDofs() const { return total_coef * 3; }
  int GetNumBlocks() const { return static_cast<int>(blocks.size()); }

  // Get the DOF offset for a block (coef_offset * 3)
  int GetBlockDofOffset(int block_idx) const {
    return blocks[block_idx].coef_offset * 3;
  }

  // Get the DOF count for a block (coef_count * 3)
  int GetBlockDofCount(int block_idx) const {
    return blocks[block_idx].coef_count * 3;
  }
};
