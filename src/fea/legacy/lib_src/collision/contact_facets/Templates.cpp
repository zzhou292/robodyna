// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"

namespace tlfea::contact::contact_facets {
namespace {
unsigned TriangularIndex(unsigned n, unsigned i, unsigned j) noexcept {
  return j * (n + 1) - (j ? j * (j - 1) / 2 : 0) + i;
}
Triangle Face(unsigned a, unsigned b, unsigned c) noexcept {
  return {{static_cast<std::uint8_t>(a), static_cast<std::uint8_t>(b), static_cast<std::uint8_t>(c)}};
}
}
bool BuildTemplates(unsigned level, Templates& out) noexcept {
  const unsigned n = 1u << level;
  for (unsigned j = 0; j <= n; ++j) {
    for (unsigned i = 0; i <= n; ++i) {
      auto& vertex = out.q4_vertices[j * (n + 1) + i];
      vertex.i = i;
      vertex.j = j;
      if (EvaluateQ4Shape(1. - 2. * i / n, 1. - 2. * j / n, vertex.weights) != Status::kOk) return false;
    }
    for (unsigned i = 0; i + j <= n; ++i) {
      auto& vertex = out.t3_vertices[TriangularIndex(n, i, j)];
      vertex.i = i;
      vertex.j = j;
      vertex.weights[0] = static_cast<double>(n - i - j) / n;
      vertex.weights[1] = static_cast<double>(i) / n;
      vertex.weights[2] = static_cast<double>(j) / n;
    }
  }
  unsigned q4_face = 0, t3_face = 0;
  for (unsigned j = 0; j < n; ++j) {
    for (unsigned i = 0; i < n; ++i) {
      const unsigned a = j * (n + 1) + i, b = a + 1, d = a + n + 1, c = d + 1;
      out.q4_triangles[q4_face++] = Face(a, b, c);
      out.q4_triangles[q4_face++] = Face(a, c, d);
    }
    for (unsigned i = 0; i + j < n; ++i) {
      const unsigned a = TriangularIndex(n, i, j), b = TriangularIndex(n, i + 1, j);
      const unsigned c = TriangularIndex(n, i, j + 1);
      out.t3_triangles[t3_face++] = Face(a, b, c);
      if (i + j + 1 < n) out.t3_triangles[t3_face++] = Face(b, TriangularIndex(n, i + 1, j + 1), c);
    }
  }
  return q4_face == 2 * n * n && t3_face == n * n;
}
} // namespace tlfea::contact::contact_facets
