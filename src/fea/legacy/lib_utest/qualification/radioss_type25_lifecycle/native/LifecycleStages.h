// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "LifecycleTables.h"
namespace type25_lifecycle_test::reference {
void ValidateProfileAndSpatial(const l::Input&);
std::size_t Begin(const l::Input&, OracleResult&);
void FilterRaw(const l::Input&, const Tables&, Rows&, OracleResult&);
void ClassifyRetained(const l::Input&, std::size_t retained, Rows&, OracleResult&);
void PrepareSliding(const l::Input&, const Tables&, Rows&, std::size_t retained, OracleResult&);
void AppendSliding(const l::Input&, const Tables&, Rows&, OracleResult&);
std::vector<int> Membership(const l::Input&, const Tables&, Rows&, const OracleResult&, int phase);
void Keep(const l::Input&, const Tables&, Rows&, OracleResult&);
} // namespace type25_lifecycle_test::reference
