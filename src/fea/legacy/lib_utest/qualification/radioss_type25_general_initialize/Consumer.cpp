// SPDX-License-Identifier: AGPL-3.0-or-later
#include "lib_src/collision/RadiossType25Transaction.h"
#include "lib_src/assembly/ShellPhysicalBinding.h"
int main() {
  namespace n=tlfea::contact::radioss_type25;
  n::Transaction transaction;n::TransactionConfig config;n::MixedMovingMainSource source;
  n::initial_source::PreparedSource prepared;tl::fea::ShellPhysicalBinding physical;
  n::GeneralTransactionForecast forecast;forecast.peak_device_bytes=17;
  const auto report=n::Transaction::GeneralPreflight(config,source,prepared,physical,forecast);
  return report.status!=n::TransactionStatus::SourceMismatch||forecast.peak_device_bytes!=17||
      transaction.initialization_diagnostics().available||transaction.source_info().available;
}
