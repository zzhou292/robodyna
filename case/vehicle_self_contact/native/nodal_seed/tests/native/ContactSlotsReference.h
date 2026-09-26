#pragma once
// Private independent native witness for admitted one-term alpha2/no-Prony
// LAW42. Inputs {mu_native,nu}, outputs {PM20,PM32,PM100,PM107}, all pressures
// in the caller's native working units. This witnesses contact slots after
// reader -> generic reader -> GammaInf1 updater -> UPDMAT refresh. It is not a
// general material reader, original-case source binder, or production target.
extern "C" void law42_contact_slots_native(const double* parameters,double* slots);
