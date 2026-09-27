! SPDX-License-Identifier: AGPL-3.0-or-later
subroutine ic1_native_slots(parameters,slots) bind(C,name='ic1_native_slots')
  use iso_c_binding,only:c_double
  use HEPH_NATIVE_CONSTANT_MOD
  implicit none
  real(c_double),intent(in)::parameters(4)
  real(c_double),intent(out)::slots(6)
  real(c_double)::mu(10),al(10),gs,nu,bulk,young,g,parmat(128),pm(110,1),reader_bulk
  integer::i,norder,ilaw,imat
  mu=zero;al=zero;mu(1)=parameters(1);al(1)=two;norder=1;nu=parameters(2)
  pm=zero;parmat=zero
#include "slot_gs.inc"
#include "slot_bulk.inc"
  reader_bulk=bulk
#include "slot_parmat.inc"
  ilaw=42;i=1;imat=1
#include "slot_generic.inc"
  ! Generic HM_READ_MAT explicitly preserves LAW42 PM100; no-Prony GammaInf1
  ! LAW42_UPD leaves PM32/100 unchanged before the common UPDMAT refresh.
  pm(100,1)=reader_bulk
#include "slot_pm107.inc"
  slots=[pm(20,1),pm(21,1),pm(22,1),pm(32,1),pm(100,1),pm(107,1)]
end subroutine
