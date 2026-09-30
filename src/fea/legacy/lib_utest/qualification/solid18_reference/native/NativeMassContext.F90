! SPDX-License-Identifier: AGPL-3.0-or-later
! Test-only dormant context for complete SMASS3; no mass equations.
! SMASS3's complete dormant JALE branch only names ALE%GRID%NWALE.
! Supply that context, set JALE=JEUL=0, and retain the original branch untouched.
module SOLID18_REF_ALE_MOD
  implicit none
  type GridContext
    integer :: NWALE=0
  end type
  type AleContext
    type(GridContext) :: GRID
  end type
  type(AleContext) :: ALE
end module

! The original selected population has no Q1NP replacement. Preserve the full
! native skip branch and its allocatable-index contract with an empty context.
module SOLID18_REF_Q1NP_RESTART_MOD
  implicit none
  integer :: numelq1np_g=0
  integer, allocatable :: kq1np_tab_inv(:)
end module

subroutine SOLID18_REF_MY_EXIT(code)
  implicit none
  integer, intent(in) :: code
  error stop 'Unexpected selected-SMASS3 native exit'
end subroutine
