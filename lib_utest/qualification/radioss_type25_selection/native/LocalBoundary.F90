! SPDX-License-Identifier: AGPL-3.0-or-later
! Qualification-only foreign symbols needed to compile the complete native
! routines. Stage wrappers admit local rows only; no foreign data are invented.
module tri7box
  use iso_c_binding
  implicit none
  type :: real_vector
    real(c_double), pointer :: p(:) => null()
  end type
  type :: real_matrix
    real(c_double), pointer :: p(:,:) => null()
  end type
  type :: integer_vector
    integer, pointer :: p(:) => null()
  end type
  type :: integer_matrix
    integer, pointer :: p(:,:) => null()
  end type
  type(real_vector) :: gapfi(1),gap_lfi(1),stifi(1),time_sfi(1)
  type(real_matrix) :: xfi(1),vfi(1),pene_oldfi(1)
  type(integer_vector) :: icodt_fi(1),iskew_fi(1),icont_i_fi(1)
  type(integer_matrix) :: irtlm_fi(1),islide_fi(1)
end module
