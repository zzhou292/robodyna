! SPDX-License-Identifier: AGPL-3.0-or-later
! Qualification-only declarations for complete native foreign branches.
! Wrappers admit local rows and NSNR=0; no foreign arrays are allocated.
module lifecycle_foreign
  use iso_c_binding
  implicit none
  type :: real_vector
    real(c_double), pointer :: p(:) => null()
  end type
  type :: real_matrix
    real(c_double), pointer :: p(:,:) => null()
  end type
  type :: integer_vector
    integer(c_int), pointer :: p(:) => null()
  end type
  type :: integer_matrix
    integer(c_int), pointer :: p(:,:) => null()
  end type
  type(real_vector) :: time_sfi(1), gapfi(1), gap_lfi(1), stifi(1)
  type(real_matrix) :: xfi(1), vfi(1)
  type(integer_vector) :: kremnor_fi(1), remnor_fi(1), itafi(1), icont_i_fi(1)
  type(integer_matrix) :: irtlm_fi(1), islide_fi(1)
end module
