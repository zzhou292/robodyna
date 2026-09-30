! Qualification context only. These are the fields actually consumed by the
! complete donors in this serial, explicit, no-output profile.
module h3d_mod
  implicit none
  type H3D_DATABASE
    integer :: N_SCAL_DMAS = 0
    integer :: N_VECT_CONT2 = 0
    integer :: N_VECT_PCONT2 = 0
  end type
end module

module cin_time_context
  use iso_c_binding
  implicit none
  real(c_double) :: TT = 0, DT1 = 1, DT2 = 1, DT12 = 1
end module

module element_mod
  implicit none
  integer, parameter :: nixs=10, nixq=10, nixc=6, nixtg=5
end module

subroutine MY_BARRIER()
  ! Selected NTHREAD=NSPMD=1: no concurrent native caller exists.
end subroutine
