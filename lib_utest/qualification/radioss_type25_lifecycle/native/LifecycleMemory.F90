! SPDX-License-Identifier: AGPL-3.0-or-later
! Qualification-only allocation adapters for the complete PREP_SLID_2 donor.
! They allocate its original temporary integer arrays, without changing its
! count-before-append, overflow return, or arithmetic. No production use.
module my_alloc_mod
  implicit none
  interface my_alloc
    module procedure allocate_integer_vector
  end interface
contains
  subroutine allocate_integer_vector(value, count, name)
    integer, allocatable, intent(inout) :: value(:)
    integer, intent(in) :: count
    character(*), intent(in) :: name
    allocate(value(count))
  end subroutine
end module
module my_dealloc_mod
  implicit none
  interface my_dealloc
    module procedure deallocate_integer_vector
  end interface
contains
  subroutine deallocate_integer_vector(value)
    integer, allocatable, intent(inout) :: value(:)
    if (allocated(value)) deallocate(value)
  end subroutine
end module
