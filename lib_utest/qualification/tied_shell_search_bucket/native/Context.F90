! SPDX-License-Identifier: MIT
! Test-only storage/allocator context; no search or projection arithmetic.
module stack_mod
  implicit none
  type stack_ply
    double precision :: pm(1)=0
  end type
end module
module my_alloc_mod
  implicit none
  interface my_alloc
    module procedure allocate_integer_vector
  end interface
contains
  subroutine allocate_integer_vector(values,n,label)
    integer,allocatable,intent(inout) :: values(:)
    integer,intent(in) :: n
    character(*),intent(in) :: label
    if(allocated(values).or.n<0) error stop 'Invalid native allocation context'
    allocate(values(n))
  end subroutine
end module
module my_dealloc_mod
  implicit none
  interface my_dealloc
    module procedure free_integer_vector
  end interface
contains
  subroutine free_integer_vector(values)
    integer,allocatable,intent(inout) :: values(:)
    if(.not.allocated(values)) error stop 'Invalid native deallocation context'
    deallocate(values)
  end subroutine
end module
module bucket_packet_context
  use iso_c_binding,only:c_int,c_double,c_int64_t
  implicit none
  ! Serial test oracle, installed only for the duration of one staged call.
  real(c_double),allocatable :: projection_thickness(:)
  integer(c_int),allocatable :: enumerated_pairs(:,:)
  integer(c_int64_t) :: pair_count=0
  logical :: pair_overflow=.false.
contains
  subroutine record_pairs(first,last,master,secondary)
    integer,intent(in) :: first,last,master(*),secondary(*)
    integer :: i
    do i=first,last
      pair_count=pair_count+1
      if(pair_count>size(enumerated_pairs,2)) then
        pair_overflow=.true.
      else
        enumerated_pairs(:,pair_count)=[master(i),secondary(i)]
      endif
    enddo
  end subroutine
end module
