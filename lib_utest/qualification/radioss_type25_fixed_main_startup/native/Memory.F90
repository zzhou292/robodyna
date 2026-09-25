! SPDX-License-Identifier: AGPL-3.0-or-later
! Reference-only allocation boundary for original integer and REAL4 scratch.
module startup_native_memory
  use iso_c_binding
  implicit none
  interface my_alloc
    module procedure i1,i2,i3,f3
  end interface
  interface my_dealloc
    module procedure di1,di2,di3,df3
  end interface
contains
  subroutine i1(a,n,name,stat)
    integer,allocatable,intent(inout)::a(:)
    integer,intent(in)::n
    character(*),intent(in)::name
    integer,optional,intent(out)::stat
    allocate(a(n)); if(present(stat))stat=0
  end subroutine
  subroutine i2(a,n,m,name,stat)
    integer,allocatable,intent(inout)::a(:,:)
    integer,intent(in)::n,m
    character(*),intent(in)::name
    integer,optional,intent(out)::stat
    allocate(a(n,m)); if(present(stat))stat=0
  end subroutine
  subroutine i3(a,n,m,k,name,stat)
    integer,allocatable,intent(inout)::a(:,:,:)
    integer,intent(in)::n,m,k
    character(*),intent(in)::name
    integer,optional,intent(out)::stat
    allocate(a(n,m,k)); if(present(stat))stat=0
  end subroutine
  subroutine f3(a,n,m,k,name,stat)
    real(c_float),allocatable,intent(inout)::a(:,:,:)
    integer,intent(in)::n,m,k
    character(*),intent(in)::name
    integer,optional,intent(out)::stat
    allocate(a(n,m,k)); if(present(stat))stat=0
  end subroutine
  subroutine di1(a)
    integer,allocatable,intent(inout)::a(:)
    if(allocated(a))deallocate(a)
  end subroutine
  subroutine di2(a)
    integer,allocatable,intent(inout)::a(:,:)
    if(allocated(a))deallocate(a)
  end subroutine
  subroutine di3(a)
    integer,allocatable,intent(inout)::a(:,:,:)
    if(allocated(a))deallocate(a)
  end subroutine
  subroutine df3(a)
    real(c_float),allocatable,intent(inout)::a(:,:,:)
    if(allocated(a))deallocate(a)
  end subroutine
end module
