! SPDX-License-Identifier: AGPL-3.0-or-later
module precision_mod
 use iso_c_binding
 implicit none
 integer,parameter::WP=c_double
end module
module intbufdef_mod
 implicit none
 type::intbuf_struct_
  integer::s_remnode=0,s_icont_i=0
  integer,allocatable::remnode(:),icont_i(:)
 end type
end module
module my_alloc_mod
 use iso_c_binding
 implicit none
 private
 public::my_alloc
 interface my_alloc
  module procedure int_vector,int_matrix,real_vector
 end interface
contains
 subroutine int_vector(a,n,name)
  integer,allocatable,intent(inout)::a(:)
  integer,intent(in)::n
  character(*),intent(in)::name
  allocate(a(n))
 end subroutine
 subroutine int_matrix(a,n,m,name)
  integer,allocatable,intent(inout)::a(:,:)
  integer,intent(in)::n,m
  character(*),intent(in)::name
  allocate(a(n,m))
 end subroutine
 subroutine real_vector(a,n,name)
  real(c_double),allocatable,intent(inout)::a(:)
  integer,intent(in)::n
  character(*),intent(in)::name
  allocate(a(n))
 end subroutine
end module
module my_dealloc_mod
 use iso_c_binding
 implicit none
 private
 public::my_dealloc
 interface my_dealloc
  module procedure int_vector,int_matrix,real_vector
 end interface
contains
 subroutine int_vector(a)
  integer,allocatable,intent(inout)::a(:)
  if(allocated(a))deallocate(a)
 end subroutine
 subroutine int_matrix(a)
  integer,allocatable,intent(inout)::a(:,:)
  if(allocated(a))deallocate(a)
 end subroutine
 subroutine real_vector(a)
  real(c_double),allocatable,intent(inout)::a(:)
  if(allocated(a))deallocate(a)
 end subroutine
end module
module my_move_alloc_mod
 implicit none
contains
 subroutine my_move_alloc(a,b,name)
  integer,allocatable,intent(inout)::a(:),b(:)
  character(*),intent(in)::name
  call move_alloc(a,b)
 end subroutine
end module
subroutine upgrade_remnode(ipari,required,buffer,nty)
 use intbufdef_mod
 implicit none
 integer::ipari(*),required,nty
 type(intbuf_struct_)::buffer
 ! Wrapper preallocates the exact worst-case G*S table before source execution.
 error stop 'Native removal exceeded the qualified G*S bound'
end subroutine
