! SPDX-License-Identifier: AGPL-3.0-or-later
! Compile-only declarations for source branches excluded by local1/foreign0/
! edge0. No foreign pointer/boundary array is read by the admitted native call.
module na_remote
 use iso_c_binding
 implicit none
 type::integer_vector
  integer,allocatable::p(:)
 end type
 type::integer_matrix
  integer,allocatable::p(:,:)
 end type
 type::real_vector
  real(c_double),allocatable::p(:)
 end type
 type(integer_vector)::kremnor_fi(1),remnor_fi(1)
 type(integer_matrix)::irtlm_fi(1)
 type(real_vector)::stifi(1)
end module
module na_sort
 implicit none
end module
module na_nodes
 implicit none
 type::nodal_arrays_
  integer::global_boundary_nb=0
  integer,allocatable::global_boundary(:)
 end type
end module
module na_memory
 implicit none
 private
 public::my_alloc,my_dealloc
contains
 subroutine my_alloc(a,n,label)
  integer,allocatable,intent(inout)::a(:)
  integer,intent(in)::n
  character(*),intent(in)::label
  allocate(a(n))
 end subroutine
 subroutine my_dealloc(a)
  integer,allocatable,intent(inout)::a(:)
  if(allocated(a))deallocate(a)
 end subroutine
end module
subroutine na_barrier()
end subroutine
