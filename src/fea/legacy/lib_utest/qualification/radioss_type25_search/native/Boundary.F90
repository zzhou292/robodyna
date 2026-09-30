! SPDX-License-Identifier: AGPL-3.0-or-later
! No production library depends on this test-only native boundary.
module search_mpi_boundary
  use iso_c_binding
  implicit none
  interface spmd_allreduce
    module procedure forbidden_real,forbidden_integer
  end interface
contains
  subroutine forbidden_real(a,b,n,op,comm)
    real(c_double) :: a,b
    integer :: n,op,comm
    error stop 'Unselected MPI branch entered'
  end subroutine
  subroutine forbidden_integer(a,b,n,op,comm)
    integer :: a,b,n,op,comm
    error stop 'Unselected MPI branch entered'
  end subroutine
end module
subroutine rd_search_extrema(nodes,ns,nm,no,secondary,main,one_d,x,v,saved,stiffness,boxes) bind(c)
  use iso_c_binding
  use search_constants
  implicit none
  integer(c_int),intent(in) :: nodes,ns,nm,no,secondary(*),main(*),one_d(*)
  real(c_double),intent(in) :: x(3,*),v(3,*),saved(3,*)
  real(c_double),intent(inout) :: stiffness(*)
  real(c_double),intent(out) :: boxes(6,4)
  integer :: numnod,nthread,nspmd
  common /search_domain/ numnod,nthread,nspmd
  numnod=nodes
  nthread=1
  nspmd=1
  boxes(1:3,:)=-ep30
  boxes(4:6,:)=ep30
  call i25buce_crit(x,secondary,main,ns,nm,0,saved,1,stiffness,v, &
       boxes(:,1),boxes(:,2),boxes(:,3),boxes(:,4),no,one_d)
end subroutine
