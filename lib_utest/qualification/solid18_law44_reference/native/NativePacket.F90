! SPDX-License-Identifier: AGPL-3.0-or-later
subroutine rear18_reference_native(x,rho0,local,values,perm,mass,status) bind(C)
  use iso_c_binding, only: c_double,c_int,c_int64_t
  implicit none
  real(c_double),intent(in) :: x(3,8),rho0
  integer(c_int),intent(in) :: local(8)
  real(c_double),intent(out) :: values(148),mass(8)
  integer(c_int),intent(out) :: perm(8),status
  real(c_double) :: unique_x(3,8)
  logical :: seen(8)
  integer :: n,j,axis,native_nodes(8)
  interface
    subroutine solid18_reference_native(x,rho0,values,perm,status) bind(C)
      import c_double,c_int
      real(c_double),intent(in) :: x(3,8),rho0
      real(c_double),intent(out) :: values(148)
      integer(c_int),intent(out) :: perm(8),status
    end subroutine
    subroutine rear18_scatter(x,rho,volume,nodes,mass)
      import c_double
      real(c_double),intent(in) :: x(3,8),rho,volume
      integer,intent(in) :: nodes(8)
      real(c_double),intent(out) :: mass(8)
    end subroutine
  end interface
  status=1
  if(any(local<0).or.any(local>7)) return
  unique_x=0d0
  seen=.false.
  do n=1,8
    j=local(n)+1
    if(seen(j)) then
      do axis=1,3
        if(transfer(unique_x(axis,j),0_c_int64_t)/=transfer(x(axis,n),0_c_int64_t)) return
      enddo
    else
      unique_x(:,j)=x(:,n)
      seen(j)=.true.
    endif
  enddo
  ! Complete already-qualified geometry packet consumes all eight source slots.
  call solid18_reference_native(x,rho0,values,perm,status)
  if(status/=0) return
  if(any(perm<0).or.any(perm>7)) then
    status=2
    return
  endif
  do n=1,8
    native_nodes(n)=local(perm(n)+1)+1
  enddo
  ! Native geometry's returned global rho and center volume feed complete SMASS3.
  ! Actual repeated node indices are used here; no independent-node surrogate.
  call rear18_scatter(unique_x,values(148),values(135),native_nodes,mass)
end subroutine
