! SPDX-License-Identifier: AGPL-3.0-or-later
! Test-only explicit HM_READ_MAT44 CC0/CP1/ISRATE1/VP2 packet.
! Complete authenticated SIGEPS44C remains the constitutive oracle.
subroutine filtered_zero_c_point(analytic,npts,curve,e,nu,rho,gs,a,etan,base,deps,rate, &
    layer,thickness,parent,time,values,b) bind(C,name='filtered_zero_c_point')
  use iso_c_binding, only: c_double,c_int
  use LAW44_POINT_BRIDGE_MOD, only: LAW44_POINT_PACKET
  implicit none
  integer(c_int),value :: analytic,npts
  real(c_double),value :: e,nu,rho,gs,a,etan,layer,thickness,parent,time
  real(c_double),intent(in) :: curve(2,npts+1),base(6),deps(5),rate(5)
  real(c_double),intent(out) :: values(13),b
  integer(c_int) :: mfunc
  b=0.0_c_double
  mfunc=1_c_int
  if(analytic==1) then
    mfunc=0_c_int
    ! Exact converter order; this wrapper does not use production preparation.
    b=etan*e/(e-etan)
  endif
  call LAW44_POINT_PACKET(mfunc,a,b,npts,curve,e,nu,rho,gs,base,deps,rate, &
      layer,thickness,values,parent,time,1_c_int)
end subroutine
