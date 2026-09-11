! Test packet only; the complete I2VIROT3 routine is compiled unchanged.
subroutine tl_tied_motion(x, dpara, velocity, acceleration, repeated, output) bind(C, name="tl_tied_patch_motion")
  use iso_c_binding
  implicit none
  real(c_double), intent(in) :: x(3,5), dpara(7,1), velocity(3,4), acceleration(3,4)
  real(c_double), intent(out) :: output(3,4)
  integer(c_int), intent(in) :: repeated
  real(c_double) :: v(3,5), a(3,5), vr(3,5), ar(3,5), mass(5)
  integer :: irect(4,1), msr(4), nsv(1), irtl(1), weight(5)
  v=0; a=0; vr=0; ar=0; mass=0
  v(:,1:4)=velocity; a(:,1:4)=acceleration
  irect(:,1)=[1,2,3,4]; msr=[1,2,3,4]; nsv=5; irtl=1; weight=1
  if (repeated == 1) irect(4,1)=3
  call I2VIROT3(1,4,a,irect,dpara,msr,nsv,irtl,v,mass,ar,vr,x,weight)
  output(:,1)=v(:,5); output(:,2)=vr(:,5)
  output(:,3)=a(:,5); output(:,4)=ar(:,5)
end subroutine
