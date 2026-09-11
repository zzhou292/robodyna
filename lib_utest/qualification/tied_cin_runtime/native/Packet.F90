subroutine tl_cin_native_stage(n,r,masters,secondary,x,st,force,couple,mass,inertia,stiff,stiffr, &
    saved_mass,saved_inertia,numerical_mass,force_integral,dpara,velocity,omega,acceleration, &
    angular_acceleration,time,step,kick,status) bind(C)
  use iso_c_binding
  use ieee_arithmetic
  use cin_native_interfaces
  use cin_time_context
  implicit none
  integer(c_int), value :: n,r
  integer(c_int), intent(in) :: masters(4,r),secondary(r)
  real(c_double), intent(in) :: x(3,n),st(2,r)
  real(c_double), intent(inout) :: force(3,n),couple(3,n),mass(n),inertia(n),stiff(n),stiffr(n)
  real(c_double), intent(inout) :: saved_mass(r),saved_inertia(r),numerical_mass,force_integral(6)
  real(c_double), intent(inout) :: dpara(7,r),velocity(3,n),omega(3,n),acceleration(3,n),angular_acceleration(3,n)
  real(c_double), value :: time,step,kick
  integer(c_int), intent(out) :: status
  real(c_double) :: a(3,max_nodes),ar(3,max_nodes),ms(max_nodes),iner(max_nodes),miner(max_nodes)
  real(c_double) :: sn(max_nodes),sr(max_nodes),sm(max_rows),si(max_rows),dp(7,max_rows),dm,fsav(6)
  real(c_double) :: v(3,max_nodes),vr(3,max_nodes),aa(3,max_nodes),ara(3,max_nodes)
  real(c_double) :: xx(3,max_nodes),crst(2,max_rows),adm(max_nodes),mmass(max_nodes)
  real(c_double) :: fn(3,max_nodes),fnp(3,max_nodes),ftp(3,max_nodes)
  integer :: irect(4,max_rows),nsv(max_rows),irtl(max_rows),indxc(max_rows),msr(max_nodes),weight(max_nodes)
  logical :: dependent(max_nodes)
  type(H3D_DATABASE) :: h3d
  integer :: i
  status=1
  if(.not.valid_topology(n,r,masters,secondary)) return
  if(.not.ieee_is_finite(time).or..not.ieee_is_finite(step).or..not.ieee_is_finite(kick)) return
  if(time<0.or.step<=0.or.kick<=0.or.kick>step) return
  if(.not.all(ieee_is_finite(x)).or..not.all(ieee_is_finite(st))) return
  if(.not.all(ieee_is_finite(force)).or..not.all(ieee_is_finite(couple))) return
  if(.not.all(ieee_is_finite(velocity)).or..not.all(ieee_is_finite(omega))) return
  if(.not.all(ieee_is_finite(mass)).or..not.all(ieee_is_finite(inertia))) return
  if(.not.all(ieee_is_finite(stiff)).or..not.all(ieee_is_finite(stiffr))) return
  if(.not.all(ieee_is_finite(saved_mass)).or..not.all(ieee_is_finite(saved_inertia))) return
  if(.not.ieee_is_finite(numerical_mass).or..not.all(ieee_is_finite(force_integral))) return
  if(any(mass<0).or.any(inertia<0).or.any(stiff<0).or.any(stiffr<0)) return
  if(any(saved_mass<0).or.any(saved_inertia<0)) return
  dependent=.false.
  dependent(secondary)=.true.
  do i=1,n
    if(.not.dependent(i).and.(mass(i)<=0.or.inertia(i)<=0)) return
  end do
  a=0; ar=0; ms=0; iner=0; miner=0; sn=0; sr=0; sm=0; si=0; dp=0
  v=0; vr=0; aa=0; ara=0; xx=0; crst=0; adm=0; mmass=0; fn=0; fnp=0; ftp=0
  irect=1; nsv=1; irtl=1; indxc=1; msr=1; weight=1
  a(:,1:n)=force; ar(:,1:n)=couple; ms(1:n)=mass; iner(1:n)=inertia
  ! INTTI1's entry-IN snapshot, before any row transfer.
  miner(1:n)=inertia
  sn(1:n)=stiff; sr(1:n)=stiffr; sm(1:r)=saved_mass; si(1:r)=saved_inertia
  xx(:,1:n)=x; crst(:,1:r)=st; irect(:,1:r)=masters; nsv(1:r)=secondary
  v(:,1:n)=velocity; vr(:,1:n)=omega; dm=numerical_mass; fsav=force_integral
  do i=1,r
    irtl(i)=i
    indxc(i)=i
  end do
  do i=1,n
    msr(i)=i
    mmass(i)=mass(i)
  end do
  TT=time; DT1=step; DT2=step; DT12=step
  call I2FOR28_CIN(r,n,a,irect,dp,msr,nsv,irtl,ms,weight,ar,iner,xx,sn,sr, &
      fsav,dm,adm,mmass,1,sm,si,crst,fn,indxc,miner,h3d,fnp,ftp)
  ! Packet-only ordinary free-DOF kick, with the actual post-transfer current
  ! coefficients. This is explicitly not an extraction of the engine integrator.
  do i=1,n
    if(dependent(i)) cycle
    aa(:,i)=(1/ms(i))*a(:,i)
    ara(:,i)=(1/iner(i))*ar(:,i)
    v(:,i)=v(:,i)+kick*aa(:,i)
    vr(:,i)=vr(:,i)+kick*ara(:,i)
  end do
  call I2VIROT3(r,n,aa,irect,dp,msr,nsv,irtl,v,ms,ara,vr,xx,weight)
  status=2
  if(.not.all(ieee_is_finite(a(:,1:n))).or..not.all(ieee_is_finite(ar(:,1:n)))) return
  if(.not.all(ieee_is_finite(ms(1:n))).or..not.all(ieee_is_finite(iner(1:n)))) return
  if(.not.all(ieee_is_finite(sn(1:n))).or..not.all(ieee_is_finite(sr(1:n)))) return
  if(.not.all(ieee_is_finite(dp(:,1:r))).or..not.ieee_is_finite(dm)) return
  if(.not.all(ieee_is_finite(v(:,1:n))).or..not.all(ieee_is_finite(vr(:,1:n)))) return
  if(.not.all(ieee_is_finite(aa(:,1:n))).or..not.all(ieee_is_finite(ara(:,1:n)))) return
  if(.not.all(ieee_is_finite(fsav))) return
  force=a(:,1:n); couple=ar(:,1:n); mass=ms(1:n); inertia=iner(1:n)
  stiff=sn(1:n); stiffr=sr(1:n); saved_mass=sm(1:r); saved_inertia=si(1:r)
  numerical_mass=dm; force_integral=fsav; dpara=dp(:,1:r)
  velocity=v(:,1:n); omega=vr(:,1:n); acceleration=aa(:,1:n); angular_acceleration=ara(:,1:n)
  status=0
end subroutine
