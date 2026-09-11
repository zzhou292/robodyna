module T45_STEP_PACKET
  use iso_c_binding
  use, intrinsic :: ieee_arithmetic
  use T45_INTERFACES
  use T45_STARTUP_PACKET, only: T45_RESET_CONTEXT
  use T45_PROPERTY, only: SetProperty
  implicit none
contains
  subroutine type45_native_step(kind,property,x,spin,time,dt,cycle,uvar,history,observation,status) bind(C)
    integer(c_int), intent(in) :: kind,cycle
    real(c_double), intent(in) :: property(14),x(3,2),spin(3,2),time,dt
    real(c_double), intent(inout) :: uvar(39),history(13),observation(25)
    integer(c_int), intent(out) :: status
    call StepPhase(kind,property,x,spin,time,dt,cycle,.false.,uvar,history,observation,status)
  end subroutine
  subroutine StepPhase(kind,property,x,spin,time,dt,cycle,constructor,uvar,history,observation,status)
    integer(c_int), intent(in) :: kind,cycle
    real(c_double), intent(in) :: property(14),x(3,2),spin(3,2),time,dt
    logical, intent(in) :: constructor
    real(c_double), intent(inout) :: uvar(39),history(13),observation(25)
    integer(c_int), intent(out) :: status
    real(8) :: var(39,1),d(1,3),r(1,3),force(1,3),moment(1,3),eint(1),xl(1,3)
    real(8) :: rot1(3,1),rot2(3,1),rby(1),xkm(1),xkr(1),xcm(1),xcr(1),mass(1),iner(1),off(1)
    real(8) :: x0_error(3,1),sti(3,1),stir(3,1),dt2t,msrt(1),dmelt(1)
    real(8) :: f(3,2),couple(3,2),stifn(2),stifr(2),out_force(1,6),out_couple(1,6),next(25),h(13)
    integer :: ixr(6,1),nc1(1),nc2(1),isens,neltst,ityptst,unit,i,offset
    type(SENSOR_STR_) :: sensors(0)
    status=1
    if(kind<1.or.kind>3) return
    if(constructor) then
      if(cycle/=0.or.dt/=0.or.time/=0) return
    else
      if(cycle<1.or.dt<=0.or.time<=0) return
    endif
    if(.not.all(ieee_is_finite(property)).or..not.all(ieee_is_finite(x)).or. &
       .not.all(ieee_is_finite(spin)).or..not.ieee_is_finite(time).or..not.ieee_is_finite(dt).or. &
       .not.all(ieee_is_finite(uvar)).or..not.all(ieee_is_finite(history))) return
    open(newunit=unit,status='scratch')
    call T45_RESET_CONTEXT(time,dt,cycle,unit)
    call SetProperty(kind,property)
    var(:,1)=uvar
    d(1,:)=history(1:3)
    r(1,:)=history(4:6)
    force(1,:)=history(7:9)
    moment(1,:)=history(10:12)
    eint(1)=history(13)
    ixr(:,1)=[1,1,2,0,1,71]
    xl=0
    rot1=0
    rot2=0
    rby=0
    x0_error=0
    xkm=0
    xkr=0
    xcm=0
    xcr=0
    mass=0
    iner=0
    off=1
    call T45_RSKEW33(1,1,ixr,unit,1,39,var,rby,x,xl,rot1,rot2, &
      d(:,1),d(:,2),d(:,3),r(:,1),r(:,2),r(:,3),spin,45,0,sensors,isens,nc1,nc2,x)
    call T45_RUSER33(1,unit,1,39,var,force(:,1),force(:,2),force(:,3), &
      moment(:,1),moment(:,2),moment(:,3),xkm,xkr,xcm,xcr,xl,mass,iner,off,eint, &
      rot1,rot2,d(:,1),d(:,2),d(:,3),r(:,1),r(:,2),r(:,3),45,isens,x0_error)
    sti=0
    stir=0
    msrt=0
    dmelt=0
    dt2t=1d30
    neltst=0
    ityptst=0
    call T45_RDTIME33(1,1,dt2t,neltst,ityptst,ixr,eint,sti,stir,off,xkm,xkr,xcm,xcr, &
      mass,iner,force(:,1),force(:,2),force(:,3),moment(:,1),moment(:,2),moment(:,3), &
      rot1,rot2,msrt,dmelt,39,var,45,0)
    f=0
    couple=0
    stifn=0
    stifr=0
    call T45_RCUM33(1,1,xl,nc1,nc2,f,force(:,1),force(:,2),force(:,3),couple, &
      moment(:,1),moment(:,2),moment(:,3),sti,stir,stifn,stifr, &
      out_force(:,1),out_force(:,2),out_force(:,3),out_force(:,4),out_force(:,5),out_force(:,6), &
      out_couple(:,1),out_couple(:,2),out_couple(:,3),out_couple(:,4),out_couple(:,5),out_couple(:,6),39,var)
    close(unit)
    do i=1,2
      offset=8*(i-1)
      next(offset+1:offset+3)=f(:,i)
      next(offset+4:offset+6)=couple(:,i)
      next(offset+7)=stifn(i)
      next(offset+8)=stifr(i)
    enddo
    next(17:20)=[xkm(1),xkr(1),xcm(1),xcr(1)]
    next(21:23)=xl(1,:)
    next(24:25)=[mass(1),iner(1)]
    h(1:3)=d(1,:)
    h(4:6)=r(1,:)
    h(7:9)=force(1,:)
    h(10:12)=moment(1,:)
    h(13)=eint(1)
    if(.not.all(ieee_is_finite(var)).or..not.all(ieee_is_finite(h)).or..not.all(ieee_is_finite(next))) return
    uvar=var(:,1)
    history=h
    observation=next
    status=0
  end subroutine
end module
