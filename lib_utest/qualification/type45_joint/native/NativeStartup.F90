module T45_STARTUP_PACKET
  use iso_c_binding
  use, intrinsic :: ieee_arithmetic
  use T45_INTERFACES
  use T45_PROPERTY, only: SetProperty
  use T45_MESSAGE_MOD, only: errors,warnings
  implicit none
  interface
    subroutine T45_RESET_CONTEXT(time,step,cycle,unit)
      real(8), intent(in) :: time,step
      integer, intent(in) :: cycle,unit
    end subroutine
    subroutine T45_AUTOMATIC(kind,scf,dt,x,mass,inertia,stifn,stifr,npby,uvar,observation,unit)
      integer, intent(in) :: kind,npby(6,2),unit
      real(8), intent(in) :: scf,dt,x(3,5),mass(5),inertia(5),stifn(5),stifr(5)
      real(8), intent(inout) :: uvar(39)
      real(8), intent(out) :: observation(5)
    end subroutine
  end interface
contains
  subroutine type45_native_startup(kind,property,xinput,roles,damping,coefficient,dt,uvar,observation,status) bind(C)
    integer(c_int), intent(in) :: kind,roles(2)
    real(c_double), intent(in) :: property(14),xinput(3,5),damping(2,2),coefficient(4,2),dt
    real(c_double), intent(inout) :: uvar(39),observation(13)
    integer(c_int), intent(out) :: status
    call StartupPhase(kind,property,xinput,roles,damping,coefficient,dt,.true.,uvar,observation,status)
  end subroutine
  subroutine StartupPhase(kind,property,xinput,roles,damping,coefficient,dt,automatic_enabled,uvar,observation,status)
    integer(c_int), intent(in) :: kind,roles(2)
    real(c_double), intent(in) :: property(14),xinput(3,5),damping(2,2),coefficient(4,2),dt
    logical, intent(in) :: automatic_enabled
    real(c_double), intent(inout) :: uvar(39),observation(13)
    integer(c_int), intent(out) :: status
    real(8) :: x(3,5),xl(1,3),mass(1),inertia(1),stifn(1),stifr(1),viscm(1),viscr(1)
    real(8) :: var(39,1),rby(14,2),ms(5),iner(5),sn(5),sr(5),gmass(1),automatic(5),next(13)
    integer :: ix(4,1),ixr(6,1),extra(5,2),npby(6,2),members(2),itab(5),igeo(32),i,n,unit
    character(len=NCHARTITLE) :: title
    status=1
    if(kind<1.or.kind>3.or.any(roles<0).or.any(roles>2)) return
    if(.not.all(ieee_is_finite(property)).or..not.all(ieee_is_finite(xinput)).or. &
       .not.all(ieee_is_finite(damping)).or..not.all(ieee_is_finite(coefficient)).or..not.ieee_is_finite(dt)) return
    if(property(1)<=0.or.property(2)<=0.or.property(2)>1) return
    if(automatic_enabled.and.dt<=0) return
    if(.not.automatic_enabled.and.dt/=0) return
    open(newunit=unit,status='scratch')
    call T45_RESET_CONTEXT(0.d0,0.d0,0,unit)
    call SetProperty(kind,property)
    errors=0
    warnings=0
    x=xinput
    ix(:,1)=[1,2,3,71]
    ixr(:,1)=[1,1,2,3,1,71]
    if(kind==1) then
      ix(3,1)=0
      ixr(4,1)=0
    endif
    ! GET_SKEW45's ID_KJ is defined by this matching, zero-extra row. This
    ! explicit decoded registry context does not claim an original SDI export.
    extra=0
    extra(4,1)=71
    extra(1,2)=1
    xl(1,:)=x(:,2)-x(:,1)
    var=0
    mass=0
    inertia=0
    stifn=0
    stifr=0
    viscm=0
    viscr=0
    title='supplied TYPE45 scalar profile'
    call T45_RINI45(1,unit,1,ix,x,xl,mass,inertia,stifn,stifr,viscm,viscr,var,39,ixr,extra,71,title)
    next(1:4)=[stifn(1),stifr(1),viscm(1),viscr(1)]
    npby=0
    rby=0
    members=0
    ms=0
    iner=0
    sn=0
    sr=0
    n=0
    do i=1,2
      npby(1,i)=3+i
      npby(6,i)=200+i
      if(roles(i)==1) then
        n=n+1
        npby(2,i)=1
        members(n)=i
      else if(roles(i)==2) then
        npby(1,i)=i
      endif
      rby(14,i)=damping(1,i)
      rby(10:12,i)=damping(2,i)
      ms(i)=damping(1,i)
      iner(i)=damping(2,i)
      ms(3+i)=coefficient(1,i)
      iner(3+i)=coefficient(2,i)
      sn(3+i)=coefficient(3,i)
      sr(3+i)=coefficient(4,i)
      if(roles(i)==0) then
        sn(i)=coefficient(3,i)
        sr(i)=coefficient(4,i)
      endif
    enddo
    itab=[101,102,103,201,202]
    igeo=0
    igeo(1)=71
    gmass=0
    call T45_RINI45_RB(1,39,1,ixr,npby,members,rby,stifr,var,itab,igeo,extra,gmass,ms,iner)
    if(errors/=0) then
      close(unit)
      status=2
      return
    endif
    automatic=0
    if(automatic_enabled) call T45_AUTOMATIC(kind,property(1),dt,x,ms,iner,sn,sr,npby,var(:,1),automatic,unit)
    next(5:9)=automatic
    next(10:13)=var(34:37,1)
    close(unit)
    if(.not.all(ieee_is_finite(var)).or..not.all(ieee_is_finite(next))) return
    uvar=var(:,1)
    observation=next
    status=0
  end subroutine
end module
