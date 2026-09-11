module T45_INTERFACES
  use T45_SENSOR_MOD, only: SENSOR_STR_
  use T45_NAMES_AND_TITLES_MOD, only: NCHARTITLE
  implicit none
  interface
    subroutine T45_RINI45(nel,iout,iprop,ix,x,xl,mass,xiner,stifn,stifr,viscm,viscr,uvar,nuvar,ixr,ixr_kj,id,titr)
      import NCHARTITLE
      integer, intent(in) :: nel,iout,iprop,nuvar,id,ix(4,1),ixr(6,*),ixr_kj(5,*)
      real(8), intent(in) :: x(3,*)
      real(8), intent(inout) :: xl(1,3),mass(nel),xiner(nel),stifn(nel),stifr(nel),viscm(nel),viscr(nel),uvar(nuvar,*)
      character(len=NCHARTITLE), intent(in) :: titr
    end subroutine
    subroutine T45_RINI45_RB(nel,nuvar,iprop,ixr,npby,lpby,rby,stifr,uvar,itab,igeo,ixr_kj,gmass,ms,iner)
      integer, intent(in) :: nel,nuvar,iprop,ixr(6,*),npby(6,*),lpby(*),itab(*),igeo(32),ixr_kj(5,*)
      real(8), intent(in) :: rby(14,*),ms(*),iner(*)
      real(8), intent(inout) :: stifr(*),uvar(nuvar,*),gmass(*)
    end subroutine
    subroutine T45_RSKEW33(jft,jlt,ixr,iout,iprop,nuvar,uvar,rby,x,xl,rot1,rot2,dx,dy,dz,rx,ry,rz,vr, &
                            igtyp,nsensor,sensor_tab,isens,nc1,nc2,xdp)
      import SENSOR_STR_
      integer, intent(in) :: jft,jlt,ixr(6,*),iout,iprop,nuvar,igtyp,nsensor
      integer, intent(out) :: isens,nc1(*),nc2(*)
      real(8), intent(in) :: rby(*),x(3,*),vr(3,*),xdp(3,*)
      real(8), intent(inout) :: uvar(nuvar,*),xl(1,3),rot1(3,1),rot2(3,1),dx(*),dy(*),dz(*),rx(*),ry(*),rz(*)
      type(SENSOR_STR_), intent(in) :: sensor_tab(nsensor)
    end subroutine
    subroutine T45_RUSER33(nel,iout,iprop,nuvar,uvar,fx,fy,fz,xmom,ymom,zmom,xkm,xkr,xcm,xcr,xl,mass,iner,off,eint, &
                           rot1,rot2,dx,dy,dz,rx,ry,rz,igtyp,isens,x0_err)
      integer, intent(in) :: nel,iout,iprop,nuvar,igtyp,isens
      real(8), intent(in) :: xl(1,3),rot1(3,1),rot2(3,1)
      real(8), intent(inout) :: uvar(nuvar,*),fx(*),fy(*),fz(*),xmom(*),ymom(*),zmom(*),xkm(*),xkr(*),xcm(*),xcr(*)
      real(8), intent(inout) :: mass(*),iner(*),off(*),eint(*),dx(*),dy(*),dz(*),rx(*),ry(*),rz(*),x0_err(3,*)
    end subroutine
    subroutine T45_RDTIME33(jft,jlt,dt2t,neltst,ityptst,ixr,eint,sti,stir,off,xkm,xkr,xcm,xcr,umas,uiner, &
                            fx,fy,fz,xmom,ymom,zmom,rot1,rot2,msrt,dmelt,nuvar,uvar,jntyp,jsms)
      integer, intent(in) :: jft,jlt,ixr(6,*),nuvar,jntyp,jsms
      integer, intent(inout) :: neltst,ityptst
      real(8), intent(inout) :: dt2t,eint(*),sti(3,*),stir(3,*),off(*),xkm(*),xkr(*),xcm(*),xcr(*)
      real(8), intent(inout) :: umas(*),uiner(*),fx(*),fy(*),fz(*),xmom(*),ymom(*),zmom(*)
      real(8), intent(inout) :: rot1(*),rot2(*),msrt(*),dmelt(*),uvar(nuvar,*)
    end subroutine
    subroutine T45_RCUM33(jft,jlt,xl,nc1,nc2,f,forx,fory,forz,xm,xmom,ymom,zmom,sti,stir,stifn,stifr, &
                          fx1,fx2,fy1,fy2,fz1,fz2,mx1,mx2,my1,my2,mz1,mz2,nuvar,uvar)
      integer, intent(in) :: jft,jlt,nc1(*),nc2(*),nuvar
      real(8), intent(in) :: xl(1,3),forx(*),fory(*),forz(*),xmom(*),ymom(*),zmom(*),uvar(nuvar,*)
      real(8), intent(inout) :: f(3,*),xm(3,*),sti(3,*),stir(3,*),stifn(*),stifr(*)
      real(8), intent(out) :: fx1(*),fx2(*),fy1(*),fy2(*),fz1(*),fz2(*),mx1(*),mx2(*),my1(*),my2(*),mz1(*),mz2(*)
    end subroutine
  end interface
end module
