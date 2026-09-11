module cin_native_interfaces
  use iso_c_binding
  use h3d_mod
  implicit none
  integer, parameter :: max_nodes=4096, max_rows=128, max_elements=512
  interface
    subroutine I2FOR28_CIN(nsn,nmn,a,irect,dpara,msr,nsv,irtl,ms,weight,ar,iner,x,stifn,stifr, &
        fsav,dmast,adm,mmass,idel2,smass,siner,crst,fncont,indxc,miner,h3d,fncontp,ftcontp)
      import H3D_DATABASE, c_double
      integer :: nsn,nmn,irect(4,*),msr(*),nsv(*),irtl(*),weight(*),idel2,indxc(*)
      real(c_double) :: a(3,*),dpara(7,*),ms(*),ar(3,*),iner(*),x(3,*),stifn(*),stifr(*)
      real(c_double) :: fsav(*),dmast,adm(*),mmass(*),smass(*),siner(*),crst(2,*),fncont(3,*)
      real(c_double) :: miner(*),fncontp(3,*),ftcontp(3,*)
      type(H3D_DATABASE) :: h3d
    end subroutine
    subroutine I2VIROT3(nsn,nmn,a,irect,dpara,msr,nsv,irtl,v,ms,ar,vr,x,weight)
      import c_double
      integer :: nsn,nmn,irect(4,*),msr(*),nsv(*),irtl(*),weight(*)
      real(c_double) :: a(3,*),dpara(7,*),v(3,*),ms(*),ar(3,*),vr(3,*),x(3,*)
    end subroutine
    subroutine CHK2MSR3NB(nsn,nsv,itag,itask,irect,irtl,itag2,ixs,ixc,ixtg,ixq,iparg, &
        itagl,ms,iner,smas,siner,adm,cnel,addcnel,ofc,oft,oftg,ofur,nindg,bufs,nindex,tagel,itab,ilev)
      import c_double
      integer :: nsn,nsv(*),itag(*),itask,irect(4,*),irtl(*),itag2(*),ixs(10,*),ixc(6,*)
      integer :: ixtg(5,*),ixq(10,*),iparg(1,*),itagl(*),cnel(0:*),addcnel(0:*)
      integer :: ofc,oft,oftg,ofur,nindg,bufs(*),nindex(*),tagel(*),itab(*),ilev
      real(c_double) :: ms(*),iner(*),smas(*),siner(*),adm(*)
    end subroutine
  end interface
contains
  logical function valid_topology(n,r,masters,secondary)
    integer, intent(in) :: n,r,masters(4,r),secondary(r)
    logical :: dependent(max_nodes)
    integer :: k
    valid_topology=.false.
    if(n<1.or.n>max_nodes.or.r<1.or.r>max_rows) return
    if(any(masters<1).or.any(masters>n).or.any(secondary<1).or.any(secondary>n)) return
    dependent=.false.
    do k=1,r
      if(dependent(secondary(k))) return
      dependent(secondary(k))=.true.
    end do
    do k=1,r
      if(any(dependent(masters(:,k)))) return
    end do
    valid_topology=.true.
  end function
end module
