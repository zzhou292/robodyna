! Expected-data-only caller: original ININT3 initial clear, whole COR3/PEN3/PWR3.
subroutine initial_state_full_history_native(dims,x,itab,irect,nsv,candn,cande,admsr,neighbors, &
    normals,lbound,bisectors,gaps,gapnm,segment_type,irtlm,pene,time_s,icont,global_ids,sharp,prior_irtlm,prior_pene,source_main_gap) bind(C)
  use iso_c_binding
  use initial_state_constants
  implicit none
  integer(c_int),intent(in)::dims(5),itab(*),irect(4,*),nsv(*),candn(*),cande(*)
  integer(c_int),intent(in)::admsr(4,*),neighbors(4,*),lbound(*),segment_type(*)
  real(c_double),intent(in)::x(3,*),gaps(*),gapnm(4,*),source_main_gap(*)
  real(c_float),intent(in)::normals(3,4,*),bisectors(3,2,*)
  integer(c_int),intent(out)::irtlm(4,*),icont(*),prior_irtlm(4,*)
  integer(c_int),intent(in)::global_ids(*),sharp
  real(c_double),intent(out)::pene(5,*),time_s(*),prior_pene(5,*)
  integer,parameter::MVSIZ=512
  integer::n,ns,g,r,jlt,i,k,warnings,warning0,total,first
  integer::ix1(MVSIZ),ix2(MVSIZ),ix3(MVSIZ),ix4(MVSIZ),nsvg(MVSIZ)
  integer::mvoisn(MVSIZ,4),ibound(4,MVSIZ),mseglo(160),msr(256)
  real(c_double)::xi(MVSIZ),yi(MVSIZ),zi(MVSIZ)
  real(c_double)::xx(MVSIZ,5),yy(MVSIZ,5),zz(MVSIZ,5)
  real(c_double)::nnx(MVSIZ,5),nny(MVSIZ,5),nnz(MVSIZ,5)
  real(c_double)::gap_s(MVSIZ),gap_m(MVSIZ),gap_n(4,MVSIZ),gap_mxl(MVSIZ)
  real(c_double)::main_max(160),main_limit(160),secondary_limit(128),stfn(128),penmin,penmax
  character(len=128)::title
  n=dims(1);ns=dims(2);g=dims(3);r=dims(4);jlt=dims(5)
  if(n<1.or.n>256.or.ns<1.or.ns>128.or.g<1.or.g>160.or.r<1.or.r>640.or.jlt<0.or.jlt>4096) &
      error stop 'Bounded native initial-history extent'
  do i=1,ns
    if(nsv(i)<1.or.nsv(i)>n)error stop 'Native history secondary ordinal'
  enddo
  do i=1,g
    do k=1,4
      if(irect(k,i)<1.or.irect(k,i)>n.or.admsr(k,i)<1.or.admsr(k,i)>r) &
          error stop 'Native history main/reference ordinal'
    enddo
    mseglo(i)=global_ids(i)
    main_max(i)=source_main_gap(i) ! Actual post-INI/pre-BUC GAP_M, unchanged by BUC's corner1 write.
  enddo
  do i=1,jlt
    if(candn(i)<1.or.candn(i)>ns.or.cande(i)<1.or.cande(i)>g)error stop 'Native history candidate ordinal'
  enddo
  irtlm(1:4,1:ns)=0;pene(1:5,1:ns)=ZERO;time_s(1:2*ns)=ZERO;icont(1:ns)=0
  stfn=ONE;msr=1;main_limit=EP30;secondary_limit=EP30
  ! PENMIN's conditional branches assign identical state in ordinary INACTI5;
  ! PENMAX is an unused dummy in this whole routine. Zero selects no invented
  ! tolerance, and IRESP0 excludes its precision-dependent update.
  penmin=ZERO;penmax=ZERO;warnings=0;warning0=0;title='initial source history'
  prior_irtlm(1:4,1:ns)=0;prior_pene(1:5,1:ns)=ZERO
  if(jlt==0)return
  time_s(1:ns)=EP20
  total=jlt
  do first=1,total,128
  jlt=min(128,total-first+1)
  call INITIAL_STATE_COR3(jlt,1,x,irect,nsv,cande(first),candn(first),xi,yi,zi, &
      ix1,ix2,ix3,ix4,nsvg,ns,gaps,gap_s,admsr,normals, &
      xx(:,1),xx(:,2),xx(:,3),xx(:,4),xx(:,5), &
      yy(:,1),yy(:,2),yy(:,3),yy(:,4),yy(:,5), &
      zz(:,1),zz(:,2),zz(:,3),zz(:,4),zz(:,5), &
      nnx,nny,nnz,neighbors,mvoisn,main_max,gap_m,gapnm,gap_n, &
      secondary_limit,main_limit,gap_mxl,lbound,ibound)
  call INITIAL_STATE_PEN3(jlt,candn(first),cande(first),penmin,penmax, &
      xx(:,1),xx(:,2),xx(:,3),xx(:,4),xx(:,5), &
      yy(:,1),yy(:,2),yy(:,3),yy(:,4),yy(:,5), &
      zz(:,1),zz(:,2),zz(:,3),zz(:,4),zz(:,5),xi,yi,zi,ns, &
      ix1,ix2,ix3,ix4,nsvg,g,mseglo,gap_s,irect,irtlm,time_s,pene,itab, &
      segment_type,sharp,nnx,nny,nnz,gap_n,mvoisn,gap_mxl,1,ibound,bisectors,1,5)
  enddo
  jlt=total
  prior_irtlm(1:4,1:ns)=irtlm(1:4,1:ns);prior_pene(1:5,1:ns)=pene(1:5,1:ns)
  call INITIAL_STATE_PWR3(itab,5,cande,candn,stfn,x,jlt,nsv,warnings,pene, &
      1,25,msr,irtlm,irect,ns,1,title,mseglo,icont,warning0,penmin,0)
  ! Literal ININT3 post-PWR normalization for INACTI5.
  pene(1:2,1:ns)=ZERO
end subroutine
