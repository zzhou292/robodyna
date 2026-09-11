subroutine tl_cin_native_witness(n,r,nq,nt,masters,secondary,quads,triangles,active,node_active, &
    saved_mass,saved_inertia,signed_secondary,mass,inertia,status) bind(C)
  use iso_c_binding
  use ieee_arithmetic
  use cin_native_interfaces
  implicit none
  integer(c_int), value :: n,r,nq,nt
  integer(c_int), intent(in) :: masters(4,r),secondary(r),quads(4,nq),triangles(3,nt)
  integer(c_int), intent(in) :: active(nq+nt),node_active(n)
  real(c_double), intent(in) :: saved_mass(r),saved_inertia(r)
  integer(c_int), intent(inout) :: signed_secondary(r)
  real(c_double), intent(inout) :: mass(n),inertia(n)
  integer(c_int), intent(out) :: status
  integer :: nsv(max_rows),irect(4,max_rows),irtl(max_rows),itag(max_nodes),itag2(max_nodes)
  integer :: ixs(10,1),ixq(10,1),ixc(6,max_elements),ixtg(5,max_elements),iparg(1,1)
  integer :: itagl(max_nodes),tagel(max_elements),itab(max_nodes),bufs(4*max_rows),nindex(max_rows)
  integer :: cnel(0:4*max_elements),addcnel(0:max_nodes+1),nindg,i,j,k
  real(c_double) :: ms(max_nodes),iner(max_nodes),sm(max_rows),si(max_rows),adm(max_nodes)
  status=1
  if(.not.valid_topology(n,r,masters,secondary)) return
  if(nq<0.or.nt<0.or.nq+nt<1.or.nq+nt>max_elements) return
  if(any(quads<1).or.any(quads>n).or.any(triangles<1).or.any(triangles>n)) return
  if(any(active<0).or.any(active>1).or.any(node_active<0).or.any(node_active>1)) return
  if(.not.all(ieee_is_finite(saved_mass)).or..not.all(ieee_is_finite(saved_inertia))) return
  if(.not.all(ieee_is_finite(mass)).or..not.all(ieee_is_finite(inertia))) return
  if(any(saved_mass<0).or.any(saved_inertia<0).or.any(mass<0).or.any(inertia<0)) return
  nsv=1; irect=1; irtl=1; itag=1; itag2=1; ixs=1; ixq=1; ixc=1; ixtg=1; iparg=0
  itagl=0; tagel=0; itab=1; bufs=0; nindex=0; cnel=0; addcnel=0; nindg=0
  ms=0; iner=0; sm=0; si=0; adm=0
  nsv(1:r)=secondary; irect(:,1:r)=masters; itag(1:n)=node_active
  ms(1:n)=mass; iner(1:n)=inertia; sm(1:r)=saved_mass; si(1:r)=saved_inertia
  ixc(2:5,1:nq)=quads; ixtg(2:4,1:nt)=triangles; tagel(1:nq+nt)=active
  do i=1,r
    irtl(i)=i
  end do
  ! Complete incidence of this independently declared tiny shell inventory.
  ! One-based native CNEL entries; each element is listed once per node.
  k=1
  do i=1,n
    itab(i)=i
    addcnel(i)=k
    do j=1,nq
      if(any(quads(:,j)==i)) then
        cnel(k)=j
        k=k+1
      end if
    end do
    do j=1,nt
      if(any(triangles(:,j)==i)) then
        cnel(k)=nq+j
        k=k+1
      end if
    end do
  end do
  addcnel(n+1)=k
  call CHK2MSR3NB(r,nsv,itag,0,irect,irtl,itag2,ixs,ixc,ixtg,ixq,iparg,itagl, &
      ms,iner,sm,si,adm,cnel,addcnel,0,nq,nq,nq+nt,nindg,bufs,nindex,tagel,itab,28)
  if(nindg/=0) return
  signed_secondary=nsv(1:r)
  mass=ms(1:n)
  inertia=iner(1:n)
  status=0
end subroutine
