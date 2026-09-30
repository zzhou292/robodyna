! SPDX-License-Identifier: AGPL-3.0-or-later
! Source-shaped local arrays around the already compiled complete NORMP.
! No translated numerical operator, production helper or extra clock.
subroutine rd_current_main_normals(counts,x,irect,roles,neighbors,neighbor_edges,refs,stiffness, &
    actnor,tagnod,provided_free,prior,flag1_normals,normals,bound,bisectors,primary_skip,free_output, &
    free_edge_count,status) bind(C)
  use iso_c_binding
  use startup_native_mpi
  implicit none
  integer(c_int),intent(in)::counts(5)
  real(c_double),intent(in)::x(3,counts(1)),stiffness(counts(3))
  integer(c_int),intent(in)::irect(4,counts(3)),roles(counts(3)),neighbors(4,counts(3)),neighbor_edges(4,counts(3))
  integer(c_int),intent(in)::refs(4,counts(3)),actnor(counts(3)),tagnod(counts(1)),provided_free(max(1,counts(5)))
  real(c_float),intent(in)::prior(3,4,counts(3))
  real(c_float),intent(out)::flag1_normals(3,4,counts(3)),normals(3,4,counts(3)),bisectors(3,2,counts(4))
  integer(c_int),intent(out)::bound(counts(4)),primary_skip(counts(2)),free_output(4,4*counts(3)),free_edge_count,status
  integer::numnod,numels,nspmd,ninter25,nthread
  common /STARTUP_NATIVE_COUNTS/numnod,numels
  common /STARTUP_NATIVE_PARTITION/nspmd,ninter25
  common /STARTUP_NATIVE_THREADS/nthread
  integer::i,p,g,nrefs,nfree,nb_free
  integer::msr(counts(1)),tage(counts(3)),free_ids(counts(3)),free_bound(4,4*counts(3))
  integer::iadnor(4,counts(3)),dummy(1),dummy_matrix(1,1),fr_edge(2,2),ledge(15,4*counts(3))
  real(c_float)::work_normals(3,4,counts(3)),fskyt(3,counts(4)),fskyn(3,counts(4))
  real(c_double)::edge_stiffness(1)
  type(mpi_comm_nor_struct)::buffers
  numnod=counts(1);numels=0;nspmd=1;ninter25=1;nthread=1
  p=counts(2);g=counts(3);nrefs=counts(4)
  status=0;free_edge_count=0;free_output=0;primary_skip=0
  free_ids=0;nfree=0
  call I25FREE_BOUND(g,neighbors,irect,stiffness,nfree,free_ids)
  if(nfree/=counts(5))then
    status=1;return
  endif
  do i=1,nfree
    if(free_ids(i)/=provided_free(i))then
      status=1;return
    endif
  enddo
  ! MAIN_NORM clears VTX_BISECTOR each force-base update. FLAG1 resets LBOUND.
  ! Copy the exact supplied persistent face cache; unused T3 slots retain it.
  normals=prior;bisectors=0;bound=0;flag1_normals=prior
  tage=0;free_bound=0;nb_free=0;work_normals=0
  fskyt=0;fskyn=0;edge_stiffness=0;iadnor=0;ledge=0
  dummy=0;dummy_matrix=0;fr_edge=0
  do i=1,numnod
    msr(i)=i
  enddo
  call I25NORMP(1,g,p,irect,x,normals,numnod,msr,1,stiffness,edge_stiffness, &
      actnor,roles,tagnod,neighbors,neighbor_edges,dummy_matrix,fr_edge,work_normals,buffers, &
      0,0,ledge,bound,nrefs,refs,dummy_matrix,dummy,bisectors,1,nb_free,free_bound, &
      tage,free_ids,nfree,fskyt,iadnor,0,dummy,dummy,0,fskyn)
  flag1_normals=normals;primary_skip=tage(1:p)
  free_edge_count=nb_free
  if(nb_free>0)free_output(:,1:nb_free)=free_bound(:,1:nb_free)
  call I25NORMP(1,g,p,irect,x,normals,numnod,msr,1,stiffness,edge_stiffness, &
      actnor,roles,tagnod,neighbors,neighbor_edges,dummy_matrix,fr_edge,work_normals,buffers, &
      0,0,ledge,bound,nrefs,refs,dummy_matrix,dummy,bisectors,2,nb_free,free_bound, &
      tage,free_ids,nfree,fskyt,iadnor,0,dummy,dummy,0,fskyn)
end subroutine
