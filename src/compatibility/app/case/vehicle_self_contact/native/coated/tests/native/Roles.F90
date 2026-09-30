! SPDX-License-Identifier: AGPL-3.0-or-later
subroutine rd_coated_roles(nnode,nsolid,nshell,x,solids,shells,kinds,offsets,incidences,roles) bind(C)
  use iso_c_binding
  use element_mod
  implicit none
  integer(c_int),value::nnode,nsolid,nshell
  real(c_double),intent(in)::x(3,nnode)
  integer(c_int),intent(in)::solids(nixs,*),shells(4,*),kinds(*),offsets(nnode+1),incidences(*)
  integer(c_int),intent(out)::roles(*)
  integer::numelc,numeltg,numels,numels8,numels10,numels16,numels20
  common /COATED_REFERENCE_COUNTS/ numelc,numeltg,numels,numels8,numels10,numels16,numels20
  integer::ixc(nixc,1),ixtg(nixtg,1),dummy_offsets(nnode+1),dummy_incidence(1)
  integer::ixs10(6,1),ixs16(8,1),ixs20(12,1),irect(4),i
  if(nnode<1.or.nnode>1024.or.nsolid<0.or.nsolid>256.or.nshell<0.or.nshell>256) &
      error stop 'Classification reference cap'
  numelc=0;numeltg=0;numels=nsolid;numels8=nsolid
  numels10=0;numels16=0;numels20=0
  ixc=0;ixtg=0;dummy_offsets=0;dummy_incidence=0
  ixs10=0;ixs16=0;ixs20=0
  do i=1,nshell
    if(kinds(i)/=3.and.kinds(i)/=7)error stop 'Incoming role is not an ordinary shell'
    roles(i)=kinds(i)
    irect=shells(:,i)
    call IN24COQ_SOL3(irect,ixc,ixtg,roles(i),x,dummy_offsets,dummy_offsets, &
        dummy_incidence,dummy_incidence,offsets,incidences,solids,ixs10,ixs16,ixs20)
  end do
end subroutine
