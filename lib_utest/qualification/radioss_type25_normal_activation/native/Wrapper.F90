! SPDX-License-Identifier: AGPL-3.0-or-later
! Full I25FREE_BOUND and I25TAGN; local serial qualification composition only.
subroutine na_masks(counts,irect,roles,neighbors,admsr,main_stiffness,secondary_stiffness, &
    history,normal_offsets,normal_mains,removal_offsets,removed_mains,optimized, &
    active,tags,free_count,free_ids) bind(C)
 use iso_c_binding
 use na_nodes
 implicit none
 integer(c_int),intent(in)::counts(7),irect(4,counts(2)),roles(counts(2))
 integer(c_int),intent(in)::neighbors(4,counts(2)),admsr(4,counts(2)),history(4,max(1,counts(3)))
 integer(c_int),intent(in)::normal_offsets(counts(4)+1),normal_mains(max(1,counts(5)))
 integer(c_int),intent(in)::removal_offsets(counts(3)+1),removed_mains(max(1,counts(6)))
 integer(c_int),intent(in)::optimized(max(1,counts(7)))
 real(c_double),intent(in)::main_stiffness(counts(2)),secondary_stiffness(max(1,counts(3)))
 integer(c_int),intent(out)::active(counts(2)),tags(counts(1)),free_count,free_ids(counts(2))
 integer::numnod,numels,nspmd,ninter25,ispmd,nthread
 common /NA_COUNTS/numnod,numels
 common /NA_PARTITION/nspmd,ninter25,ispmd
 common /NA_THREADS/nthread
 integer::dummy(1),iadfrnor(1,2),iadelem(2,1),ledge(15,1),e2s(max(1,counts(4)))
 type(nodal_arrays_)::nodes
 numnod=counts(1);numels=0;nspmd=1;ninter25=1;ispmd=0;nthread=1
 dummy=0;iadfrnor=0;iadelem=0;ledge=0;e2s=0
 active=0;tags=0;free_count=0;free_ids=0
 call I25FREE_BOUND(counts(2),neighbors,irect,main_stiffness,free_count,free_ids)
 ! Input is only the actual optimized suffix. TAGN's separate retained-row loop
 ! still receives the full staged history. Setting prefixcount0 only reindexes
 ! that suffix; no numerical field or local/global main identity is rewritten.
 call I25TAGN(1,1,counts(2),counts(3),0,1,iadfrnor,dummy,history,roles, &
     counts(7),0,optimized,secondary_stiffness,active,irect,tags,iadelem,dummy, &
     admsr,normal_offsets,normal_mains,2,removal_offsets,removed_mains,0,0,ledge, &
     free_count,free_ids,0,dummy,dummy,neighbors,e2s,counts(4),main_stiffness, &
     0,0,dummy,dummy,nodes)
end subroutine
