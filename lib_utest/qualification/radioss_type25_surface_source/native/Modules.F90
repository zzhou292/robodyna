! SPDX-License-Identifier: AGPL-3.0-or-later
! Qualification-only storage/interface scaffolding. No surface decisions here.
module my_alloc_mod
  implicit none
  interface my_alloc
    module procedure alloc1, alloc2
  end interface
  interface my_dealloc
    module procedure dealloc1, dealloc2
  end interface
contains
  subroutine alloc1(a,n,name)
    integer,allocatable,intent(inout)::a(:)
    integer,intent(in)::n
    character(*),intent(in)::name
    if(allocated(a).or.n<0) error stop 'Invalid native oracle allocation'
    allocate(a(n))
  end subroutine
  subroutine alloc2(a,n,m,name)
    integer,allocatable,intent(inout)::a(:,:)
    integer,intent(in)::n,m
    character(*),intent(in)::name
    if(allocated(a).or.n<0.or.m<0) error stop 'Invalid native oracle allocation'
    allocate(a(n,m))
  end subroutine
  subroutine dealloc1(a)
    integer,allocatable,intent(inout)::a(:)
    deallocate(a)
  end subroutine
  subroutine dealloc2(a)
    integer,allocatable,intent(inout)::a(:,:)
    deallocate(a)
  end subroutine
end module
module setdef_mod
  implicit none
  type set_
    integer::nb_part=0,nb_solid=0,nb_sh4n=0,nb_sh3n=0,nb_quad=0
    integer::nb_sphcel=0,nb_tria=0,nb_truss=0,nb_beam=0,nb_spring=0
    integer::nb_surf_seg=0,ext_all=0
    integer,allocatable::part(:),solid(:),sh4n(:),sh3n(:),quad(:)
    integer,allocatable::sphcel(:),tria(:),truss(:),beam(:),spring(:)
    integer,allocatable::surf_nodes(:,:),surf_eltyp(:),surf_elem(:)
  end type
end module
module inverted_group_mod
  implicit none
  type invertgroup_struct_
    integer,allocatable::indparts(:),parts(:),indpartc(:),partc(:),indparttg(:),parttg(:)
    integer,allocatable::indpartq(:),partq(:),indparttria(:),parttria(:),indpartt(:),partt(:)
    integer,allocatable::indpartp(:),partp(:),indpartr(:),partr(:),indpartsph(:),partsph(:)
  end type
end module
module set_scratch_mod
  implicit none
  type set_scratch
    integer::sz_surf=0,sz_line=0
    integer,allocatable::surf(:,:)
  end type
end module
module message_mod
  implicit none
end module
module names_and_titles_mod
  implicit none
  integer,parameter::ncharfield=80
end module
module surf_mod
  implicit none
  integer,parameter::ext_surf=1,all_surf=3
end module
module element_mod
  implicit none
  integer,parameter::nixs=11,nixq=7,nixc=7,nixtg=6
end module
module rd_surface_observation
  implicit none
  integer,parameter::rd_face_cap=512
  integer::rd_raw_face(rd_face_cap),rd_raw_solid(rd_face_cap),rd_sort_index(rd_face_cap)
  integer::rd_sorted_count=0
contains
  subroutine rd_reset()
    rd_raw_face=0
    rd_raw_solid=0
    rd_sort_index=0
    rd_sorted_count=0
  end subroutine
  subroutine rd_face(address,face,parent)
    integer,intent(in)::address,face,parent
    integer row
    if(address<1.or.mod(address-1,6)/=0) error stop 'Native face buffer observation alignment'
    row=(address-1)/6+1
    if(row>rd_face_cap.or.face<1.or.face>6.or.parent<1.or.parent>64) &
      error stop 'Native face observation cap'
    rd_raw_face(row)=face
    rd_raw_solid(row)=parent
  end subroutine
  subroutine rd_sorted(count,index)
    integer,intent(in)::count,index(*)
    if(count<0.or.count>rd_face_cap) error stop 'Native surface sorting observation cap'
    rd_sorted_count=count
    rd_sort_index(1:count)=index(1:count)
  end subroutine
end module
! Unselected original calls remain linked to explicit fatal diagnostics.
subroutine shell_surface_buffer_remesh()
  error stop 'Remeshing is outside surface oracle profile'
end subroutine
subroutine quad_surface_buffer()
  error stop 'QUAD family is outside surface oracle profile'
end subroutine
