module finalization_interfaces
  implicit none
  interface
    subroutine first_connections(nn,nsn,nodes,is1,counts,multi)
      integer,intent(in) :: nn,nsn,nodes(nsn),is1
      integer,intent(out) :: counts(nn),multi
    end subroutine
    subroutine kinini(ikine)
      integer :: ikine(*)
    end subroutine
    subroutine i2tid3(x,irect,st,msr,nsv,irtl,itab,ikine,ikine1,dmin,ipari,tzinf,iddlevel, &
        id,titr,intbuf_tab,dsearch,iproj,ixs,ixc,ixs10,ixs16,ixs20,stb, &
        nsn_multi_connec,t2_add_connec,t2_nb_connec,t2_connec,ixtg)
      use intbufdef_mod
      use element_mod
      use names_and_titles_mod
      integer :: irect(4,*),msr(*),nsv(*),irtl(*),itab(*),ikine(*),ikine1(*),ipari(*)
      integer :: iddlevel,id,iproj,nsn_multi_connec,t2_add_connec(*),t2_nb_connec(*),t2_connec(*)
      integer :: ixs(nixs,*),ixc(nixc,*),ixs10(6,*),ixs16(8,*),ixs20(12,*),ixtg(nixtg,*)
      double precision :: x(3,*),st(2,*),dmin(*),tzinf,dsearch,stb(2,*)
      character(nchartitle) :: titr
      type(intbuf_struct_) :: intbuf_tab
    end subroutine
  end interface
end module
