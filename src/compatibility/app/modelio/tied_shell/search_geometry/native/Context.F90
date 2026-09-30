! SPDX-License-Identifier: MIT
! Test-only leading dimensions and supplied native population counts.
! These sizes cover every field referenced by the complete donor, including
! its unselected layered-property branches; they are not solver defaults.
module search_geometry_context
  implicit none
  integer, parameter :: npropg=702, npropgi=98, npropm=20
  integer :: numnod=0, numelc=0, numeltg=0, iintthick=0
end module

module element_mod
  implicit none
  integer, parameter :: nixc=6, nixtg=5
end module

module search_geometry_packet_tools
  use iso_c_binding, only : c_int, c_double
  use, intrinsic :: ieee_arithmetic, only : ieee_is_finite
  implicit none
contains
  logical function valid_order(order, count)
    integer(c_int), intent(in) :: order(*), count
    logical :: seen(128)
    integer :: i, value
    valid_order=.false.
    seen=.false.
    do i=1,count
      value=order(i)
      if(value<1.or.value>count) return
      if(seen(value)) return
      seen(value)=.true.
    enddo
    valid_order=.true.
  end function

  ! Build supplied-order incidence only. INCOQ3 performs all matching/ranking.
  ! Each family's order is retained within every node's adjacency slice.
  subroutine build_incidence(nodes, order, offsets, elements)
    integer, intent(in) :: nodes(:,:), order(:)
    integer, intent(out) :: offsets(:), elements(:)
    integer :: counts(size(offsets)-1), cursor(size(offsets)-1)
    integer :: rank, element, slot, node
    counts=0
    do rank=1,size(order)
      element=order(rank)
      do slot=1,size(nodes,1)
        node=nodes(slot,element)
        counts(node)=counts(node)+1
      enddo
    enddo
    offsets(1)=0
    do node=1,size(counts)
      offsets(node+1)=offsets(node)+counts(node)
    enddo
    cursor=offsets(1:size(counts))
    do rank=1,size(order)
      element=order(rank)
      do slot=1,size(nodes,1)
        node=nodes(slot,element)
        cursor(node)=cursor(node)+1
        elements(cursor(node))=element
      enddo
    enddo
  end subroutine
end module
