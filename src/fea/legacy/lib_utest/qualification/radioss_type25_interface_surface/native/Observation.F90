module interface_native_observation
  implicit none
  integer :: last_matched_solid=0
  type interface_surface
    integer :: nseg=0
    integer :: nodes(256,4)=0,eltyp(256)=0,elem(256)=0
  end type
end module
subroutine rd_interface_reset()
  use interface_native_observation
  implicit none
  last_matched_solid=0
end subroutine
subroutine rd_interface_match(n)
  use interface_native_observation
  implicit none
  integer,intent(in)::n
  last_matched_solid=n
end subroutine
integer function bitset(value,position)
  implicit none
  integer,intent(in)::value,position
  error stop 'Unselected IMBIN branch in interface oracle'
  bitset=value
end function
