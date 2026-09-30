! SPDX-License-Identifier: AGPL-3.0-or-later
module LF_INTERFACES
  use iso_c_binding, only: c_double,c_int
  implicit none
  interface
    subroutine constant_failure_native(d1,damage,told,active,element,dpla,time,stress,values) &
        bind(C,name='constant_failure_native')
      import c_double,c_int
      real(c_double),value :: d1,damage,told,dpla,time
      integer(c_int),value :: active,element
      real(c_double),intent(in) :: stress(5)
      real(c_double),intent(out) :: values(3)
    end subroutine
    subroutine layered_failure_parent(flags,weights,parent) bind(C,name='layered_failure_parent')
      import c_double,c_int
      integer(c_int),intent(in) :: flags(3)
      real(c_double),intent(in) :: weights(3)
      real(c_double),intent(inout) :: parent
    end subroutine
    subroutine law44_point_section(pos,force,moment) bind(C,name='law44_point_section')
      import c_double
      real(c_double),intent(out) :: pos(3),force(3),moment(3)
    end subroutine
    function law44_point_filter(cutoff,dt) result(value) bind(C,name='law44_point_filter')
      import c_double
      real(c_double),value :: cutoff,dt
      real(c_double) :: value
    end function
    function law44_point_shell_rate(dx,thickness,dt) result(value) bind(C,name='law44_point_shell_rate')
      import c_double
      real(c_double),intent(in) :: dx(8)
      real(c_double),value :: thickness,dt
      real(c_double) :: value
    end function
  end interface
end module
