! SPDX-License-Identifier: AGPL-3.0-or-later
! Complete selected callers retain these branches; this profile never enters them.
subroutine B18F_M2LAWPI()
  error stop 'Unselected beam LAW2'
end subroutine
subroutine B18F_FAIL_BEAM18()
  error stop 'Unselected beam failure model'
end subroutine
subroutine B18F_SIGEPS34PI()
  error stop 'Unselected beam LAW34'
end subroutine
subroutine B18F_SIGEPS36PI()
  error stop 'Unselected beam LAW36'
end subroutine
subroutine B18F_SIGEPS71PI()
  error stop 'Unselected beam LAW71'
end subroutine
subroutine B18F_SIGEPS131PI()
  error stop 'Unselected beam LAW131'
end subroutine
