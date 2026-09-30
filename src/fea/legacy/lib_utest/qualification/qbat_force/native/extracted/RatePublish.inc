      asrate = one   ! to be changed for default value
      gbuf%epsd(1:nel) = asrate * epsd_glob(1:nel) + (one - asrate) * gbuf%epsd(1:nel)      
!-------------------------------------------------------------------------------
