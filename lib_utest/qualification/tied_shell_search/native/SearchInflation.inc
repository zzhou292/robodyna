        if(dsearch==zero) then
          do i=1,nrtm
            segment_data(i,2) = max(ZEP05*segment_data(i,1),ZEP6*segment_data(i,2))           
          enddo
        endif
