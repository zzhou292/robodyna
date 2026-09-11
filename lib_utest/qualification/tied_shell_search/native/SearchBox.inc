            nodes_id(1:4) = irect(1:4,i) ! get the node ids of the segment
            x_nodes(1:3,1:4) = x(1:3,nodes_id(1:4)) ! get the coordinates of the nodes
            x_min(1:3) = huge(x_min(1:3))
            x_max(1:3) = -huge(x_max(1:3))
            do j=1,4 ! loop over the nodes of the segment to find the minimum and maximum coordinates
              x_min(1:3) = min(x_min(1:3),x_nodes(1:3,j))
              x_max(1:3) = max(x_max(1:3),x_nodes(1:3,j))
            enddo
            ! add the influence zone
            x_min(1:3) = x_min(1:3) - segment_data(i,2)
            x_max(1:3) = x_max(1:3) + segment_data(i,2)
