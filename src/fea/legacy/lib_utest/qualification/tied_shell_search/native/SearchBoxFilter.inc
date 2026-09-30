                      x_node(1:3) = x(1:3,node_id)
                      if(x_node(1)<x_min(1).or.x_node(1)>x_max(1)) cycle
                      if(x_node(2)<x_min(2).or.x_node(2)>x_max(2)) cycle
                      if(x_node(3)<x_min(3).or.x_node(3)>x_max(3)) cycle
