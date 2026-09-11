      cell_size = four * dd
      cell_nb(1:3) = int(distance(1:3)/cell_size)
      cell_nb(1:3) = max(cell_nb(1:3),1)
