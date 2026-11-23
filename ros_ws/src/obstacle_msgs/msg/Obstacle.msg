# One obstacle, described as a circle in the odometry frame.

int32 id          # tracking id (-1 for wall pieces, which are not tracked)
float64 x         # centre [m]
float64 y         # centre [m]
float64 vx        # velocity [m/s], 0 for static obstacles
float64 vy        # velocity [m/s], 0 for static obstacles
float64 radius    # circle radius, safety margin already included [m]
bool is_moving    # true if the obstacle is moving
bool is_wall      # true if this circle is one piece of a long wall/flat surface
int32 hits        # number of scans this obstacle has been seen in
