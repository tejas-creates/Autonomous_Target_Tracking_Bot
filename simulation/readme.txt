This folder contains several files.

simulation_world.sdf is responsible for the gazebo simulation world. It is obround shaped with several static and dynamic obstacles. All the obstacles are defined to be static in the simulation_world but are moved (essentially teleported) by another ros2 script, this method is chosen so that the obstacles do not react to any colliding force.

Target_AGV.sdf defined an AGV, which is being followed by another AGV. This target has ackerman drive with dimensions same as that of the following AGV.

launch->world.launch.py : this launch file is responsible to launch the simulation world and the Target_AGV and the AGV, in the world and this file can be launched from the bash file as well.

Garden world (garden_world.sdf)
-------------------------------
A realistic back-yard version of simulation_world.sdf: same world name, same 12 x 8 m operating
area, same 7 static and 20 moving obstacles (names, collision r 0.10 m / h 0.30 m, start positions).
Static obstacles look like pots, planters and a stump; moving ones like small garden robots.
A 0.55 m hedge just outside the area is the LiDAR boundary; trees, pergola, bench, lamp post,
bird bath, compost bins and the fence are all outside the hedge. The gravel path follows
docs/trajectory.csv and is flat (<= 3 mm, no collision, so the wheels sit on it, not in it).

garden/fetch_assets.py         downloads the CC0 assets into garden/assets/external (already included;
                               only needed to re-fetch): Poly Haven scanned pots + garden props + photo
                               textures, ambientCG grass/gravel, Gazebo Fuel oak/pine trees.
                               Sources: garden/assets/external/CREDITS.md
garden/build_garden_world.py   regenerates garden_world.sdf + garden/assets (procedural textures, OBJ
                               meshes) and fits the scanned pots to the 0.10 x 0.30 m obstacle size;
                               edit this script, not the .sdf.
ros_ws/src/random_obstacles     ROS 2 package (node random_obstacle_controller) that moves the 20 robots.
                               random: each robot drives a straight line segment; at its end it picks
                               the next one within +/-45 deg of its heading. If that is blocked (wall,
                               pot) or it is about to collide with another robot, it drives to any
                               random free point in the garden (away from the other robot).
                               crossing: each robot shuttles back and forth on a segment that crosses
                               the Target_AGV route, so the target drives through the traffic (it does
                               not collide with obstacles) and the AGV has to find a way.
                               Both stay inside the area and clear of the pots.

launch/world.launch.py arguments:
  world:=garden|plain          (default garden)

The moving obstacles are started by ros_ws/launch/simulation.launch.py, arguments:
  random_obstacles:=true|false (default true, starts the random_obstacles package)
  obstacle_motion:=crossing|random (default random, set in the launch file)
  obstacle_seed:=N             (default -1 = random every run)
