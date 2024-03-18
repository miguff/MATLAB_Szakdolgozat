load ("occupancy_map50.mat")
Map = occupancyMap(omap);

start_x = 5;
start_y = 8;
start_orientation = 0;
goal_x = 17;
goal_y = 17;
goal_orientation = 0;


robot_pose = [start_x start_y start_orientation];
goal_pose = [goal_x goal_y goal_orientation];

