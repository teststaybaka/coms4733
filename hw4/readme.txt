Path planning:
Environment: python2.7 with PIL(http://www.pythonware.com/products/pil/).
Execute: python route-finding.py
Output file: pic.bmp, route.res.
The picture shows the resulting path as well as the obstacle map. Be aware that this map is mirrored.
The .res file saves the actual coordinate of each point in the path.

If you want to change the file path for different start and goal points and obstacles map, please change line 135 and line 144.

Map Convering:
map_converter.m: Help change the format of obstacles map to be used in the simulator.
Output file: obs_map.txt

Actual running:
hw4_team_19.m: Actual controlling code for the robot.