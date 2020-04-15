This is the package for milestone2

## Usage

for race:
```
roslaunch team5 race.launch
```

### launch 
- get_waypoint.launch (get waypoint using wall follow)
- race.launch (for race using pure pursuit)
 

### log
- wp.csv (mid waypoint prepared for single path pure pursuit)

### scripts
- pure_pursuit.py 
- visualize_points.py
- wall_follow.py
- waypoint_logger.py

### waypoints
- dealcsv.py (manually adjust the waypoint get from wall follow)
- wp-2020-04-14-23-45-53.csv (points at inner path, get wp-inner.csv)
- wp-2020-04-15-06-31-57.csv (points at inner path, get wp-mid.csv)
- Multi-Paths
  - points.txt (points set from rviz)
  - spline.py
  - multiwpi.csv 

<img src="/waypoints/Multi-Paths/paths.png" />

using scipy.interpolate.splprep to generate multiple paths, in our cases, we generate 10 for the use of RL.

 
