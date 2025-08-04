## Method 1
Approximate Cell Decomposition

Implement in 3D to consider all orientations of the robot

<!-- Use A* To plan the path -->
Compare A*, Djikstra, and maybe D* paths

Recompute the planned path to only allow rotations at goal points then move in a straight line to the next goal point. May be countable for the compliance with holonomic constraints



## Method 2
Rapidly Exploring Random Trees

Implement in 3D to consider all orientations of the robot if possible

Compare A*, Djikstra, and maybe D* paths


## Method 3
Dynamic Window Approach/Bug approach depending on time constraints



## Other Ideas

### Raytracing method(s)

- Send out x number of rays to see if they make it to the goal
  - Could send out many rays and let them bounce "realistically" assuming everything is perfectly reflective OR let each bounce spawn some number of other rays where some or all have some variation baked in (protect against corner reflectors)
  - Needs an algorithm to smooth the final path without hitting any obstacles (avoid the actual bounce)
  - Could decompose configuration space such that the goal lives in a region then check if the ray tracer can get to the region. Follow a simple line from the edge of the region to the goal
- Send out an initial ray to the goal and score it based on the distance the ray is inside of obstacles.
  - Send out x number of candidate arrays in random directions and let them bounce y number of times then stop and check from the end of each ray to goal how long the ray is inside obstacles. Choose the shortest. Repeat
- Could do this in 3d by the bounce sets the layer your are then shooting the ray through
- Set the boundary around the goal to be the biggest circle possible that doesn't hit any obstacles
- Have a tunable buffer that is an offset you move back from the ray collision point. Can cast a ray between this buffer point and the same buffer distance on the outgoing ray to provide additional "safety"
- Once you reach the goal zone, call the plot adjacency to get the rest of the way

- If a ray makes it to the final region, save it. Continue for some set number of additional random rays. Create graph of interesections of all successful arrays and do a graph search over this array




