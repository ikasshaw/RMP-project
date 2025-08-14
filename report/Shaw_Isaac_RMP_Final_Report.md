# Robot Motion Planning - Final Project
## Prepared by: Isaac Shaw

This report details the three path planning methods implemented for the Robot Motion Planning course final project.

Three different path planning methods were implemented for this project. These were:
    - Approximate Cell Decomposition - 3D
    - A basic Ray Tracer
    - Rapidly Expanding Random Trees

Each method can be run with the corresponding run_<method>.m file or calling the functions directly. A number of options for each method are included to allow for experimentation. All methods include debug options to produce plots to visualize the methods operation.

To support development and testing of the methods, CreateRandomEnvironment.m was modified to produce outputs meeting the requirements of the project. The function, createEnvironment, outputs the vertices of a Robot, A, a cell array, B, with the vertices of k obstacles, and q_init and q_goal poses for the robot. A random seed can be passed so environments could be reproduced. The function also generates a boundary around the workspace. The project description did not require handling of boundaries so this was implemented for the methods as an optional argument. Omission of the bounds variable when calling any of the methods results in the bounds being computed as a rectangle enclosing all obstacles and the robot vertices at q_init and q_goal.

Each method outputs the planned path as as a 3xP array. This path can be plotted with the plotPath function when passed in with path, robot frame handle, robot vertices, and the cell array of obstacle vertices.

### Method I: Approximate Cell Decomposition - 3D

For method I, the approximate cell decomposition developed as part of the course homework was extended to work in three dimensional configuration space. This allowed theta to be varied along this third dimension.

FIGURE OF TYPICAL OUTPUT

#### Overview of Method
In this implementation, the configuration space was discretized into slices at intervals of theta. For each slice, the C-Obstacles for the given theta were calculated and stored. 

The split cell logic implemented as part of the homework was extended to divide the theta dimension on each split in addition to the X and Y dimensions. For each slice, the obstacle-cell intersection logic implemented in the homework was applied. Each split produces 8 children cells. A cell was labeled as mixed if any layer had an intersection, empty if empty for all slices, and full if all layers were full. In cases where a cell was split with an odd number of slices, the middle slice was included in all 8 children.

Additionally, an option to respect the workspace boundary was implemented. This allowed a 2xm matrix to be passed in to specify the boundary vertices. Similar logic to the cell obstacle intersection logic was implemented for the cell boundary intersection. The significant difference with the boundary was that an additional check was done on the centroid of the cell. If this was inside the boundary, the subsequent path would stay within the free space and was therefore accepted. if the cell centroid fell outside the bounds, this was considered mixed even when the cell contained no obstacles. 

The resulting adjacency graph was search with A* utalizing euclidean distance as the heuristic to find a viable path.

#### Observations
The core functionality, splitting and intersection checking, was easily adapted to 3D. In 2d, a naive algorithm for post adjacency checking was implemented and sufficient for scenarios with 5 obstacles within a few seconds. While this MATLAB implementation was not particularly performance focused, extending the algorithm over theta significantly increased the time complexity. When enforcing boundary checks, even simple configurations (three obstacles with theta between 0 and 2 Pi in .2 radian steps) only reached a split depth of 4 when running for 30 seconds. Ignoring boundary checks did a bit better coming in at 18-20 seconds depeneding on the exact workspace setup.

THe most challenging aspect of this methods implementing was managing the bookkeeping in through cell splits. Initially, the plan was to use either a brute force search or a somewhat less expensive cell face checking. These options were abondoned as the time to reach even trivial solutions grew too large. Debugging the adjacencies bookkeeping also proved difficult. A debugging view was implemented to display the adjacencies. Debug outputs for this view are included in the figure below. 

FIGURE OF ADJACENCY CHECK


### Method II: Rapidly Expanding Random Trees

An implementation of rapidly expanding random trees was chosen for method II. Paths were generated adhering to the kinematic constraints of a differential drive robot. Node connections used either Dubins (forward only) or optionally, Reeds-Shepp (forward and reverse) control algorithms to enforce the kinematic constraints. 

#### Overview


#### Observations


#### Challenges


#### Suggested Improvements



### Method III: Ray Tracer

For the final method, an attempt to generate a somewhat novel solution was made. Ray tracers are commonly implemented to simulate lighting effects in computer graphics applications. These algorithms emit spawned rays from a light source (or in some implementations, from the viewer source) and following the reflections until they either reach some max number of bounces or collide with a sensor. While no specific sources were used to generate the algorithm for this method, [x], [y], and [z] provide excellent overviews and implementations of ray tracers for computer graphics. 

#### Overview

The basic algorithm works as follows.
Compute fully inflated C-space obstacles
Compute a safe goal zone in C-space
Generate a set of initial ray angles and add to processing stack
Pop top ray from stack
Cast the ray and find the first collision point. 
From this point, generate n children rays
Rank children based on an heuristic
Add ranked 2-n children to processing stack
Cast best child ray.
If depth reaches max depth OR ray intersects goal
Pop next ray from stack

Several ray types could be spawned at each bounce. These were specular, diffuse, goal, and random. Specular reflections were reemitted across the surface normal. Diffuse reflections were emitted evenly over a hemisphere from the surface. Goal rays were emitted directly towards the goal from the hit point and random rays were emitted at a random angle from the surface. Rays were explored via depth first search where the best child ray was followed until the goal was reached or the max number of bounces were reached. This allowed for some optimization to stop rays if their best case distance was greater than the best so far distance. 

#### Observations
This algorithm was a moderate success. The implementation of the rays being emitted from the fully inflated C-space obstagles allows a pivot-drive-pivot path to be calculated which adheres to the constraints of a differential drive mobile robot.

Generally, the method was efficient when obstacles tended to be clustered together as the algorithm quickly cross large empty spaces without extraneous exploration. 


#### Challenges
The first implementation attempt recalculated the C-space obstacles based on the emitted ray angle. This caused several issues the most notable of which was invalid configurations when rotating to the newly emitted ray's angle. This required a "backoff" step to move back along the ray until the new angle could be safely turned to without a collision. This added significant processing overhead and was ineffective in even moderately cluttered workspaces. It also took away from the idea that this was a ray tracer as it no longer followed the core features of reflection ultimately acting as a poorly performing probabilistic roadmap method. 

The tradeoff with the inflated C-obstacles is that valid paths are excluded. 


#### Suggested Improvements
Calculate the intersection points of the successfull line segments and generate a graph between these connections. This might allow the goals to 

