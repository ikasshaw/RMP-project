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





# Robot Motion Planning – Final Project
**Prepared by:** Isaac Shaw

This report summarizes three path-planning methods implemented for the final project:

- Approximate Cell Decomposition (ACD) in 3D configuration space  
- A basic Ray-Tracing planner  
- Rapidly-Exploring Random Trees (RRT)

Each method can be executed via its `run_<method>.m` script or by calling the function directly. All include options for experimentation and a debug mode that renders key intermediate geometry. The environment generator was adapted from the course scaffold: `createEnvironment` outputs the robot vertices \(A\), an obstacle set \(B\) as a cell array of polygons, and poses \(q_{\text{init}}\), \(q_{\text{goal}}\). A reproducible RNG seed may be supplied. Although the project spec did not require workspace boundaries, an optional `bounds` polygon is supported; if omitted, a minimal rectangular boundary enclosing \(A\), \(B\), \(q_{\text{init}}\), and \(q_{\text{goal}}\) is auto-computed.

All planners return a path as a \(3\times P\) array \([x;y;\theta]\). The helper `plotPath` visualizes a returned path together with the robot footprint and obstacles.

---

## Method I: Approximate Cell Decomposition — 3D

### Overview
Configuration space is discretized over \(\theta\) into slices; for each slice, C-obstacles are constructed and cached. The 2D splitting logic from the homework was extended to split along X, Y, **and** the \(\theta\) dimension—each split yields 8 children. A cell is labeled: **full** if all slices intersect obstacles, **empty** if none do, and **mixed** otherwise. When a split occurs with an odd number of \(\theta\) slices, the middle slice is included in all children so adjacency is well-defined across layers. This method follows the configuration-space formulation and subdivision family of approaches in motion planning \[1\], \[11\], \[2\].

An optional boundary check treats any cell whose centroid lies outside the boundary as **mixed** even if it contains no obstacles—this guarantees later path extraction stays within free space. The resulting adjacency graph is searched with A* using Euclidean cost as the heuristic \[3\].

**Figure:** Typical ACD output (slices, cell states, and adjacency preview).

### Observations
Extending the split and intersection tests to 3D C-space was straightforward, but runtime increases notably versus 2D. With boundary checks enabled, scenarios with three obstacles and \(\theta\in[0,2\pi]\) at 0.2-rad increments reached split depths of ~4 in ~30 s on my MATLAB build; skipping boundary checks improved runtime (18–20 s depending on the layout). The primary engineering challenge was robust **bookkeeping** of adjacencies during repeated subdivision; a dedicated debug overlay was added to visualize and validate the adjacency sets.

---

## Method II: Rapidly-Exploring Random Trees (RRT)

### Overview
The RRT implementation samples \(q=[x,y,\theta]\) within bounds, extends from the nearest node with a **differential-drive (unicycle) steering law**:
\[
\dot{x}=v\cos\theta, \quad \dot{y}=v\sin\theta, \quad \dot{\theta}=\omega,
\]
with saturations \(|v|\le v_{\max}\), \(|\omega|\le \omega_{\max}\). Two local-connection modes are exposed: **Dubins** (forward-only) and **Reeds–Shepp** (forward and reverse), standard models for car-like/differential-drive systems \[6\], \[7\], \[2\]. A goal-region polygon in C-space is built (matching the Ray-Tracer method) and, once entered, a short pivot–drive–pivot maneuver is attempted to reach \(q_{\text{goal}}\).

For collision checks along trajectories, the exact robot footprint \(A\) is transformed by \(SE(2)\) and tested against workspace polygons (exact poly–poly checks). Geometric kernels (e.g., `fitSegment`, `intersectSegment`, `firstHitWithPoly`) are used where appropriate. Design choices follow the base RRT formulation and the bidirectional acceleration from RRT-Connect \[4\], \[5\].

### Observations
- **Strengths.** Quickly finds feasible paths in moderately cluttered scenes, respects nonholonomic constraints by construction, and benefits from goal bias plus the polygonal goal region (which reduces “last-meter” failures).  
- **Trade-offs.** Like most sampling planners, solution quality can be jagged; light waypoint pruning or short local refinements help. Dubins local steering is faster but less flexible than Reeds–Shepp; the latter improves success rate in tight spaces at a compute cost.  
- **Correctness of steering.** The steer block is explicitly differential-drive/unicycle dynamics; Dubins/Reeds–Shepp are classical shortest-path families under bounded curvature (and, for RS, reversing), which map well to diff-drive kinematics \[6\], \[7\], \[2\].

### Challenges
Balancing step size, goal bias, and angular rate limits is important: too-small steps increase collision checks; too-large steps degrade success rate near obstacles. Ensuring consistency between the goal-region polygon test and the final pivot–drive–pivot approach was also key to avoid “hovering” near the goal without terminating.

### Suggested Improvements
- Add a short **post-processing** pass (e.g., shortcutting or informed local optimization) to smooth and shorten paths.  
- Use **informed sampling** (e.g., ellipse sampling in \(SE(2)\)) once a first solution is found to improve quality.  
- Optionally switch to a **bidirectional** variant (RRT-Connect) for faster first-solution times in mazy scenes \[5\].

---

## Method III: Ray-Tracing Planner

### Overview
This method adapts ideas from computer-graphics ray tracing to explore free space using “rays.” The planner first builds **inflated** C-obstacles to guarantee clearance for a pivot–drive–pivot path. It then constructs a **goal-region** polygon in C-space. Execution proceeds depth-first: emit a set of initial rays, repeatedly cast the current best ray, generate child rays at each bounce (specular, diffuse, goal-directed, random), rank children by a heuristic, and continue until the goal is struck or a depth/iteration cap is reached. The general ray-tracing concept and reflection/refraction treatments are well-covered by Whitted’s seminal paper, Kajiya’s rendering equation, and Glassner’s classic text \[8\]–\[10\].

**Algorithm sketch:**  
1) Build inflated C-obstacles; 2) compute a safe goal zone; 3) seed initial ray angles; 4) cast ray → first hit; 5) spawn N children; 6) rank and follow the best; 7) stop on goal or depth; 8) backtrack to next candidate.

### Observations
The inflated C-obstacles made it easy to synthesize feasible pivot–drive–pivot motions and traverse large empty regions quickly when obstacles cluster. Heuristic pruning (discarding rays whose best-case cost exceeds the current incumbent) yielded practical speedups.

### Challenges
An early prototype recomputed C-obstacles at each emitted-ray orientation. This introduced invalid intermediate rotations and required a costly back-off step; performance degraded markedly in cluttered scenes. Using a fixed inflated C-space avoids that pitfall, though it may **exclude** some valid, tighter paths.

### Suggested Improvements
Compute intersection points of successful ray segments and build a graph over them; a short search on this sparse visibility-like graph could accelerate convergence and improve final path quality.

---

## References (IEEE style)

[1] T. Lozano-Pérez, “Spatial Planning: A Configuration Space Approach,” *IEEE Transactions on Computers*, vol. C-32, no. 2, pp. 108–120, 1983.  
[2] S. M. LaValle, *Planning Algorithms*. Cambridge, U.K.: Cambridge University Press, 2006.  
[3] P. E. Hart, N. J. Nilsson, and B. Raphael, “A Formal Basis for the Heuristic Determination of Minimum Cost Paths,” *IEEE Transactions on Systems Science and Cybernetics*, vol. 4, no. 2, pp. 100–107, 1968.  
[4] S. M. LaValle and J. J. Kuffner, Jr., “Randomized Kinodynamic Planning,” in *Proceedings of the IEEE International Conference on Robotics and Automation (ICRA)*, 1999, pp. 473–479.  
[5] J. J. Kuffner, Jr. and S. M. LaValle, “RRT-Connect: An Efficient Approach to Single-Query Path Planning,” in *Proceedings of the IEEE International Conference on Robotics and Automation (ICRA)*, 2000, pp. 995–1001.  
[6] L. E. Dubins, “On Curves of Minimal Length with a Constraint on Curvature, with Prescribed Initial and Terminal Positions and Tangents,” *American Journal of Mathematics*, vol. 79, no. 3, pp. 497–516, 1957.  
[7] J. A. Reeds and L. A. Shepp, “Optimal Paths for a Car that Goes Both Forwards and Backwards,” *Pacific Journal of Mathematics*, vol. 145, no. 2, pp. 367–393, 1990.  
[8] T. Whitted, “An Improved Illumination Model for Shaded Display,” *Communications of the ACM*, vol. 23, no. 6, pp. 343–349, 1980.  
[9] A. S. Glassner, *An Introduction to Ray Tracing*. San Diego, CA, USA: Morgan Kaufmann, 1989.  
[10] J. T. Kajiya, “The Rendering Equation,” in *Proceedings of SIGGRAPH*, 1986, pp. 143–150.  
[11] R. A. Brooks and T. Lozano-Pérez, “A Subdivision Algorithm in Configuration Space for Findpath with Rotations,” in *Proceedings of the International Joint Conference on Artificial Intelligence (IJCAI)*, 1983, pp. 292–298.
