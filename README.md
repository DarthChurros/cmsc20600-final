[![Open in Visual Studio Code](https://classroom.github.com/assets/open-in-vscode-718a45dd9cf7e7f842a935f5ebbe5719a5e09af4491e668f4dbf3b35d5cca122.svg)](https://classroom.github.com/online_ide?assignment_repo_id=12356467&assignment_repo_type=AssignmentRepo)
# A* Maze Navigation

**Names:** Maxwell Krolak, Tejas Narayan, Justin Shin

## Project Description

Our project aimed to extend our particle filter localization to enable a robot
to not only identify its position but also navigate to arbitrary other points in
the provided maze. With a combination of informed pathfinding and localization,
our goal was to have the robot move to a provided location.

We used two different approaches to navigate through the maze, which we will
refer to as the **gradient** method and the
**parametric** method.

### Particle filter optimization

Because navigating the maze required live
positional updates from the robot, our particle filter needed to be both fast
and precise in order to provide motion commands in real time. We built on the
particle filter project (which was already heavily optimized) in order to make
this project work.

- Further vectorization, particularly in measurement_model(), through numpy
- Distinguishing "deep scans" for initialization from "quick scans" to be
used in real time
- Increased map resolution to account for very small variations in position
- Optimized particle cloud initialization by precomputing a valid
coordinate set rather than randomizing and then filtering for validity
- Motion model now accounts for backward movement (more of a bug fix, but
still)

### Pathfinding

Once the robot has determined its position in the maze, we use the map to
determine the shortest safe route to its destination. To do this, we implemented
Dijkstra's algorithm on the maze's coordinate grid to find the shortest path
from any location to the destination. By excluding points close to the walls of
the maze, we could somewhat account for the size of the robot and eliminate
paths likely to cause collisions.

Though A* would somewhat optimize this process by only searching part of the
maze, we computed all optimal paths for three reasons: ease of implementation,
built-in error handling, and to remove the need for recomputation.

This is where the solutions we applied start to diverge. In the gradient method,
we used the entire shortest paths tree to determine the best new node to go to
from *any point in the graph*. This solution didn't decide on a path to the
destination before the robot started moving; it just determined the next best
action one step at a time given the state of the robot and the maze layout.

In our parametric movement model, we relied on a precomputed path derived from the
Dijkstra tree. We originally intended to compute this path with A*, but the
performance drop from computing the full tree was negligible, so we chose to
reuse that logic--despite our project goal, we never actually implemented A*
(though the path we found wouldn't have changed if we had).

### Path modification

After computing a path through the maze, we applied some transformations on it
before applying to to the robot's motion in order to simplify movement planning
and execution. Because the LiDAR only gives us five scans per second, the rate
at which we can tell the robot to update its velocity is limited to 5Hz. This
means that the less frequently we need to issue movement updates, the more
reliably the robot will follow the provided path.

#### Gradient

Because there was no precomputed path, all path modifications took place in real
time. From the robot's current position as determined by the particle filter,
we charted out the optimal path until it turned. Approximating that section of
the path as a line segment, we were able to skip many of the intermediate nodes
and target the later ones with negligible impact on the route.

#### Parametric

This solution uses a series of points to chart out a path from the initial to
the final position, then parameterizes the robot's movement along that sequence.
Originally, we handled this as a piecewise function composed of linear segments,
which we approximated from the A* path using Douglas-Peucker. Later, we fit
B-splines to the path in order to eliminate the jerky movement that comes from
advancing, turning, and then advancing again. Though our demo only used the
latter, our final submission uses both Douglas-Peucker reduction and
curve-fitting, which we found to give more reliable results.

### Movement

Once the robot was in the maze, we had to continuously issue commands based on
its last known position and our knowledge of the layout. Despite our best
efforts, the robot would occasionally stray too close to a wall, meaning that
our movement strategies had to be designed around collision avoidance and
recovery.

#### Gradient

#### Parametric
