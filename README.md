# cmsc20600-final

## Proposal

### Team Name
Fairly Unremarkable

### Members
- Justin Shin
- Max Krolak
- Tejas Narayan

### Repo
https://github.com/DarthChurros/cmsc20600-final

### Motivation
We heard that groups in past years had difficulty doing this project and so we wanted to undertake this project as a sort of self-imposed challenge.

Converting back-and-forth between the discrete domain of the a-star algorithm and the continuous domain of the real-world differential control is a highly non-trivial problem. Furthermore, pathfinding and path following in general is an important component of autonomous robotics.

### Main Components
We hope to apply a pathfinding algorithm such as A* in a physical context, using particle filter localization in order to assign the robot a starting point and coordinate its actual movements with its planned path. We anticipate that a major obstacle to overcome will be the translation of the algorithm output into the physical environment, since the pathfinding will be run on an imperfect map and the robot is likely to deviate from a simple series of commands.

### Final Product

#### a minimum viable product (MVP)
The program, when given an arbitrary destination in the maze, computes the shortest-distance path from the current location of the robot to the destination, and controls the robot to follow the path in a node-hopping fashion.
By "node-hopping" we refer to the simple path-following mechanism of driving to the next node in the path and stopping, then driving etc.

#### a solid end result
Implement a collision-avoidance module.
When the program detects that the bot's current trajectory is headed into a wall, the collision-avoidance module will compute a new route that returns the bot to the intended path.

#### an end result that you would consider to be a "stretch" (e.g., "it would be above and beyond if we could...")
Implement a solution to the traveling salesman problem.
The program, when given an arbitrary set of locations in the maze, computes the optimal (or near-optimal) path that passes through the locations, and drives the robot to follow this path.

### Timeline
We will aim to complete one component per day, starting on Nov 9. The components in question are: 
- A* algorithm on the pixel map to compute a path
- Simplifying this to a piecewise linear path to send to the robot
- Executing the path with appropriate course correction based on particle filter localization
- [Collision-avoidance module]
- Integration of above components
Early completion of each component leaves time for more extensive testing.

### Resources
- 1 turtlebot with NO arm (either lidar type works, but would prefer to have both types available)
- Use of maze

### Budget
N/A

### Risks
We anticipate that there will be significant unforeseen hurdles based on past groups’ challenges with A*. A major issue we are aware of is the question of computational load while integrating the A* solution with particle filter localization.