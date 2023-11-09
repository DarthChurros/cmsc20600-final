# cmsc20600-final

[![Open in Visual Studio Code](https://classroom.github.com/assets/open-in-vscode-718a45dd9cf7e7f842a935f5ebbe5719a5e09af4491e668f4dbf3b35d5cca122.svg)](https://classroom.github.com/online_ide?assignment_repo_id=12356467&assignment_repo_type=AssignmentRepo)
# particle_filter_project

## Implementation Plan

Names: Justin Shin and Maxwell Krolak

### How you will initialize your particle cloud (initialize_particle_cloud)?
The state of each possible particle can be specified as a triple, (x, y, θ), where x, y specify the position and θ specifies the orientation of the robot respectively. Given some desired number of particles "n", the code will sample n particles using a uniformly random distribution over all possible orientations (angles 0-360º) and spatial positions (any occupiable point in the map). 

Testing: We will plot the particle cloud on ArViz (as vector arrows to capture both
position and orientation) and see if the points are roughly uniformly distributed
across the map.

### How you will update the position of the particles will be updated based on the movements of the robot (update_particles_with_motion_model)?
Each possible robot move can be specified as a triple, (∆x, ∆y, ∆θ), where ∆x, ∆y specify the translation and ∆θ specifies the rotation of the robot respectively. The move will be added to each particle state, along with movement noise, to update the particle.

More precisely, a particle (x, y, θ) will be updated to (x', y', θ') by move (∆x, ∆y, ∆θ) as follows:
(x', y', θ') = (x, y, θ) + (∆x, ∆y, ∆θ) + (∆x * ε(), ∆y * ε(), ∆θ * ε())
where ε: {} -> ℝ is a random noise function (e.g. a normal distribution); 
(∆x * ε(), ∆y * ε(), ∆θ * ε()) is the noise term

Testing: For each particle in the cloud, we will plot in ArViz the particle's
old position/orientation, the move vector (adjusted with noise), and the particle's 
new position/orientation to see if they make sense. The new position should be at 
the end point of the move vector.

### How you will compute the importance weights of each particle after receiving the robot's laser scan data?(update_particle_weights_with_measurement_model)?
We will be using the "likelihood fields for range finders" measurement model, 
which requires a precomputed minimum obstacle distance table T. To compute
the weights, we will implement verbatin the likelihood_field_range_finder_model 
algo shown in meeting 6 slides.

Testing: We will plot the precomputed obstacle distance table T as a heat map 
in python, overlaid on an image of the map for comparison. We should expect
table entries near obstacles to be increasingly red, while those further away are blue.

### How you will normalize the particles' importance weights (normalize_particles) and resample the particles (resample_particles)?
Normalizing: To normalize the particle weights computed by the likelihood fields
model, we will divide each weight by the weight total.
w_sum := Σ w_t[i], for each i in [1,... n] w'_t[i] = w_t[i] / w_sum
The normalized w_t'[i] 's sum to 1.

Resampling: Each particle x_t[i] has probability w_t'[i] of being chosen to fill 
any one of the 'n' vacancies in the next population of points, x_t+1. We will
invoke np.random.choice with size 'n' to resample 'n' particles according to
weights w's.

Testing: We will check that w_t'[i] 's sum to 1. We will draw a large sample from a population of points according to some arbitrary weights, and bar graph the relative frequencies on matplotlib to ensure that they match the expected distribution.

### How you will update the estimated pose of the robot (update_estimated_robot_pose)?
At time step t of the Monte Carlo Localization algorithm, we will collect x_t:
the collection of robot state guesses/particles at time step t. Our estimate 
of the robot pose will be the average of these particle states.

Testing: Plot a particle cloud in ArViz and invoke update_estimated_robot_pose
on the particles. Verify the resulting estimate is central to the mass of the cloud.

### How you will incorporate noise into your particle filter localization?

Noise will be incorporated into our particle filter localization in the form
of movement noise. 

Recall from above that ––whenever we update a particle (x, y, θ) to (x', y', θ') 
by move (∆x, ∆y, ∆θ)–– we add the noise term (∆x * ε(), ∆y * ε(), ∆θ * ε()):
(x', y', θ') = (x, y, θ) + (∆x, ∆y, ∆θ) + (∆x * ε(), ∆y * ε(), ∆θ * ε())

Testing: We will create a particle cloud comprised of copies of the 
same particle. We will then apply the same move to each particle, adjusted with
noise. If we plot–– in ArViz–– the original cluster of particles, the various noise-adjusted move
vectors, and the new cluster of particles, we should see a new cluster with
roughly similar orientations and positions.

### A brief timeline sketching out when you would like to have accomplished each of the components listed above.

We will aim to complete one component per day, starting on Oct 13. This includes
the additional component of building the map during lab. Early completion
of each component leaves time for more extensive testing.


## Writeup

***
LIDAR variant: RPLIDAR-A1.
***

### Gif
![project.gif](https://github.com/Intro-Robotics-UChicago-Fall-2023/particle-filter-project-mkrolak-shinej/blob/main/project.gif?raw=true)

### Objectives
In this project, we must program an ROS node that implements the Monte Carlo 
particle filter algorithm for a Turtlebot3 robot, allowing the robot 
to estimate its current position in pre-configured maze environment. 

As the bot moves throughout the maze, the particle cloud generated by the 
code must converge to the bot's true location and orientation. In preliminary 
stages of the project, we must measure both the map and the closest-distance 
table of the maze.


### High level description
The bot's particle cloud constitutes the bot's educated "guesses" about its 
current position and orientation. The particle cloud is initialized to be
be uniformly randomly distributed across the maze, wrt both position and orientation,
reflecting its initial absolute uncertainty.

Updates to the particle cloud are performed passively in response to 
detected changes in bot position, with position checks being performed every received lidar scan.
In any scan callback cycle, if the bot senses that it has moved beyond a threshold 
displacement (linear or angular), it resamples its particle cloud in accordance 
with new likelihood probabilities computed from the latest scan and odometry data.
This resampling reflects the bot's guesses (hopefully) improving by reasoning 
on new data.

#### Initialization of particle cloud
Implemented in initialize_particle_cloud(self) <pub method>

In this function we generate a given number of points. For each point, we repeatedly generate its coordinates until they are 1. within our map and 2. do not occupy an obstacle. Then we generate the point's yaw. These values are assigned to the corresponding particle index in internal pose and yaw arrays, the array format enabling the application of efficient numpy operations later on. However as we still need an array of Points for publishing to Rvis we made the function update_particle_cloud to propagate the latest values in the internal state arrays to a preallocated array of points. This function is called at the end of init particle cloud so that we have the array initialized upon the code starting.

#### Movement model
Implemented in update_particles_with_motion_model(self) <pub method>

To implement the movement model, we update each particle by the latest movement of the robot (adjusted with per-particle specific noise to account for random noise in the robot's sensors and motors). We do this by first taking the rotational movement of the robot and adding that to the direction of the point. Then we take the translational movement relative to the robot's point of view and rotate it to match the reference frame of each point. Then we add this translation to the point. We finish by updating the yaw and pose arrays to the corresponding calculated values.

#### Measurement model
Implemented in update_particle_weights_with_measurement_model(self, data) <pub method>

The goal of the measurement model is to compute the weight of each particle. We do this by computing the distance to the closest obstacle and computing the weight as the product of some function on this distance.

Due to the number of computations, many parts have been optimized (to the loss of readability). The code first reads the latest lidar ranges data and extracts a pre-determined subset of angles. For each particle, we use numpy to translate and rotate the lidar ranges onto the reference frame of the given point. The transformed ranges are represented, element-wise, with x-coordinates array norm_xs and y-coordinates array norm_ys. If a transformed range is within bounds of the closestMap, we compute its distance to the nearest obstacle by reading from the closestMap table. If the range is out of bounds, its distance is set to a large distance penalty. Using numpy, we then compute the product of the probability of each distance–– setting it as the new weight of the particle.

#### Resampling
Implemented in resample_methods(self) <pub method>

resample_particles invokes np.random.choice to sample particle indices (functionally equivalent to particles) in accordance with the latest particle weights. The Counter object is used to count how many of each particle index was sampled.

The resample variants of each internal state array (e.g. poses_resample) are filled with the appropriate number of particle copies.
If a particle at index i was sampled n times, then we would fill an n-long stretch of poses_resample with the particle's pose [x_i, y_i] and yaws_resample with the particle's yaw yaw_i respectively, which is functionally equivalent to having n copies of particle i.

Now we swap the self.X_resample and self.X array references so that the current internal states correctly reflect the newly sampled particles.

#### Incorporation of Noise
Noise is implemented in *latter parts of* update_particles_with_motion_model(self) <pub method>

Noise is introduced into the system via movement noise in the motion model. During each motion_model update, three "random tape" arrays are computed using a gaussian distribution–– reflecting a unique random noise value for each particle's x, y, and yaw. Each particle is then adjusted by its corresponding x, y, and yaw noise.

#### Update robot pose.
Implemented in update_estimated_robot_pose(self) <pub method>

To determine the robot position we used a weighted average on the points x and y to determine the location of the robot and used a weighted average on the quaternion vector to determine the orientation. This was implemented very simply in the update_estimated_robot_pose function.

#### Optimization of parameters
The primary metrics we optimized for during parameters tweaks were convergence accuracy (tightness of cloud cluster) and speed.

- z_random, z_max, prserve_factor: Initially, with too many lidar scans, the final likelihood probability product would often zero out. To address this we set our z_max and z_random values such that their ratio was .1 and added a "preserve_factor" constant to boost each component in the product. These measures allowed a more reasaonable final product. 
- sigma_hit: we set standard deviation sigma_hit as 30 cm, which seemed to be a reasonable range of deviations for accurate scans. 
- number of particles and scan angles: through trial-and-error, we first determined the minimum viable number of each value (viable as in it enables rapid cloud convergence). As the performance of our code improved, we could steadily increase the number particles and scans.
- noise: incorporating 15º stdev of noise for rotation and 5cm stdev of translation allowed the cloud to quickly converge while leaving sufficient leeway to track the robot.


### Challenges
#### Coordinate transformations
Initially, the convergence behavior of our particle cloud resampling was poor
because we had failed to properly transform the lidar ranges vectors onto
each particle position/orientation while computing their likehoold field probabilities.

We carefully reasoned through the required coordinate transforms by paper–– 
applying a rotation matrix then getting the required x,y components–– so that
we could reimplement them correctly in code.


#### Performance issues
Initially, due to performance issues, our code was forced to make a trade-off 
between the number of particles and the number of angles (number of angles as 
in how many angles are considered in the likelihood field probability of each particle). 
We could run 1k particles with 10 angles, or 100 particles with 100 angles.

Now, with some optimizations in place, we no longer need to compromise.
Our current configuration supports 10k particles with 360 angles each. 

The key areas of optimization:
1. Pre-allocated memory: 
Previously, resampling would allocate a new buffer of particles every scan cycle.

Now, we pre-allocate everything: 1. the array of particles, and 2. three
pairs of arrays to represent the internal states of each particle.
Resampling simply transfers particle states from one buffer in each pair to the other.
Each scan cycle the roles of these buffers swap, with one holding new (resample) 
particle values, the other holding old particle values.

2. Using numpy arrays
Previously, to compute any kind of value aggregate of particles (e.g.
likelihood probability), we had to iterate over each particle in a for-loop,
extracting each particle's state and incorporating it into the accumulated value.

Now, by exposing the internal states of particles (x,y, and yaw) as numpy arrays,
we can use vectorized numpy operations to compute these aggregate values.
Numpy array operations are fast because of contiguous memory allocation,
which improves locality and avoids the pointer-chasing that typically plagues
field accesses in Python.

The particle cloud is updated to the latest values in the internal state arrays
only when necessary–– just before publishing.


#### Float error
Due to the large number of particles involved, float error would be a problem
while normalizing particle weights. If the weights did not sum to 1, np.random.choice
invoked on those weights would fail.

We wrote code that detects a weight sum float error and distributes it across
the particle weights to compensate for it.

### Future Work

#### SLAM/Roomba algo
We optimized the particle filter significantly and were sufficently satisfied with our results. However if we had more time and were tasked with improving I think it would be cool to implement a roomba algorithm where it learns the layout of a given room and then computes the map and finally locates itself in the unknown space simular to the slam algorithm. If this system was volentarly adopted by many Americans it could be used to curate advertisments based on observations of peoples houses.

#### Futher performance improvements
update_particle_weights_with_measurement_model takes around 0.4s each scan cycle
to execute–– over 2 orders of magnitude slower than the next slowest component––
which is far from ideal.

The use of multi-threading in *_measurement_model to divide the array of particles can
speed this up significantly. One complication to this approach is Python's GIL
(global interpreter lock), which prevents concurrent accesses to the fields of 
an object. To bypass this issue, the internal particles state arrays must be declared
as shared arrays.

#### Kidnapping resilience
Kidnapping, or the relocation of the bot by an unaccounted external force,
is a major vulnerability of the particle filter algorithm. If a bot is moved
when its particle cloud has already converged to a tight cluster, the cloud
may fail to converge to the new location.

A possible fix is to, during each scan cycle, reserve a small portion of the particle cloud 
to be distributed randomly across the maze at resample. This ensures that,
even if the robot is "kidnapped", at least a few points will have a high match
with the new location, enabling fast reconvergence of the cloud.

### Takeaways 
#### Numpy
In optimization we learned alot about the efficency of numpy as it allowed us to run our code, in some cases, 100x faster. This will help in other projects as we will be able to use more data with the same runtime and create more accurate and results.

#### The value in statistical models
By the accuracy of the monte carlo model we have learned about the power of statistical models for performing complex tasks in robotics. By demonstrating the ability of monte carlo for localization we have learned that predictive tasks can be automated through a simple process and done accurately.

#### Volume of data matters
When we use only a few lidar angles, the particle cloud converges quite slowly. With 360 angle measurements, the convergence is almost instantaneous. The more data the bot collects from/considers about the environment, the more discrepancies that can be detected in order to rapidly weed out bad guesses.
