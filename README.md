# Racing Using SQN Reinforcement Learning

We are trying to combine reinforcement learning and trajectory optimization to win in F 1/10 racing. This repository is under heavy development and is not yet stable for any use-case.

Soft Q Network training of RL agent for high level decision making. https://github.com/christopher-hsu/f1tenth-spinningup

## Milestone 1 Proposal:

You can read our proposal here: [Proposal](docs/Proposal.pdf)


## Milestone 2 (Race one):

The only purpose is to complete going around the race track in the shortest time possible. There are not any other competitors in the race. We found the shortest path around the track (using b-spline) and are using a pure pursuit algorithm to track the path around the track. Using the old f110 simulator you can see the performance of our method around the track (the optimized path is pink):

<p align="center">
  <img src="first_race/videos/first_race.gif">
</p>


## Milestone 3 (Race two):

The main goal is to drive safely around the track by avoiding obstacles. The method we used to approach this problem is with a high-level Time-To-Collision (TTC) style calculation to decide on which path provides the most space. We follow the waypoint associated with the best TTC using Pure Pursuit:

<p align="center">
  <img src="second_race/videos/second_race.gif">
</p>




## Logistics

We have included both the new and old f110 simulators in this repository. Make sure that the new simulator pre requisites are properly installed (docker,etc.).

**To run our code:**

  * Clone this repositpory in src folder of your ROS workspace and catkin_make the workspace (make sure you dont have a duplicate new or old F110 simulator in your workspace since that might cause issues when making the workspace)

Then run the following depending on which simulator you are planning on using:

#### Race one: single car, no obstacles 

For the old simulator:

  *  Run the following in a new terminal: `roslaunch first_race race_old.launch`

For the new simulator:

  *  Go to `f1tenth_gym_ros` folder and run the following in a new terminal: `sudo ./docker.sh`
  *  Run the following in a new terminal: `roslaunch first_race race_new.launch`

#### Race two: single car, obstacles

For the old simulator:

  *  Run the following in a new terminal: `roslaunch second_race race_old.launch`

For the new simulator:

  *  Go to `f1tenth_gym_ros` folder and run the following in a new terminal: `sudo ./docker.sh`
  *  Run the following in a new terminal: `roslaunch second_race race_new.launch`

#### Progress updates

In addition to finishing the prerequisites for the first race, we have completed the following milestones preparing for the final race:

  *  We created 10 lanes that go around the track (used for the action space of our RL) and you can see them below:
  
  <p align="center">
  <img src="first_race/waypoints/Multi-Paths/paths.png">
</p>

  *  We then created more lanes in order to cover more of the track.
  
  <p align="center">
  <img src="second_race/waypoints/Multi-Paths2/paths.png">
</p>
 
  
  *  We Implemented the foundation/structure of SQN for training. This includes defining appropriate observation state and possible reward structure
  *  We are done with integrating pure pursuit with RL structure so that we can take RL output and act on it and control the car appropritely
  *  We are shifting to a local coordinate frame and including all sensor reading for the RL observation state.
  *  We will integrate the TTC path selector into the RL framework
  *  Reward shaping
  
  
