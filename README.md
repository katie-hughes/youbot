# Milestone 2
Katie Hughes

My solution `code/milestone2.py` uses the modern robotics functions `ScrewTrajectory` and `CartesianTrajectory` to plan the desired trajectory of moving a block. The code breaks the trajectory into 8 segments: 
1. Move from the start position to the standoff position above the cube start location
2. Move straight down in the z direction in preparation to grasp the cube
3. Grasp the cube
4. Move straight up, returning to the standoff position
5. Move to the standoff position corresponding to the cube's final location
6. Move straight down in the z direction in preparation to release the cube
7. Release the cube
8. Move straight up, returning to the final standoff position

I set the time of the motion to be proportional to the cartesian distance the gripper needs to travel. This does not work very well for steps 2, 4, 7, and 8, where I am moving a very small distance in between the standoff and grasp, so for these steps I multiply the time by a scaling factor of 5. This factor can easily be changed in the future if I need to adjust the speed at which the gripper travels.

To run my solution, from the `Hughes_Katherine_milestone2` directory run: 
`$ python3 code/milestone2.py`
This will create an output file called `trajectory.csv` which will be located in the directory `Hughes_Katherine_milestone2`. This file can be loaded into `Scene8_gripper.csv` in CoppeliaSim to play my simulated trajectories.

Worked with: Nick Morales