# Capstone
Katie Hughes

My solution `code/capstone.py` combines milestones 1-3.

To run my solution, from the `Hughes_Katherine_capstone` directory run: 
`$ python3 code/capstone.py`
This will create an output file called `configs.csv` which will be located in the directory `Hughes_Katherine_capstone`. This file can be loaded into `Scene8_gripper.csv` in CoppeliaSim to play my simulated trajectory. It will also create `errors.csv` which is the vector $X_{err}$ at each timestep. The created image `errors.png` plots these errors over the course of the simulation.

All of my code and functions are contained in `code/capstone.py`. At the very beginning of the script, you can modify 3 booleans to determine whether the "best", "overshoot", or "newblock" scenario runs. The default is "best". The directories `results/best`, `results/overshoot`, and `results/newTask` contain the outputs of each of these three scenarios. The `log.txt` file is what the terminal output is for these 3 cases. Additionally, in this very first section, you can change the $K_i,K_p$ gains, the starting config, and the block locations if you wish. 

## Functions
* `NextStep`: Given a config 13-vector and controls to follow, computes the next config 13-vector using an Euler step.
* `TrajectoryGenerator`: Generates the desired trajectory in the form of a list of 8-vectors that are meant for scene 8 (used in Milestone 2)
* `TrajectoryGenerator2`: A direct copy of `TrajectoryGenerator`, except instead of returning a list of 8-vectors specifying the configuration for scene 8, it returns a list of transformation matrices that the end effector should follow.
* `FeedbackControl`: Applies feedforward control plus PI control and returns a commanded end effector twist

## Main Loop
First, I generate a desired trajectory using my function TrajectoryGenerator2. Then, I loop through each matrix in this function. This gives me $X_d$ and $X_{d\_next}$. I also find the actual end effector position $X$ via converting the configuration to a transformation matrix (this is automated by a function I wrote, `get_X`). With these three transformation matrices, I can call `FeedbackControl` to get a new commanded end effector twist $V$. I then calculate the Jacobian at the current configuration, and am able to get a list of wheel and joint controls to drive the end effector to the desired configuration. Finally, I calculate the config at the next timestep using these controls through `NextState`, and repeat the loop. I save the configs and errors at each timestep to view in CoppeliaSim/plot. 

## Behavior

Overall, I am fairly happy with the way that my simulation behaves. I can change $K_p$ and $K_i$ and see different behavior in how the robot adjusts to the path. All of my simulations result in the robot picking up and placing the blocks at essentially exactly the desired positions, as the error converges to 0. One thing I was unable to get a good example of was overshoot. Increasing $K_p$ should increase the overshoot, but I was unable to get a smooth overshoot in my results directory -- perhaps this could be solved by choosing a different timestep. 

I also tried to implement collision avoidance, but ran into strange behavior in my simulation that was significantly worse than not including it. First, I tried adding in the joint limits as specified in the YouBot datasheet (http://www.youbot-store.com/wiki/index.php/YouBot_Detailed_Specifications). This made it so that the arm was unable to reach down to grab the brick, but it followed the rest of the motion reasonably. I then tried constraining joints 3 and 4 to less than -0.2 and this also did not work -- in fact, with this constraint, the arm passed through itself on the way to picking up the first block. The code for this is present in my submission, but I was unable to find the appropriate limits, so they are currently listed as all "None". 