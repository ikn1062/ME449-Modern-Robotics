# Mobile Manipulation Capstone

## Submission Introduction:::

The detailed version of the software can be accessed through code.py or code.txt. The software contains a function called
mobile_manipulation with additional helper functions including, but not limited to, NextState, TrajectoryGeneration, 
and FeedbackControl. 

In regards to the solution method used to create the mobile_manipulation function, no other additional methods were used,
and the function was purely based off of the description of the code in the milestones provided in the Mobile Manipulation
Capstone Coursera course. Moreover, singularity avoidance and joint enhancements were NOT implemented in this version.


## Main function:::

mobile_manipulation(Tsc_i, Tsc_g, Tse_i, robot_config, Ki, Kp):
    Capstone Mobile Manipulator function
* param Tsc_i: the initial resting configuration of the cube object (np array)
* param Tsc_g: the desired final resting configuration of the cube object (np array)
* param Tse_i: the reference initial configuration of the youBot (np array)
* param robot_config: A vector containing 3 chassis configuration, 5 arm configuration, 4 wheel angles (np array)
* param Ki: PI Controller (np array)
* param Kp: Constant Control Gain (np array)
* return: Csv file which drives the youBot to successfully pick up the block and put it down at the desired location
             A data file containing the 6-vector end-effector error (csv file)


## Results:::

The results directory contains three difference scenarios, including "best", "overshoot", and "newTask", which all solve a
pick-and-place problem with the youBot. The "best" and "overshoot" results deal with the block at initial configuration 
(x,y,theta)=(1,0,0) and final configuration (x,y,theta)=(0,-1,-pi/2), with varying Ki and Kp values to achieve tuned 
controllers. The last "newTask" will have the initial and final configurations of the block at varying locations, proving
the feasibility of the software. The detailed version of the results can be found in the results directory.
