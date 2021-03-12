# Project 3 Part 1

## Installation:
***
1. Move this entire folder (du_brooksbank_joseph_project_3) into your `catkin/src` folder. This is often something like 
`/home/YOURUSER/catkin_ws/src`.
2. Move to the root of your catkin_ws and run `catkin_make`
3. Source either `setup.bash` or `setup.zsh`, depending on which shell you use 

## To Run:
***
1. run `roscore` in any terminal
2. In a terminal sourced as above, run `roslaunch du_brooksbank_joseph_project_3 wallfollow.launch`

## Example Video:
https://youtu.be/WtvcA_01e28


## Discretization
I chose to create essentially 3 ACTIONS and 6 STATES for my q_table. The actions being: FORWARD, TURN_LEFT, and TURN_RIGHT.
The turning actions are not strictly turning but turning and moving forward slightly, so the robot can 'see' the effects of the turn
rather than spinning in place. For the states, I decided on having 2 buckets for the readings from the FRONT of the robot:
CLOSE and FAR. The front readings should only really come into play when approaching a corner, so the resolution isn't
as important. For the RIGHT side of the robot, I decided on CLOSE, GOOD, and FAR. The robot should attempt to maintain a 
'GOOD' distance from the wall it is following, so a higher resolution of observations is required. with 2 observations 
from the front and 3 from the side, 6 states are created. As no learning is occurring, I simply assigned an ACTION to 
each state with a large reward, while all other actions are 0. 
### Notes:
I intentionally set the robot down at a slight angle towards the wall, to show that it has self-correcting ability.
I wasn't sure if simply setting the robot down and having it go in a straight line counted as enough.
I intentionally set a `z` value when moving the robot, because sometimes if I didn't specify that the wheels fly off
when placing the robot. I think the ground isn't perfectly flat, and they would be moved into the floor. 
I believe I may either need to adjust the turning functions for part 2, or add actions to handle faster turns.
I am using python's `Enum` class here, which I discovered is fairly useless. I plan on creating a class for Actions and
their functionality for part 2, as well as a class for the Q_table.