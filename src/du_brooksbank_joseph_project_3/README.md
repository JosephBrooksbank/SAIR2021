# Project 3

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
Part 1:
https://youtu.be/WtvcA_01e28

**Part 2:**
https://youtu.be/A8SN8WBLvF8




 ## PART 1
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


## PART 2:
## Changes to structure
In part 1, I had 3 actions and 6 states, for very simple navigation along a wall. This was all done essentially in
one class, hard coded in. This was not sufficient for part 2. Currently, The robot has: 2 states for left, 5 states for 
front, 2 states for right-front corner, and 5 states for right-- Giving a total of 100 states for the robot overall. Additionally
the robot now has not only TURN_LEFT, TURN_RIGHT, and STRAIGHT, but a further discretization of turning for SLIGHT_LEFT
and SLIGHT_RIGHT. Both Actions and States are now handled in their own OOP style classes, in a way that I am quite happy with. 
This allows creation of more states and actions, as well as editing of current states and actions very easily. 

### Notes: Tuesday, March 15
This is my first submission of the project. The robot has not learned very well, especially on this first map. Over this
week I expect much more progress can be made on the new, simpler map. A lot of my issues came from the robot sensing 
the inner walls, I believe. Other changes I plan on making are tweaks to the linear and angular velocities of the actions,
and making the epsilon value scale with total runs.

Here is a video of progress so far: https://youtu.be/A8SN8WBLvF8


### Notes: Thursday, March 18
I was hoping to make substantial progress from Tuesday, given the new map and real time factor changes. Unfortunately, 
it doesn't seem like any modifications I had on the system really influenced it in noticeable ways. The robot does not so
much 'learn' as stumble forward, praying that it will randomly not crash. I am unsure why the learning works so poorly,
but I suspect something in my implementation is incorrect. As I say in the video: Thank you for the quarter, this was a 
very interesting class, and I realized that robotics may be something I want to pursue as a career at some point. Good
luck next quarter!

Video: https://youtu.be/LqRxellreq0

































