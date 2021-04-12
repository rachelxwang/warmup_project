# Warmup Project

### Robot Behaviors
* **Driving in a Square**

  High Level Description: In order to direct the robot to drive in a square, I directed it to move forward for 5 seconds and make a 90 degree turn four times. I used the Twist message, specifically the linear x to move forward and the angular z to make turns.

  Code Explanation: I created a `DriveSquare` class, which has the following functions:
    * `turn` -  directs robot to turn 90 degrees by publishing a Twist message with z angular velocity of 0.5, waiting approximately pi seconds (since angular velocity is 1/2 and we want to travel pi/2), and then publishing the Twist message with the z angular velocity set back to 0

    * `go` - directs robot to drive forward for 5 seconds by publishing a Twist message with x linear velocity of 0.2, waiting 5 seconds, then publishing the Twist with the x linear velocity set back to 0

    * `run` - repeats the turn and go movements four times

  ![driving in square](./drive_square.gif)

* **Person Follower**

  High Level Description: In order to direct the robot to follow the person (or object), I used the LaserScan message data from the LiDAR to determine the current state of the robot and person and proportional control to determine how the robot should move. For the linear movement, I directed it to move at a constant velocity until it was less than 0.4m away from the person. For the angular movement, I used the current angle from the robot to the object (e.g. 0 degrees is directly in front of the robot, 90 degrees is directly to the left of the robot, etc.) as the process variable and 0 degrees as the desired set-point to determine the necessary angular velocity to turn the robot towards the person.

  Code Explanation: I created a `PersonFollower` class, which has the function `run` to run the node and the function `process_scan`, which:
    * if there is no object detected, publishes the Twist with 0 angular and linear velocity (so the robot does nothing).

    * if an object is detected, (1) retrieves the current angle by getting the min of the ranges and using the list index of the min as the angle, (2) sets the linear velocity to 0.3 by default or 0 if the object is within 0.4m of the robot, (3) determines the error signal as the difference between current angle and desired angle of 0 degrees, (4) sets the angular velocity using the error signal and a k_p of 0.01, and (5) publishes the Twist.

  ![follower person](./person_follower.gif)

* **Wall Follower**

  High Level Description: In order to direct the robot to follow the wall, I used the LaserScan message data from the LiDAR to determine the current positions of the walls to the front and left of the robot. I used proportional control to set angular velocity so that if the robot strays of the path that is 0.5m away from the left wall it is following, then it will turn towards the path to try to get back on the path. I used the difference between the max range and the distance from the front of the robot to the wall to make the turns.

  Code Explanation: I created a `WallFollower` class with a `run` function to run the node and a `process_scan` function, which:
    * retrieves the distance to any object directly in front of or to the left of the robot from the LaserScan,

    * uses proportional control to control the angular velocity to try to stay on the path 0.5m from the left wall if there is no wall directly in front of the robot,

    * uses the distance from the front wall to determine angular velocity to make a right turn (this is done so the angle of the turn is more or less than 90 degrees based on what angle the robot approaches the wall) if there is a wall directly in front of the robot,

    * and publishes a constant linear velocity and the calculated angular velocity to the cmd_vel topic.

  ![wall follower](./wall_follower.gif)

### Challenges
One challenge that I faced was encountering and overcoming unexpected robot behaviors in the gazebo, particularly in the wall follower problem. The issue I ran into the most was that the robot would not see a wall directly in front of it and would proceed to drive through the wall. The disconnect between what I was seeing in the gazebo and what the robot was seeing was a confusing and frustrating challenge, but I overcame it by realizing that shutting down and restarting the gazebo would often fix the issue. Another challenge I faced was the noise in the environment and robot movements. This sometimes made it hard to tell whether unexpected movements (e.g. turn angle is significantly less than or greater than 90 degrees, turtlebot drifts, etc.) were due to a mistake in my implementation or due to the noise. I overcame this by running and testing each of my behaviors many times to figure it which of the bad behaviors were from the implementation and which were due to noise. Another way I worked around this was by trying to reduce the noise by decreasing all my speeds so there would be fewer issues with acceleration and drifting.
### Future Work
If I had more time, I would improve my wall follower behavior. Sometimes the angle of the turn is much less than or much greater than 90 degrees, even when the robot approaches at the same angles as other turns which did make an accurate 90 degree turn. This resulted in a rounded path along the walls (either rounding inwards to the center or outwards towards the wall), due to my implementation of trying to correct back to the path. Increasing the k_p for the path correction only resulted in extremely wobbly behavior or curvy paths and over-correction/overshooting the path. One way I'd like to improve it would be to add a function (to replace my current path correction implementation) which can make a turn of any given degree angle. If the closest object to the robot is not immediately to its front, back, left, or right, then it would stop, make the necessary turn so that it is perpendicular to the front/back walls and parallel to the left/right walls, and then continue moving forwards. A possible issue with this idea, however, is that turns like these are often messed up by noise and not the exact angle I would want, and so it would potentially create other unexpected behavior issues.
### Takeaways
* **Gazebo Issues**: Restarting the gazebo and relaunching the world is often a good troubleshooting method. I found that I could not rely on what I was saw in the gazebo matching what the robot was seeing (e.g. it would drive through what I would expect to be solid walls, it would stop moving as if hitting a wall even when there did not appear to me to be a wall), especially when manually moving the robot around or resetting the world. I spent a lot of time debugging my code, when the actual issue was coming from the gazebo, so stopping and re-running the roslaunch commands would help me and others save time in the future.

* **Just Run It**: I approached each behavior by meticulously planning beforehand the logic of my implementation and how exactly the ideas from class would work for my behaviors. For example, before any implementation, I tried to calculate and identify good values for the proportional control constants (k_p and the goal variable). Although planning before writing code is certainly important, something I found very helpful was actually to just write some code and run it. In other classes or other types of programming, it is often very difficult to think through the initial logic and debug at the same time. However, I think the visual cues from being able to actually see the robot moving and executing the written commands helped me to understand the logic much better. In other words, my takeaway is that sometimes just running something and seeing the resulting behavior is the most helpful in understanding the logic for why something is happening or how it works.
