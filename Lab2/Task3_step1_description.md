1. Made the pen color black using
```ros2 service call /turtle1/set_pen turtlesim/srv/SetPen "'r': 0
'g': 0
'b': 0
'width': 10
'off': 0"
```
2. Cleared the turtle pane using ```ros2 service call /clear std_srvs/srv/Empty```
3. Used the absolute orientations and arrow keys to move forward and rotate 90 degrees four times.
