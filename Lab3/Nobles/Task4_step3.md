The turtle client node requests data from the turtle server node using the structure specified in the .srv file. The server node has some helper methods and can return the message specified in the .msg file.
```python
self.turtle = TurtleMsg()
```

The client node can get data about the turtle from the server node via TurtleMsg
```python
self.turtle_sub = self.create_subscription(TurtleMsg, 'turtleState', self.turtle_callback, 1)
```
After receiving data, the turtle client updates its color and position.
