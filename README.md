Joy to Twist

Project Description This project controls a robot using a gamepad. It turns controller inputs into speed commands for the robot.

Controls Left Stick: Linear velocity (Forward / Backward) Right Stick: Angular velocity (Rotate Left / Right)

Publishers and Subscribers

Node: joy_to_twist_node Subscribes to /joy (sensor_msgs/msg/Joy) Publishes to /robot_velocity (geometry_msgs/msg/TwistStamped) Publishes to /input_pada (geometry_msgs/msg/TwistStamped)

Node: joy_to_twist_sub Subscribes to /robot_velocity (geometry_msgs/msg/TwistStamped) Subscribes to /input_pada (geometry_msgs/msg/TwistStamped) It prints out the velocity and input data.

Node: joy_to_twist_tui Subscribes to /input_pada (geometry_msgs/msg/TwistStamped) It visualizes the position of the sticks in the terminal.