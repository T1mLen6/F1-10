# Lab 1: Intro to ROS 2

## Written Questions

### Q1: During this assignment, you've probably ran these two following commands at some point: ```source /opt/ros/foxy/setup.bash``` and ```source install/local_setup.bash```. Functionally what is the difference between the two?

Answer: (The first one is to source the ROS foxy packages and tools, and the second one is to source my own workspace build)

### Q2: What does the ```queue_size``` argument control when creating a subscriber or a publisher? How does different ```queue_size``` affect how messages are handled?

Answer: (The queue_size controls the size of message that can be buffered before deleted. The queue_size can store some messages (as a cache) if the generation speed is faster than sending speed, or the receiving speed is faster than the processing speed.)

### Q3: Do you have to call ```colcon build``` again after you've changed a launch file in your package? (Hint: consider two cases: calling ```ros2 launch``` in the directory where the launch file is, and calling it when the launch file is installed with the package.)

Answer: (calling ```ros2 launch``` in the directory where the launch file is DOES NOT need to rebuild the package, however, calling it when the launch file is installed with the package DOES need to build to make the changes take effect)
