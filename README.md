## Object Following Car with ROS and Gazebo

**This project simulates a robot car following a moving object using ROS (Robot Operating System) and Gazebo.** The car utilizes an ultrasonic sensor mounted on a servo-like link to detect the nearest object and maintain a following behaviour.

**Features:**

*   Implements ultrasonic sensor for object detection.
*   Controls car movement based on sensor readings.
*   Simulates object following behavior in Gazebo.

**Usage:**

**Dependencies:**

*   ROS (Recommended: ROS Noetic)
*   Gazebo (Compatible with ROS Noetic)

**Steps:**

1.  Clone this repository into your ROS workspace (e.g., `~/catkin_ws/src`):

    ```bash
    cd ~/catkin_ws/src
    git clone https://github.com/Michael-Medhat/Object-follower.git
    ```

2.  Build the package and Source the environment variables:

    ```bash
    cd ~/catkin_ws
    catkin_make
    source devel/setup.bash
    ```

3. Launch the object follower gazebo simulation:
    ```bash
    roslaunch my_robot_control robot.launch carspeed:="2"
     ```
