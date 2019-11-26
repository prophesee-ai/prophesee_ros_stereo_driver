# Prophesee ROS Stereo Driver

![Event-based vision by Prophesee](event-based_vision_PROPHESEE.png)

This metapackage contains ROS stereo driver based on a wrapper around Prophesee Driver.
It offers the follosing ROS nodes
    * prophesee_ros_stereo_publisher - publishing data from Prophesee sensors to ROS topics
    * prophesee_ros_stereo_viewer - listening data from ROS topics and visualizing them on a screen

Supported Prophesee EVK:
  * VGA-CD: PSEE300EVK, PEK3SVCD
  

## Installation

  * First of all, you would need to install dependencies, such as Prophesee Driver SDK. Prophesee Driver SDK can be downloaded from our Knowledge Center. In case if you do not have an access to Knowledge Center yet, then please provide us a short decription of your research project and request an access via this webform https://www.prophesee.ai/contact-us/

    ```
        sudo apt install prophesee-*
    ```

  * Clone the source to your catkin workspace ( [create a workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace), if needed)

    ```
        cd catkin_ws/src
        git clone https://github.com/prophesee-ai/prophesee_ros_wrapper.git
        git clone https://github.com/prophesee-ai/prophesee_ros_stereo_driver.git
        cd ..
    ```

  * Compile (2 times to generate messages)

    ```
        catkin_make
    ```

  * Source the workspace

    ```
        source ~/catkin_ws/devel/setup.bash
    ```
  
  

## Getting Started
  
The package includes the following ROS nodes:
  * prophesee_ros_stereo_publisher
  * prophesee_ros_stereo_viewer

### Data publisher

To publish data from Prophesee stereo camera to ROS topics:

  ```
        roslaunch prophesee_ros_stereo_driver prophesee_stereo_publisher.launch
  ```

The following topics will be published:
  * /prophesee/camera/camera_info - info about the camera
  * /prophesee/camera/cd_events_buffer - buffer of CD (Change Detection) events
  * /prophesee/camera/imu - IMU data
 
 

### Data viewer

To visualize data from ROS topics:

  ```
        roslaunch prophesee_ros_stereo_driver prophesee_stereo_viewer.launch
  ```

## Contact
The code is open to contributions, so do not hesitate to ask questions, propose pull requests or create bug reports. In case of any issue, please add it here on github. 
For any other information contact us [here](https://www.prophesee.ai/contact-us/) 

