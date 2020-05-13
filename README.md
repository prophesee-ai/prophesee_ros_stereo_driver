# Prophesee ROS Stereo Driver

![Event-based vision by Prophesee](event-based_vision_PROPHESEE.png)

This metapackage contains ROS stereo driver based on a wrapper around Metavision SDK Driver module.
It offers the follosing ROS nodes:
  * prophesee_ros_stereo_publisher - publishing data from Prophesee sensors to ROS topics
  * prophesee_ros_stereo_viewer - listening data from ROS topics and visualizing them on a screen

Supported Prophesee EVK:
  * stereo event-based only
  

## Installation

First of all, you would need to install dependencies, such as Metavision SDK:

  * Request an access to Knowledge Center, if not done yet. To get an access, fill the [webform](https://www.prophesee.ai/contact-us/) and provide us a short description of your research project.

  * Sign up for a trial version of [Metavision SDK](https://support.prophesee.ai/portal/en/kb/articles/sdk-trial-request-form), if not done yet.

  * Install Metavision SDK following [the instructions on Knowledge Center](https://support.prophesee.ai/portal/en/kb/articles/linux-software).


Then, compile GitHub code:

  * Clone the source to your catkin workspace ( [create a workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace), if needed)

    ```
        cd catkin_ws/src
        git clone https://github.com/prophesee-ai/prophesee_ros_wrapper.git
        git clone https://github.com/prophesee-ai/prophesee_ros_stereo_driver.git
        cd ..
    ```

  * Compile

    ```
        catkin_make
    ```

  * Source the workspace

    ```
        source ~/catkin_ws/devel/setup.bash
    ```
  
  

## Getting Started
  
The package contains the following ROS nodes:
  * prophesee_ros_stereo_publisher
  * prophesee_ros_stereo_viewer

### Data publisher

To publish data from Prophesee stereo camera to ROS topics:

  * Update serial numbers of your stereo cameras in prophesee_stereo_publisher.launch file (if not done yet)
    * Find the serial numbers of your cameras
      * either on the camera itself (only the last digits are given, therefore, complete them with zeros up to 8 digits)
      * or via metavision_platform_info tool
    * Update the serial numbers in prophesee_stereo_publisher.launch file using any editor

  * launch the Stereo Publisher

  ```
        roslaunch prophesee_ros_stereo_driver prophesee_stereo_publisher.launch
  ```

The following topics will be published:
  * /prophesee/camera_left/camera_info - info about the left camera
  * /prophesee/camera_left/cd_events_buffer - buffer of CD (Change Detection) events from the left camera
  * /prophesee/camera_right/camera_info - info about the right camera
  * /prophesee/camera_right/cd_events_buffer - buffer of CD (Change Detection) events from the right camera


### Data viewer

To visualize data from ROS topics:

  ```
        roslaunch prophesee_ros_stereo_driver prophesee_stereo_viewer.launch
  ```

## Contact
The code is open to contributions, so do not hesitate to ask questions, propose pull requests or create bug reports. In case of any issue, please add it here on github. 
For any other information contact us [here](https://www.prophesee.ai/contact-us/) 

