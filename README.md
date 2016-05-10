leap_baxter
====

Built on top of the fabulous ROS wrapper for the Leap Motion API made by Juan Camilo Gamboa Higuera (https://github.com/juancamilog/leap_client).

# Juan's instructions for the wrapper

Needs ROS hydro or newer, and the Leap SDK. It has only been tested on Ubuntu 14.04. You can get the Leap SDK here: https://developer.leapmotion.com/downloads

Once you have downloaded the SDK bundle, you need to set the LEAPSDK environment variable to the path where the LeapSDK folder exists. For example, on my system I added the line

```bash
    export LEAPSDK=~/lib/LeapSDK
```
to my ~/.bashrc file, because I copied the LeapSDK folder into ~/lib. The LeapSDK folder should contain the following files: **include/Leap.h**, **include/LeapMath.h**, **lib/x64/libLeap.so** and **lib/x86/libLeap.so**.

To run, execute ``roslaunch leap_client leap_client.launch``

To visualize the data run "rosrun rviz rviz" and load the rviz config file "launch/leap_client.rviz"

# To control the baxter 

1. Run the baxter.sh script to connect to Baxter
``cd ~/catkin_ws; ./baxter.sh``

2. Launch the ros wrapper (captures poses from hand and pubishes to ROS topic 
``roslaunch leap_client leap_client.launch``

3. Run script to convert hand data to robot joint angles and send angles to robot
``rosrun leap_client hand_controller.py``

##Troubleshooting

If the lights on the leap won't turn on, or no data is being published, the leap daemon may need to restart. To do this, run
``sudo leapd``

If this does not work, run ``sudo service leapd restart``
