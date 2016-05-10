# leap_baxter : Control Baxter's Arm with a Leap


Built on top of ROS wrapper for the Leap Motion made by Juan Camilo Gamboa Higuera (https://github.com/juancamilog/leap_client).

Clone and install Juan's code before running this package ``git clone https://github.com/juancamilog/leap_client.git``

# To control the baxter 

1. Run the baxter.sh script to connect to Baxter
``cd ~/catkin_ws; ./baxter.sh``

2. Launch our launch file
``roslaunch leap_baxter leap_baxter.launch``

##Troubleshooting

If the lights on the leap won't turn on, or no data is being published, the leap daemon may need to restart. To do this, run
``sudo leapd``

If this does not work, run ``sudo service leapd restart``
