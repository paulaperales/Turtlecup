# Turtlecup 
The idea of this project was to simulate a kind of RoboCup using Turtlebots2. However, it could not be possible to use the Navigation Stack for more than one Turtlebot. Therefore there is just one of them that looks for the ball and shoots, 2 goalkeepers and 2 that work as a VAR, whose only task is to detect whether there was goal or not.


![portada](https://user-images.githubusercontent.com/87413904/149670424-4d6e0bd6-6050-4ba8-974a-5fb892ac83ea.png)

# Instalation 
1. Install ROS Noetic from the [Official Webpage](http://wiki.ros.org/noetic/Installation/Ubuntu).
2. Install Turtlebot 2 simulator for Noetic:
   - Copy this [tb2.rosinstall](tb2.rosinstall) file in your _/home/user_ directory.
   - Open a command window and execute:
      ```
      sudo apt install libusb-dev libftdi-dev python-is-python3 pyqt5-dev-tools

      sudo apt install ros-noetic-openslam-gmapping ros-noetic-joy ros-noetic-base-local-planner ros-noetic-move-base

      mkdir $HOME/tb2_ws
      cd $HOME/tb2_ws

      wstool init src ../tb2.rosinstall

      rm -rf src/ar_track_alvar/ar_track_alvar

      catkin_make_isolated
      ```

  - If you want to have access to the packages created in that workspace every time a command window is opened, also execute:
      ```
      echo "source $HOME/tb2_ws/devel_isolated/setup.bash" >> ~/.bashrc
      ```
chmod

