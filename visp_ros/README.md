visp_ros
========

A basket of generic ros nodes based on ViSP library.

# Installation

## Prerequisities

Install ros-<version>-visp package that matches your ros distribution (groovy, hydro, indigo), as for example:

	$ sudo apt-get install ros-hydro-visp

If you want to use the nodes that allow to control real robots such as Biclops PT head, Viper 650, Viper 850, Afma4 or Afma6 robots, you need to build ViSP from source and install ViSP in '/opt/ros/<ros-version>' in order to overwrite the version that was installed using the previous line. 

	$ cd soft
	$ svn checkout svn://scm.gforge.inria.fr/svn/visp/trunk/ViSP ViSP-code
	$ mkdir ViSP-build-ros; cd ViSP-build-ros
	$ cmake -DCMAKE_INSTALL_PREFIX=/opt/ros/hydro -DBUILD_SHARED_LIBS=ON ../ViSP-code
	$ make; sudo make install

## Get the source

	$ cd ~/catkin_ws/src

Get vision_visp stack that contains visp_bridge package:

	$ git clone https://github.com/lagadic/vision_visp.git (master branch)

Get visp_ros package:

	$ git clone https://github.com/lagadic/visp_ros.git (master branch)

## Build visp_ros package

	$ cd ~/catkin_ws
	$ catkin_make -DCMAKE_BUILD_TYPE=Release --pkg visp_ros

# Usage

	$ source ~/catkin_ws/devel/setup.bash
