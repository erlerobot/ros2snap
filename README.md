snappy packages out of ROS ones
===============================

Index
------
- [Create a trusty chroot for dev purposes](#Create-a-trusty-chroot-for-dev-purposes)
- [Building a snap for the whole ROS ](https://github.com/erlerobot/ros2snap/blob/master/README.md#building-a-snap-for-the-whole-ros-)


Create a trusty chroot for dev purposes
---------------------------

This repository is based on Martin Pitt's (@martinpitt) work (http://www.piware.de/2015/01/snappy-package-for-robot-operating-system-tutorial/).

###Creating a chroot with trusty

```
mkdir ~/trusty
qemu-debootstrap --arch=armhf trusty ~/trusty/
apt-get install qemu-user-static
sudo cp /usr/bin/qemu-arm-static ~/trusty/usr/bin/
sudo mount -o bind /dev ~/trusty/dev
sudo mount -o bind /proc ~/trusty/proc
sudo mount -o bind /sys ~/trusty/sys

Next comes the magic. This registers the ARM executable format with the QEMU static binary. Thus, the path to qemu-arm-static has to match where it is located on the host and slave systems:

```
```
echo ':arm:M::\x7fELF\x01\x01\x01\x00\x00\x00\x00\x00\x00\x00\x00\x00\x02\x00\x28\x00:\xff\xff\xff\xff\xff\xff\xff\x00\xff\xff\xff\xff\xff\xff\xff\xff\xfe\xff\xff\xff:/usr/bin/qemu-arm-static:' > /proc/sys/fs/binfmt_misc/register
```

```bash
sudo chroot ~/trusty
```

###Installing ROS

```bash
sudo update-locale LANG=C LANGUAGE=C LC_ALL=C LC_MESSAGES=POSIX
```

Configure `sources.list`
```
cat <<EOF > /etc/apt/sources.list
# See http://help.ubuntu.com/community/UpgradeNotes for how to upgrade to
# newer versions of the distribution.

deb http://ports.ubuntu.com/ubuntu-ports/ trusty main restricted
deb-src http://ports.ubuntu.com/ubuntu-ports/ trusty main restricted

## Major bug fix updates produced after the final release of the
## distribution.
deb http://ports.ubuntu.com/ubuntu-ports/ trusty-updates main restricted
deb-src http://ports.ubuntu.com/ubuntu-ports/ trusty-updates main restricted

## N.B. software from this repository is ENTIRELY UNSUPPORTED by the Ubuntu
## team. Also, please note that software in universe WILL NOT receive any
## review or updates from the Ubuntu security team.
deb http://ports.ubuntu.com/ubuntu-ports/ trusty universe
deb-src http://ports.ubuntu.com/ubuntu-ports/ trusty universe
deb http://ports.ubuntu.com/ubuntu-ports/ trusty-updates universe
deb-src http://ports.ubuntu.com/ubuntu-ports/ trusty-updates universe

## N.B. software from this repository may not have been tested as
## extensively as that contained in the main release, although it includes
## newer versions of some applications which may provide useful features.
## Also, please note that software in backports WILL NOT receive any review
## or updates from the Ubuntu security team.
# deb http://ports.ubuntu.com/ubuntu-ports/ trusty-backports main restricted universe
# deb-src http://ports.ubuntu.com/ubuntu-ports/ trusty-backports main restricted universe

deb http://ports.ubuntu.com/ubuntu-ports/ trusty-security main restricted
deb-src http://ports.ubuntu.com/ubuntu-ports/ trusty-security main restricted
deb http://ports.ubuntu.com/ubuntu-ports/ trusty-security universe
deb-src http://ports.ubuntu.com/ubuntu-ports/ trusty-security universe
# deb http://ports.ubuntu.com/ubuntu-ports/ trusty-security multiverse
# deb-src http://ports.ubuntu.com/ubuntu-ports/ trusty-security multiverse

deb http://packages.ros.org/ros/ubuntu trusty main
EOF
```

```bash
apt-get install -y wget
wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -
sudo apt-get update
```

Install ROS (826 MB):
```bash
sudo apt-get install -y ros-indigo-ros-base ros-indigo-mavros ros-indigo-mavros-extras ros-indigo-serial
```

###Getting snappy-tools (**not working*)

Add the sources
```bash
cat <<EOF >> /etc/apt/sources.list
deb http://ppa.launchpad.net/snappy-dev/tools/ubuntu vivid main 
deb-src http://ppa.launchpad.net/snappy-dev/tools/ubuntu vivid main
EOF

sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys F1831DDAFC42E99D
apt-get update
apt-get source -b snappy-tools
```

###Compiling a ROS package

```bash
apt-get install git
mkdir -p catkin_ws/src
cd catkin_ws/src
catkin_init_workspace
git clone https://github.com/erlerobot/ros_erle_takeoff_land
 
catkin_make_isolated --install # or "catkin_make install"

```



Building a snap for the whole ROS 
---------------------------

----

**Not working**

The snap generated at the end is missing some dependencies:
```bash
(ErleRobotics)ubuntu@localhost:/apps$ rossnap.ros-tutorial roscore
Traceback (most recent call last):
  File "/apps/ros-tutorial/0.3/bin/roscore", line 62, in <module>
    import roslaunch
  File "/apps/ros-tutorial/0.3/lib/python2.7/dist-packages/roslaunch/__init__.py", line 48, in <module>
    import rospkg
ImportError: No module named rospkg

```

----

###Building the .snap

The ``build.sh`` script creates an Ubuntu Snappy package for the ROS tutorial
and all of its ROS and Ubuntu dependencies. It needs to run in a minimal Ubuntu
15.04 (vivid) environment, preferably schroot or a container. 

```bash
./build.sh
```

This should create `/tmp/ros/ros-tutorial_0.3_<arch>.snap`.

This script currently contains a number of workarounds for snappy bugs (see
https://bugs.launchpad.net/snappy-ubuntu/+bugs)

This currently defaults to the "jade" ROS release. If you want to build indigo
instead, append "indigo" to the above build.sh command line.

###Installing the snap package


 - Start the snappy VM/instance. See
   http://www.ubuntu.com/cloud/tools/snappy#snappy-local for details.

 - Install the snapp on your snappy VM:

       snappy-remote --url=ssh://localhost:8022 install /tmp/ros/ros-tutorial_0.3_amd64.snap

   This assumes that you forward the snappy VM's ssh port to the local port
   8022, and that your package architecture is amd64 (on others the .snap
   file name will look accordingly).


###Running the snap package

At the moment you have to call the ROS tools through a `rossnap.ros-tutorial`
wrapper, which ensures that it can see the bundled Ubuntu packages. This
wrapper will go away soon when Snappy apps are run in a proper mount namespace.

ROS uses Qt4, and for running the `turtlesim` simulator you need to provide an
X display. If your host runs Ubuntu 14.10 or later, you can ssh into snappy
with forwarding the local X socket:

    ssh -R 6010:/tmp/.X11-unix/X0 -p 8022 ubuntu@localhost
    export DISPLAY=localhost:10.0   # in ssh

(Assuming that you forward the snappy VM's ssh port to local port 8022).

If you have Ubuntu 14.04 LTS or earlier, you instead need to use "ssh -X ..."
to log into the snappy VM and install xauth there:

    sudo mount -o remount,rw /
    sudo /usr/bin/apt-get update
    sudo /usr/bin/apt-get install -y xauth
    sudo mount -o remount,ro /

*Attention*: This *modifies* the Snappy Core image and thus you stop being able
to update it as a system image. However, this is only for the turtle demo;
usually a snappy package would just be a self-contained service.

Then log into snappy with three times, and run the three components:

    ros-tutorial.rossnap roscore
    ros-tutorial.rossnap rosrun turtlesim turtlesim_node  # this needs X forwarding
    ros-tutorial.rossnap rosrun turtlesim turtle_teleop_key

