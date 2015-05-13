#!/bin/sh
# Create snappy package for the ROS tutorial. Defaults to the "jade" release,
# but you may specify "indigo" or another release as first command line
# argument.
#
# (C) 2014 - 2015 Canonical Ltd.
# Author: Martin Pitt <martin.pitt@ubuntu.com>

set -e

WORKSPACE=/tmp/ros
ROSRELEASE="${1:-jade}"

if [ -e "$WORKSPACE" ]; then
    echo "$WORKSPACE already exists, please clean up first" >&2
    exit 1
fi

# install ROS/Catkin packages necessary for building ROS
sudo apt-get install -y wget build-essential
host_ubuntu_release=$(awk '/http:.*\/ubuntu/ {print $3; exit}' /etc/apt/sources.list)
echo "deb http://packages.ros.org/ros/ubuntu $host_ubuntu_release main" | sudo tee /etc/apt/sources.list.d/ros-latest.list
wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -
sudo apt-get update --no-list-cleanup -o Dir::Etc::sourcelist=/etc/apt/sources.list.d/ros-latest.list -o Dir::Etc::sourceparts=/dev/null
NEEDED_PKGS_DEPS="$(apt-get install -s python-rosdep python-rosinstall|grep ^Inst | awk '{print $2}')"
sudo apt-get install -y python-rosdep python-rosinstall-generator python-wstool python-rosinstall
sudo rosdep init

# download ROS packages
mkdir -p "$WORKSPACE"
cd "$WORKSPACE"

rosdep update
rosinstall_generator rosbash ros_tutorials --rosdistro $ROSRELEASE --deps --wet-only --tar > tutorial.rosinstall
wstool init -j4 src tutorial.rosinstall

# install needed Ubuntu packages
if [ "$ROSRELEASE" != "indigo" ]; then
    NEEDED_PKGS=$(rosdep install --reinstall -s --from-paths src --ignore-src --rosdistro $ROSRELEASE | awk '/apt-get.*install/ {print $NF}' | xargs)
else
    # indigo does not support Ubuntu vivid, so hardcode the list (output
    # from above command run on trusty)
    NEEDED_PKGS="libapr1-dev libaprutil1-dev python-nose python-paramiko python-yaml cmake python-empy python-netifaces libqt4-dev qt4-qmake libgtest-dev python-coverage libboost-all-dev liblog4cxx10-dev python-mock python-dev python-numpy python-rospkg libtinyxml-dev libconsole-bridge-dev python-catkin-pkg python-rosdep libqt4-dbus libqt4-network libqt4-script libqt4-test libqt4-xml libqtcore4 pkg-config"
fi

# resolve all dependencies of the above
NEEDED_PKGS_DEPS="$NEEDED_PKGS_DEPS $(apt-get install -s $NEEDED_PKGS|grep ^Inst | awk '{print $2}')"

sudo apt-get install -y --no-install-recommends $NEEDED_PKGS

# build ROS packages
./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release

# respect $TMPDIR, work around https://github.com/ros/catkin/issues/710
sed -i '/mktemp/ s!/tmp/!--tmpdir !' install_isolated/setup.sh

# add snappy metadata
mkdir -p install_isolated/meta
cat <<EOF > install_isolated/meta/package.yaml
name: ros-tutorial
architecture: `dpkg --print-architecture`
version: 0.3
vendor: Martin Pitt <martin.pitt@ubuntu.com>
icon: meta/ros-tutorial.svg
binaries:
 - name: bin/rossnap
EOF
# FIXME: proper icon
touch install_isolated/meta/ros-tutorial.svg
cat <<EOF > install_isolated/meta/readme.md
ROS tutorial

This is the ROS tutorial snappy app.
EOF

# copy in Ubuntu package dependencies
# FIXME: this is missing files created in postinst/at runtime, alternatives, etc.
mkdir -p install_isolated/debs
for f in `dpkg -L $NEEDED_PKGS_DEPS`; do [ -f $f ] || continue; d=${f#/}; mkdir -p install_isolated/debs/$(dirname $d); cp -a $f install_isolated/debs/$d; done

# FIXME: add wrapper to make deb lib packages work from /apps/.../debs (will be
# fixed once we do proper overlay mounting of /debs into the app ns)
cat <<EOF > install_isolated/bin/rossnap
#!/bin/bash -e
mydir=\$(dirname \$(dirname \$0))
export PYTHONPATH=\$mydir/debs/usr/lib/python2.7/dist-packages:\$PYTHONPATH
export PATH=\$mydir/debs/usr/bin:\$mydir/debs/bin::\$PATH
export LD_LIBRARY_PATH=\$mydir/debs/lib:\$mydir/debs/lib/x86_64-linux-gnu:\$mydir/debs/usr/lib:\$mydir/debs/usr/lib/x86_64-linux-gnu:\$PATH
. \$mydir/setup.bash
exec "\$@"
EOF
chmod 755 install_isolated/bin/rossnap

# FIXME: work around snappy build bug with dangling symlinks (LP: #1400304)
for l in `find install_isolated/debs -type l`; do [ -e $l ] || rm $l; done

# FIXME: work around snappy install bug with dangling symlinks (LP: #1400303)
rm -f install_isolated/debs/usr/share/X11/rgb.txt

# remove -dev packages
for p in $NEEDED_PKGS_DEPS; do
    if [ "${p%-dev}" != "$p" ]; then
        echo "   removing $p"
        for f in `dpkg -L $p`; do [ ! -f $f ] || rm -f install_isolated/debs/$f; done
    fi
done

# install snappy tools
echo "deb http://ppa.launchpad.net/snappy-dev/tools/ubuntu vivid main" | sudo tee /etc/apt/sources.list.d/snappy.list
wget -O - 'http://keyserver.ubuntu.com:11371/pks/lookup?op=get&search=0xF1831DDAFC42E99D' | sudo apt-key add -
sudo apt-get update --no-list-cleanup -o Dir::Etc::sourcelist=/etc/apt/sources.list.d/snappy.list -o Dir::Etc::sourceparts=/dev/null
sudo apt-get install -y snappy-tools

# build the .snap
snappy build install_isolated

if [ ! -e $WORKSPACE/ros-tutorial*.snap ]; then
    echo "failed to build $WORKSPACE/ros-tutorial*.snap" >&2
    exit 1
fi

echo Successfully built $(ls $WORKSPACE/ros-tutorial*.snap)
