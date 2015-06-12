#!/usr/bin/env python

# Usage: python generate_snappy_metadata.py <package>
# run at the root of a catkin workspace
# Consequences:
#   write package.yaml file to snappy_build/<package>/meta
#   Copy install directory of catkin workspace to snappy_build
#   Copy all run dependency files into snappy_build/<package>
# TODO check if all copies can be turned into symlinks
# TODO Command line args

import bloom.generators.common
import catkin_pkg.packages

import apt
import os
import shutil
from sys import argv

def copy_installed_files(key):
  # Resolve key to system package name
  key_entry = bloom.generators.common.resolve_rosdep_key(key, "ubuntu", "trusty")
  if key_entry is None or (key_entry[0] is None) or (len(key_entry[0]) == 0):
    return

  apt_name = key_entry[0][0]

  # Use apt python API to get all files
  cache = apt.Cache()
  run_pkg = cache[apt_name]

  # TODO: Catch errors
  # TODO: If run dependency not installed, install it using package manager
  # TODO: What about local installs?

  run_dep_files = run_pkg.installed_files
  cache.close()

  # Copy all the files
  for dep_path in run_dep_files:
    if os.path.isfile(dep_path):
      fullpath = "snappy_build/" + package_key + dep_path
      if not os.path.exists(os.path.dirname(fullpath)):
        os.makedirs(os.path.dirname(fullpath))
      shutil.copy2(dep_path, fullpath)

def copy_recursive_dependencies(package, copied_packages):
  run_dep_keys = [dep.name for dep in package.run_depends]

  for run_dep in run_dep_keys:
    if run_dep in copied_packages:
      continue
    copy_installed_files(run_dep)
    copied_packages.add(run_dep)
    # Get the package.xml of the run dependencies if it's a ROS package
    ros_path = os.path.dirname(os.getenv("ROS_ROOT")) + "/" + run_dep
    packages = catkin_pkg.packages.find_packages(ros_path)
    for package in packages.values():
      copy_recursive_dependencies(package, copied_packages)


def check_create_dir(dirname):
  if not os.path.exists(dirname):
    os.mkdir(dirname)

def collect_binaries(path):
  pkg_dir = "install/" + path + "/" + package_key + "/"
  if os.path.exists(pkg_dir):
    snappy_dir = snappy_bin_dir + path + "/"
    check_create_dir(snappy_dir)

    ret = ""
    for binary in os.listdir(pkg_dir):
      if os.access(pkg_dir + binary, os.X_OK) and os.path.isfile(pkg_dir + binary):
        f = open(snappy_dir + binary, "w+")
        # TODO Parse python path version
        # for the love of god, do something better to get the snap root

        script = "#!/bin/bash\n" +\
                 "mydir=$(dirname $(dirname $(builtin cd \"`dirname \"${BASH_SOURCE[0]}\"`\" > /dev/null && pwd)))\n" +\
                 ". $mydir/install/setup.bash\n" +\
                 ". $mydir/opt/ros/" + distro + "/setup.bash\n" + \
                 "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$mydir/usr/lib/x86_64-linux-gnu\n" +\
                 "export PATH=$PATH:$mydir/usr/bin\n" +\
                 "export PYTHONPATH=$PYTHONPATH:$mydir/usr/lib/python2.7/dist-packages\n" +\
                 "export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:$mydir/usr/lib/pkgconfig:$mydir/usr/lib/x86_64-linux-gnu/pkgconfig\n" +\
                 "$mydir/" + pkg_dir + binary + "\n"
        f.write(script)
        f.close()
        ret += " - name: bin/" + path + "/" + binary + "\n"

    return ret
  return ""


distro = os.getenv("ROS_DISTRO", "jade")

path = os.getcwd()
package_key = ""
if len(argv) > 1:
  package_key = argv[1]

packages = catkin_pkg.packages.find_packages(path+"/src/")

if len(packages) == 0:
  print "No packages found in catkin workspace. Abort."
  exit()

# Is there a fancier way to do this?
os.system("catkin_make install")

# If no package was specified, just grab the first one
if package_key == "":
  package_key = packages.keys()[0]
  print "No package given on command line. Retrieving package: " + package_key

package = packages[package_key]
if package is None:
  print "Requested package " + package_key + " not found, abort."
  exit()

print "Building snap for package " + package_key

if os.path.exists("snappy_build/" + package_key):
  shutil.rmtree("snappy_build/" + package_key)

check_create_dir("snappy_build")

# Copy all files in install to snappy_build/<package>
shutil.copytree("install", "snappy_build/" + package_key + "/install")

snappy_meta_dir = "snappy_build/" + package_key + "/meta/"
snappy_bin_dir = "snappy_build/" + package_key + "/bin/"

check_create_dir(snappy_meta_dir)

check_create_dir(snappy_bin_dir)

# get first maintainer name and email
maintainer_string = package.maintainers[0].name + " " + package.maintainers[0].email

version = package.version

description = package.description

print "Copying all recursive run dependencies into snap"
copied_packages = set()
copy_recursive_dependencies(package, copied_packages)

# Inject description into readme.md
f = open(snappy_meta_dir + "readme.md", "w+")
f.write(package_key + "\n\n")
f.write(description)
f.close()

# TODO: architecture, icon metadata and icon path

binaries_string = ""

print "Checking lib, share, and launch for executables"
binaries_string += collect_binaries("lib")
binaries_string += collect_binaries("share")

# Create scripts launching the launchfiles out of our package
launchdir =  "install/share/" + package_key + "/launch"
if os.path.exists(launchdir):
  launchfiles = [os.listdir(launchdir)]
  for launchfile in launchfiles:
    launch_string = "roslaunch " + package_key + " " + launchfile
    f = open(snappy_bin_dir, "w+")
    f.write(launch_string)
    f.close()

  binaries_string += '\n'.join([" - name: bin/" + launch for launch in launchfiles])

# Copy /opt/ros/<distro>/setup.bash and dependencies
# better way of doing this?
shutil.copy2("/opt/ros/indigo/setup.bash",\
    "snappy_build/" + package_key + "/opt/ros/indigo/setup.bash")
shutil.copy2("/opt/ros/indigo/setup.sh", "snappy_build/" + package_key + "/opt/ros/indigo/setup.sh")
shutil.copy2("/opt/ros/indigo/_setup_util.py",\
    "snappy_build/" + package_key + "/opt/ros/indigo/_setup_util.py")

data = "name: " + package_key + "\n" +\
       "version: " + version + "\n" +\
       "vendor: " + maintainer_string + "\n" +\
       "binaries:\n" + binaries_string

f = open(snappy_meta_dir + "package.yaml", "w+")
f.write(data)
f.close()

os.system("snappy build snappy_build/" + package_key)
