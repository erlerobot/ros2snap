#!/usr/bin/env python

# Usage: python generate_snappy_metadata.py <package>
# run at the root of a catkin workspace
# Consequences:
#   write package.yaml file to snappy_build/<package>/meta
#   Copy install directory of catkin workspace to snappy_build
#   Copy all run dependency files into snappy_build/<package>
# TODO check if all copies can be turned into symlinks

import bloom.generators.common
import catkin_pkg.packages

import apt
import os
import shutil
from sys import argv

def copy_installed_files(key):
  # Resolve key to system package name
  key_entry = bloom.generators.common.resolve_rosdep_key(key, "ubuntu", "trusty")
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

def copy_recursive_dependencies(package):
  run_dep_keys = [dep.name for dep in package.run_depends]

  for run_dep in run_dep_keys:
    copy_installed_files(run_dep)
    # Get the package.xml of the run dependencies if it's a ROS package
    # TODO what to do about system packages?
    ros_path = os.path.dirname(os.getenv("ROS_ROOT")) + "/" + run_dep
    packages = catkin_pkg.packages.find_packages(ros_path)
    for package in packages.values():
      copy_recursive_dependencies(package)


def check_create_dir(dirname):
  if not os.path.exists(dirname):
    os.mkdir(dirname)

distro = os.getenv("ROS_DISTRO", "jade")

path = os.getcwd()
package_key = ""
if len(argv) > 1:
  package_key = argv[1]

packages = catkin_pkg.packages.find_packages(path+"/src/")

if len(packages) == 0:
  print "No packages found in catkin workspace. Abort."
  exit()

# If no package was specified, just grab the first one
if package_key == "":
  package_key = packages.keys()[0]
  print "No package given on command line. Retrieving package: " + package_key

package = packages[package_key]
if package is None:
  print "Requested package " + package_key + " not found, abort."
  exit()

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

copy_recursive_dependencies(package)

# Inject description into readme.md
f = open(snappy_meta_dir + "readme.md", "w+")
f.write(package_key + "\n\n")
f.write(description)
f.close()

# TODO: architecture, icon metadata and icon path

binaries_string = ""

# Collect all executable files from install/lib/<package>
# Need to generate a wrapper bash script for each executable
pkg_lib_dir = "install/lib/" + package_key + "/"
if os.path.exists(pkg_lib_dir):
  lib_binaries = [binary for binary in os.listdir(pkg_lib_dir)\
      if os.access(pkg_lib_dir + binary, os.X_OK) and os.path.isfile(pkg_lib_dir + binary)]

  snappy_lib_dir = snappy_bin_dir + "lib/"
  check_create_dir(snappy_lib_dir)

  for binary in lib_binaries:
    f = open(snappy_lib_dir + binary, "w+")
    script = "#!/usr/bin/bash\n" +\
             "mydir=\"$( cd \"../../$( dirname \"${BASH_SOURCE[0]}\" )\" && pwd )\"\n" +\
             "echo \"Snappy app root is: $mydir\"\n" +\
             ". $mydir/opt/ros/" + distro + "/setup.bash\n" + \
             ". $mydir/install/setup.bash\n" +\
             "$mydir/" + pkg_lib_dir + binary
    f.write(script)
    f.close()

  binaries_string += '\n'.join([" - name: bin/lib/" + binary for binary in lib_binaries])

# Also collect scripts that were installed to install/share/<package>
pkg_share_dir = "install/share/" + package_key + "/"
if os.path.exists(pkg_share_dir):
  share_scripts = [script for script in os.listdir(pkg_share_dir)\
      if os.access(pkg_share_dir + script, os.X_OK) and os.path.isfile(pkg_share_dir + binary)]

  snappy_share_dir = snappy_bin_dir + "share/"
  check_create_dir(snappy_share_dir)

  for binary in share_scripts:
    f = open(snappy_bin_dir + "share/" + binary, "w+")
    script = "#!/usr/bin/bash\n" +\
             "mydir=\"$( cd \"../../$( dirname \"${BASH_SOURCE[0]}\" )\" && pwd )\"\n" +\
             ". $mydir/opt/ros/" + distro + "/setup.bash\n" + \
             ". $mydir/install/setup.bash\n" +\
             "$mydir/" + pkg_share_dir + binary
    f.write(script)
    f.close()

  binaries_string += "\n".join([" - name: bin/share/" + script for script in share_scripts])

# Create scripts launching the launchfiles out of our package
launchdir =  pkg_share_dir + "launch"
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

# TODO: Tell the snappy people that the service does not seem to start, or that this approach is useless because
# it does not affect the environment
# Write a script that sets up our environment and add it as a service
# Get name of folder where the script lives
"""service ="#!/bin/bash\n" +\
         "mydir=\"$(dirname $( cd \"$( dirname \"${BASH_SOURCE[0]}\" )\" && pwd ))\"\n" +\
         ". $mydir/opt/ros/" + distro + "/setup.bash\n" + \
         ". $mydir/install/setup.bash\n"

f = open(snappy_bin_dir + package_key + "_service.start", "w+" )
f.write(service)
f.close()

services_string = " - name: " + package_key + "_service\n" +\
    "   start: \". bin/" + package_key + "_service.start\"\n" +\
    "   description: Set up environment for " + package_key"""

# TODO Inject extra necessary dependencies (Is this needed? Maybe not, if we get all recursive deps.)

data = "name: " + package_key + "\n" +\
       "version: " + version + "\n" +\
       "vendor: " + maintainer_string + "\n" +\
       "binaries:\n" + binaries_string #+ "\n" +\
       #"services:\n" + services_string

f = open(snappy_meta_dir + "package.yaml", "w+")
f.write(data)
f.close()
