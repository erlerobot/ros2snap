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
import rosdep2

import apt
import os
import shutil
from sys import argv

def copy_installed_files(key):
  # Resolve key to system package name
  apt_name = bloom.generators.common.resolve_rosdep_key(key, "ubuntu", "trusty")[0][0]

  # Use apt python API to get all files
  cache = apt.Cache()
  run_pkg = cache[apt_name]

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
      shutil.copy(dep_path, fullpath)



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

if not os.path.exists("snappy_build"):
  os.mkdir("snappy_build")

# Copy all files in install to snappy_build/<package>
shutil.copytree("install", "snappy_build/" + package_key + "/install")

snappy_meta_dir = "snappy_build/" + package_key + "/meta/"
snappy_bin_dir = "snappy_build/" + package_key + "/bin/"

if not os.path.exists(snappy_meta_dir):
  os.mkdir(snappy_meta_dir)

if not os.path.exists(snappy_bin_dir):
  os.mkdir(snappy_bin_dir)

# get first maintainer name and email
maintainer_string = package.maintainers[0].name + " " + package.maintainers[0].email

version = package.version

description = package.description

# Inject description into readme.md
f = open(snappy_meta_dir + "readme.md", "w+")
f.write(package_key + "\n\n")
f.write(description)
f.close()

# TODO: architecture, icon metadata and icon path

binaries_string = ""
# Collect all executable files from install/lib/<package> and launchfiles
pkg_lib_dir = "install/lib/" + package_key + "/"
if os.path.exists(pkg_lib_dir):
  lib_binaries = [binary for binary in os.listdir(pkg_lib_dir) if os.access(pkg_lib_dir + binary, os.X_OK)]
  for lib_binary in lib_binaries:
    shutil.copyfile(pkg_lib_dir + lib_binary, snappy_bin_dir + lib_binary)

  binaries_string += '\n'.join([" - name: install/lib/" + binary for binary in lib_binaries])

# Create scripts launching the launchfiles out of our package
launchdir = "install/share/" + package_key + "/launch"
if os.path.exists(launchdir):
  launchfiles = [os.listdir(launchdir)]
  for launchfile in launchfiles:
    launch_string = "roslaunch " + package_key + " " + launchfile
    f = open(snappy_bin_dir)
    f.write(launch_string)
    f.close()

  binaries_string += '\n'.join([" - name: bin/" + launch for launch in launchfiles])

# TODO for binaries specify exec and name fields, figure out duplicate entries

# TODO get keys recursively?
run_dep_keys = [dep.name for dep in package.run_depends]

for run_dep in run_dep_keys:
  copy_installed_files(run_dep)

# Copy /opt/ros/<distro>/setup.bash, since deps does not take care of it for us.
shutil.copy("/opt/ros/indigo/setup.bash", "snappy_build/" + package_key + "/opt/ros/indigo/setup.bash")

# Write a script that sets up our environment and add it as a service
service ="#!/bin/bash -e\n" +\
         "mydir=\$(dirname \$(dirname \$0))\n" + \
         ". $mydir/opt/ros/" + distro + "/setup.bash\n" + \
         ". $mydir/install/setup.bash\n" +\
         "roscore"

f = open(snappy_bin_dir + package_key + "_service.start", "w+" )
f.write(service)
f.close()

services_string = " - name: " + package_key + "_service\n" +\
    "   start: bin/" + package_key + "_service.start\n" +\
    "   description: Set up environment for " + package_key

# TODO Inject extra necessary dependencies (Is this needed? Maybe not, if we get all recursive deps.)

data = "name: " + package_key + "\n" +\
       "version: " + version + "\n" +\
       "vendor: " + maintainer_string + "\n" +\
       "binaries:\n" + binaries_string + "\n" +\
       "services:\n" + services_string

f = open(snappy_meta_dir + "package.yaml", "w+")
f.write(data)
f.close()
