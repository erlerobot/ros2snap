#!/usr/bin/env python

# Usage: python generate_snappy_metadata.py <package>
# run at the root of a catkin workspace
# Consequences:
#   write package.yaml file to snappy_build/<package>/meta
#   Copy install directory of catkin workspace to snappy_build
#   Copy all run dependency files into snappy_build/<package>

# TODO Command line args

import bloom.generators.common
import catkin_pkg.packages

import apt
import os
import shutil
import stat
import sys

def check_create_dir(dirname):
  if not os.path.exists(dirname):
    os.mkdir(dirname)

# Resolve and recursively copy dependencies
class SnappyBuilder:
  def __init__(self, package_key, packages):
    self.copied_packages = set()
    self.package_key = package_key
    self.package_key_final = package_key.replace("_", "-").lower()
    self.package = packages[package_key]
    if self.package is None:
      print "Requested package " + self.package_key + " not found, abort."
      sys.exit(-1)

    self.distro = os.getenv("ROS_DISTRO", "jade")

    self.snappy_root = "snappy_build/" + self.package_key_final + "/"
    self.snappy_meta_dir = self.snappy_root + "meta/"
    self.snappy_bin_dir = self.snappy_root + "bin/"

    # TODO for performance: exclusion rule for top-level packages
    # to ignore for recursive dependency walk

    self.cache = apt.Cache()

  def resolve_and_copy(self, key):
    # Resolve key to system package name
    key_entry = bloom.generators.common.resolve_rosdep_key(key, "ubuntu", "trusty")
    if key_entry is None or (key_entry[0] is None) or (len(key_entry[0]) == 0):
      return

    apt_name = key_entry[0][0]
    self.copy_installed_files(apt_name)

  def copy_files(self, run_dep_files):
    # Copy all the files
    for dep_path in run_dep_files:
      if os.path.isfile(dep_path):
        fullpath = self.snappy_root + dep_path
        if not os.path.exists(os.path.dirname(fullpath)):
          os.makedirs(os.path.dirname(fullpath))
        shutil.copy2(dep_path, fullpath)

  def copy_installed_files(self, apt_name):
    run_pkg = self.cache[apt_name]
    if not apt_name.startswith("ros-" + self.distro + "-"):
    #if apt_name in self.needs_system_deps:
      versions = run_pkg.versions
      # system packages
      if len(versions) > 0:
        version = versions[0]
        for dependency in version.dependencies:
          key = dependency[0].name
          if key in self.copied_packages or not self.cache.has_key(key):
            continue
          #self.copy_files(dep_pkg.installed_files)
          self.copied_packages.add(key)
          self.copy_installed_files(key)

    # Use apt python API to get all files
    #cache = apt.Cache()
    # TODO: Catch errors
    # TODO: If run dependency not installed, install it using package manager
    # TODO: What about local installs?

    self.copy_files(run_pkg.installed_files)

  def copy_recursive_dependencies(self, package):
    run_dep_keys = [dep.name for dep in package.run_depends]

    for run_dep in run_dep_keys:
      if run_dep in self.copied_packages:
        continue
      self.resolve_and_copy(run_dep)
      self.copied_packages.add(run_dep)
      # Get the package.xml of the run dependencies if it's a ROS package
      ros_path = os.path.dirname(os.getenv("ROS_ROOT")) + "/" + run_dep
      packages = catkin_pkg.packages.find_packages(ros_path)
      for package in packages.values():
        self.copy_recursive_dependencies(package)

  def collect_binaries(self, path):
    pkg_dir = "install/" + path + "/" + self.package_key + "/"
    if os.path.exists(pkg_dir):
      snappy_dir = self.snappy_bin_dir + path + "/"
      check_create_dir(snappy_dir)

      ret = ""
      for binary in os.listdir(pkg_dir):
        if os.access(pkg_dir + binary, os.X_OK) and os.path.isfile(pkg_dir + binary):
          binary_final = binary.replace("_", "-")
          script_path = snappy_dir + binary_final
          f = open(snappy_dir + binary_final, "w+")
          # TODO Parse python path version
          # for the love of god, do something better to get the snap root...
          script =\
"""#!/bin/bash
set -x
mydir=$(dirname $(dirname $(builtin cd "`dirname "${BASH_SOURCE[0]}"`" > /dev/null && pwd)))
. $mydir/install/setup.bash
. $mydir/opt/ros/%s/setup.bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$mydir/usr/lib/x86_64-linux-gnu:$mydir/usr/lib
export PATH=$PATH:$mydir/usr/bin
export PYTHONPATH=$PYTHONPATH:$mydir/usr/lib/python2.7/dist-packages
export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:$mydir/usr/lib/pkgconfig:$mydir/usr/lib/x86_64-linux-gnu/pkgconfig
$mydir/%s
set +x""" % (self.distro, pkg_dir + binary)

          f.write(script)
          f.close()
          st = os.stat(script_path)
          os.chmod(script_path, st.st_mode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)
          ret += " - name: bin/" + path + "/" + binary_final + "\n"

      return ret
    return ""

  def create_dir_structure(self):
    if os.path.exists(self.snappy_root):
      shutil.rmtree(self.snappy_root)

    check_create_dir("snappy_build")

    # Copy all files in install to snappy_build/<package>
    shutil.copytree("install", self.snappy_root + "install")

    check_create_dir(self.snappy_meta_dir)

    check_create_dir(self.snappy_bin_dir)

  def parse_write_metadata(self):
    description = self.package.description

    # Inject description into readme.md
    f = open(self.snappy_meta_dir + "readme.md", "w+")
    f.write(self.package_key + "\n\n")
    f.write(description)
    f.close()

    # get first maintainer name and email
    maintainer_string = self.package.maintainers[0].name + " <" + self.package.maintainers[0].email + ">"

    # TODO icon, architecture

    version = self.package.version

    binaries_string = ""

    print "Checking lib, share, and launch for executables"
    binaries_string += self.collect_binaries("lib")
    binaries_string += self.collect_binaries("share")

    # Create scripts launching the launchfiles out of our package
    launchdir =  "install/share/" + self.package_key + "/launch"
    if os.path.exists(launchdir):
      launchfiles = [os.listdir(launchdir)]
      for launchfile in launchfiles:
        launch_string = "roslaunch " + self.package_key + " " + launchfile
        f = open(self.snappy_bin_dir, "w+")
        f.write(launch_string)
        f.close()

      binaries_string += '\n'.join([" - name: bin/" + launch for launch in launchfiles])

    data = "name: " + self.package_key_final + "\n" +\
           "version: " + version + "\n" +\
           "vendor: " + maintainer_string + "\n" +\
           "binaries:\n" + binaries_string

    f = open(self.snappy_meta_dir + "package.yaml", "w+")
    f.write(data)
    f.close()

  def copy_env_scripts(self):
    # Copy /opt/ros/<distro>/setup.bash and dependencies
    # better way of doing this?
    shutil.copy2("/opt/ros/indigo/setup.bash",\
        self.snappy_root + "opt/ros/indigo/setup.bash")
    shutil.copy2("/opt/ros/indigo/setup.sh",\
        self.snappy_root + "opt/ros/indigo/setup.sh")
    shutil.copy2("/opt/ros/indigo/_setup_util.py",\
        self.snappy_root + "opt/ros/indigo/_setup_util.py")


path = os.getcwd()
package_key = ""
if len(sys.argv) > 1:
  package_key = sys.argv[1]

# Is there a fancier way to do this?
os.system("catkin_make install")

packages = catkin_pkg.packages.find_packages(path + "/src/")

if len(packages) == 0:
  print "No packages found in catkin workspace. Abort."
  exit()

# If no package was specified, just grab the first one
if package_key == "":
  package_key = packages.keys()[0]
  print "No package given on command line. Retrieving package: " + package_key

builder = SnappyBuilder(package_key, packages)

print "Building snap for package " + package_key
builder.create_dir_structure()

print "Parsing metadata from package.xml and writing to meta/package.yaml"
builder.parse_write_metadata()

print "Copying all recursive run dependencies into snap"
builder.copy_recursive_dependencies(builder.package)
builder.cache.close()

builder.copy_env_scripts()

os.system("snappy build snappy_build/" + builder.package_key_final)
