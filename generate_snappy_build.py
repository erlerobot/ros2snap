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
    os.makedirs(dirname)

# Resolve and recursively copy dependencies
class SnappyBuilder:
  def __init__(self, package_key, packages, pkg_root):
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
    self.pkg_root = pkg_root
    print "Package root: " + pkg_root
    setup = ". $mydir/opt/ros/%s/setup.bash\n" % self.distro
    if self.pkg_root == "install/":
      setup += ". $mydir/%ssetup.bash" % self.pkg_root
    self.environment_script = """#!/bin/bash
mydir=$(dirname $(dirname $(builtin cd "`dirname "${BASH_SOURCE[0]}"`" > /dev/null && pwd)))
%s
export ROS_MASTER_URI=http://localhost:11311
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$mydir/usr/lib/x86_64-linux-gnu:$mydir/usr/lib
export PATH=$PATH:$mydir/usr/bin
export PYTHONPATH=$PYTHONPATH:$mydir/usr/lib/python2.7/dist-packages
export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:$mydir/usr/lib/pkgconfig:$mydir/usr/lib/x86_64-linux-gnu/pkgconfig
""" % setup

    self.cache = apt.Cache()

  def resolve_and_copy(self, key):
    # Resolve key to system package name
    # TODO: Cross-platform solution for args
    key_entry = bloom.generators.common.resolve_rosdep_key(key, "ubuntu", "trusty")
    if key_entry is None or (key_entry[0] is None) or (len(key_entry[0]) == 0):
      return

    apt_name = key_entry[0][0]
    self.copy_from_apt_cache(apt_name)

  def copy_files(self, run_dep_files):
    # Copy all the files
    for dep_path in run_dep_files:
      if os.path.isfile(dep_path):
        fullpath = self.snappy_root + dep_path
        check_create_dir(os.path.dirname(fullpath))
        shutil.copy2(dep_path, fullpath)

  def copy_from_apt_cache(self, apt_name):
    # Use apt python API to get all files
    run_pkg = self.cache[apt_name]

    # TODO for performance: exclusion rule for top-level packages (e.g. python)
    # that snappy instance already has
    if not apt_name.startswith("ros-" + self.distro + "-"):
      versions = run_pkg.versions
      # system packages
      if len(versions) > 0:
        version = versions[0]
        for dependency in version.dependencies:
          key = dependency[0].name
          if key in self.copied_packages or not self.cache.has_key(key):
            continue
          self.copied_packages.add(key)
          self.copy_from_apt_cache(key)

    # TODO: Catch more errors
    # TODO: install missing run deps with package manager
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
    install_dir = "install/" + path + "/" + self.package_key + "/"
    pkg_dir = self.snappy_root + install_dir
    if os.path.exists(pkg_dir):
      snappy_dir = self.snappy_bin_dir + path + "/"
      check_create_dir(snappy_dir)

      ret = ""
      for binary in os.listdir(pkg_dir):
        if os.access(pkg_dir + binary, os.X_OK) and os.path.isfile(pkg_dir + binary):
          binary_final = binary.replace("_", "-")
          script_path = snappy_dir + binary_final
          f = open(snappy_dir + binary_final, "w+")
          # TODO Parse python version for PYTHONPATH
          # is there an env variable to get the snap root...?
          script = self.environment_script + "$mydir/%s" % (install_dir + binary)

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
    check_create_dir(self.snappy_meta_dir)
    check_create_dir(self.snappy_bin_dir)

  def copy_files_from_pkg_root(self):
    # If it's a valid root, first level will have bin, lib, include, share
    for path in os.listdir(self.pkg_root):
      # Check this folder for the package name
      if os.path.isdir(self.pkg_root + path) and (self.package_key in os.listdir(self.pkg_root + path)):
        # Copy the contents to snappy_root
        shutil.copytree(self.pkg_root + path + "/" + self.package_key,\
            self.snappy_root + "install/" + path + "/" + self.package_key)

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
    launchdir = self.pkg_root + "/share/" + self.package_key + "/launch"
    if os.path.exists(launchdir):
      launchfiles = os.listdir(launchdir)
      check_create_dir(self.snappy_bin_dir + "launch")
      for launchfile in launchfiles:
        launch_string = self.environment_script + "roslaunch " + self.package_key + " " + launchfile
        dst = self.snappy_bin_dir+ "launch/" +launchfile
        f = open(dst, "w+")
        f.write(launch_string)
        f.close()
        st = os.stat(dst)
        os.chmod(dst, st.st_mode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)

      binaries_string += '\n'.join([" - name: bin/launch/" + launch for launch in launchfiles])

    data = "name: " + self.package_key_final + "\n" +\
           "version: " + version + "\n" +\
           "vendor: " + maintainer_string + "\n" +\
           "binaries:\n" + binaries_string

    f = open(self.snappy_meta_dir + "package.yaml", "w+")
    f.write(data)
    f.close()

  def copy_env_scripts(self):
    # with the correct dependencies copied, this function is unnecessary
    # Copy /opt/ros/<distro>/setup.bash and dependencies
    # better way of doing this?
    rospath = os.path.dirname(os.path.dirname(os.getenv("ROS_ROOT")))
    dstpath = self.snappy_root + rospath
    check_create_dir(dstpath)
    shutil.copy2(rospath + "/setup.bash", dstpath + "/setup.bash")
    shutil.copy2(rospath + "/setup.sh", dstpath + "/setup.sh")
    shutil.copy2(rospath + "/_setup_util.py", dstpath + "/_setup_util.py")

  def build(self):
    print "Building snap for package " + self.package_key

    print "Parsing metadata from package.xml and writing to meta/package.yaml"
    self.parse_write_metadata()

    print "Copying all recursive run dependencies into snap"
    self.copy_recursive_dependencies(self.package)
    self.cache.close()

    self.copy_env_scripts()

    os.system("snappy build snappy_build/" + self.package_key_final)

def build_from_source():
  path = os.getcwd()
  package_key = ""
  if len(sys.argv) > 1:
    package_key = sys.argv[1]

  # Is there a fancier way to do this?
  os.system("catkin_make install")

  packages = catkin_pkg.packages.find_packages(path + "/src/")

  if len(packages) == 0:
    print "No packages found in catkin workspace. Abort."
    return

  # If no package was specified, just grab the first one
  if package_key == "":
    package_key = packages.keys()[0]
    # TODO: Change this to build the entire workspace as a snap? Namespaces?
    print "No package given on command line, choosing: " + package_key

  builder = SnappyBuilder(package_key, packages, "install/")

  # Copy all files in install to snappy_build/<package>

  builder.create_dir_structure()
  builder.copy_files_from_pkg_root()
  builder.build()

def build_from_debs():
  # TODO: Install the package debian if it is not found
  package_key = ""
  if len(sys.argv) > 1:
    package_key = sys.argv[1]
  else:
    print "Must specify a package name to build snap from ROS debs"
    return

  ros_path = os.path.dirname(os.getenv("ROS_ROOT"))
  packages = catkin_pkg.packages.find_packages(ros_path)

  if len(packages) <= 0:
    print "Error: no catkin packages found in " + ros_path + ". Abort."
    return

  builder = SnappyBuilder(package_key, packages, os.path.dirname(ros_path) + "/")

  builder.create_dir_structure()
  builder.copy_files_from_pkg_root()
  # get all the stuff we need that isn't under the package name
  key_entry = bloom.generators.common.resolve_rosdep_key(builder.package_key, "ubuntu", "trusty")
  if key_entry is None or (key_entry[0] is None) or (len(key_entry[0]) == 0):
    return

  apt_name = key_entry[0][0]
  pkg = builder.cache[apt_name]
  builder.copy_files(pkg.installed_files)

  builder.build()



build_from_debs()
