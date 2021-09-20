# IDAS-quadrotor
## Description
This Repository contains a model of a simple quadrotor and controllers made by ROS and simulated using Gazebo. This project is not intended to be applicable in general (since it's a final project for a bachelor degree only) but the goal is to improve it as much as possible and make a decent metapackage for beginners and for people searching for simple ROS or quadrotor simulation projects on internet.
The control methods used in this project are full-state controller and PID controller.

### TODO
1. update `flight_mode` package.

## Installation

### 1.Installing dependencies
First you have to install these packages:
1. Install [ROS desktop full](http://wiki.ros.org/noetic/Installation/Ubuntu) which installs Gazebo too.
2. Install PID package for your ROS distro.
```bash
sudo apt-get install ros-noetic-pid
```
3. Install numpy.
```bash
sudo apt install python3-numpy
```

### 2.Initializing Workspace
To use packages, first you need to make a catkin workspace, then clone the repository in src subdirectory in your workspace.
afterwards its proper to source your workspace in .bashrc file so whenever you open a new terminal you don't have to source it manually:
```bash
cd ~
echo 'source ~/[workspace_name]/devel/setup.bash' >> .bashrc
cd ~/[workspace_name]/src/
git clone https://github.com/siavash1999/IDAS-quadrotor.git
```
Put the name of your workspace insted of [workspace_name] in the command above.
[More About Catkin Workspace](http://wiki.ros.org/catkin/workspaces)

### 3.Building PropPlugin
In this project a Model plugin is written for gazebo that needs to be built using CMake. In order to build the shared library for plugin, following procedure must be done:
```bash
cd ~/[workspace_name]/src/PropPlugin
mkdir build
cd ./build
cmake ..
make
```
And then we should add the shared library directory to .bashrc file:
```bash
cd ~
echo 'export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:~/[workspace_name]/src/PropPlugin/build' >> .bashrc
```

### 4.Making ROS python Packages

As strange as it may seem, catkin python packages have a build stage in which the modules used in scripts are added to PYTHONPATH. The whole explaination is available in wiki page (soon, hopefully!). For building all that needs to be done is to follow below procedure:
```bash
cd ~/[workspacename]
catkin_make
```
This command will invoke CMakeLists.txt files in all packages and makes ``` /build ``` and ``` /devel ``` subdirectories in workspace.
[More About ROS Python Makefile](http://wiki.ros.org/rospy_tutorials/Tutorials/Makefile)

## How To Use
First the core node of ROS must be launched using command `roscore` in one terminal. Then in another Terminal, `robot_description` can be uploaded into ROS parameter server and spawned in gazebo using `roslaunch` :
```bash
roslaunch quadrotor quadrotor.launch
```
Now using `controller.launch` launch file, the quadrotor model will rise up from ground and hover in an approximate altitude of 1m above ground. make sure that simulation is not paused when launching.
```bash
roslaunch quadrotor controller.launch
```

## Contribution
Feel free to contact package maintainer, all questions, critisim and bug/issue reports are welcome and you can find maintainer Email address in Package.xml manifest files of all packages.

## License
[BSD](https://opensource.org/licenses/BSD-3-Clause)

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
