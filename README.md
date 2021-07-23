# IDAS-quadrotor
## Description
This Repository contains a model of a simple quadrotor and controllers made by ROS and simulated using Gazebo. This project is not intended to be applicable in general (since it's a final project for a bachelor degree only) but the goal is to improve it as much as possible and make a decent metapackage for beginners and for people searching for simple ROS or quadrotor simulation projects on internet.

## Installation
### 1.Initializing Workspace
To use packages, first you need to make a catkin workspace, then clone the repository in src subdirectory in your workspace.
afterwards its proper to source your workspace in .bashrc file so whenever you open a new terminal you don't have to source it manually:
```bash
cd ~
echo 'source ~/[workspace_name]/devel/setup.bash' >> .bashrc
```
Put the name of your workspace insted of [workspace_name] in the command above.
[More About Catkin Workspace](http://wiki.ros.org/catkin/workspaces)

### 2.Building PropPlugin
In this project a Model plugin is written for gazebo that needs to be built using CMake. In order to build the shared library for plugin, following procedure must be done:
```bsah
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
## How To Use
First the core node of ROS must be launched using command `roscore` in one terminal. Then in another Terminal, `robot_description` can be uploaded into ROS parameter server and spawned in gazebo using 'roslaunch' :
```bash
roslaunch quadrotor quadrotor.launch
```
Now using `hover_control.py` node, the quadrotor model will rise up from ground and hover in an approximate altitude of 5m above ground. make sure that simulation is not paused when running the node.
```bash
rosrun flight_mdoes hover_control.py
```
## To Do
1. Making other flight modes (Cruise and Spin) Nodes for package `flight_modes` .
2. Using a seperate package for subscribing setpoint from client and deciding what flight mode or sequence of flight modes are needed to achieve the goal.
3. Replacing VelocityBasedControllers with EffortBasedControllers.
4. Tuning EffortBasedController gains (PID gains) to achieve best behavior.
5. Making a better model of quadrotor (for now it's just a red box with 4 black boxes.)
6. For now, hover_control node is written based on a x configuration of propellers in drones. Adding a parameter to define if the drone is + or x configured and changing the FtoS matrix in hover_control (and other flight mode nodes in future) to more general form, including + configuration too. 
7. Making packages more general and applicable for other drones.
8. Making Path Planning packages!

## Contribution
Feel free to contact package maintainer, all questions, critisim and bug/issue reports are welcome and you can find maintainer Email address in Package.xml manifest files of all packages.

## License
[BSD](https://opensource.org/licenses/BSD-3-Clause)

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
