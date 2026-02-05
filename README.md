# polman-dsk-ros2-amr
Autonomous Mobile Robot (AMR) using ROS2 framework developed in Bandung Polytechnic for Manufacturing (Polman Bandung)


### **Polebot AMR: System Setup & Deployment Guide**

#### **1. Initial System Preparation**
To ensure the AMR software runs correctly, the system must support **UTF-8** and have the necessary repositories enabled.

*   **Set Locale:** 
    ```bash
    sudo apt update && sudo apt install locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8
    ```
    *This ensures communication between ROS nodes doesn't fail due to encoding issues.*

*   **Enable Repositories:**
    ```bash
    sudo apt install software-properties-common
    sudo add-apt-repository universe
    ```
    *The Universe repository is required for many ROS 2 dependencies.*

#### **2. Automated ROS 2 Installation**
For easy re-deployment, use the `ros2-apt-source` package to automatically configure your repository settings.

*   **Configure Repository:**
    ```bash
    sudo apt update && sudo apt install curl -y
    export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
    curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
    sudo dpkg -i /tmp/ros2-apt-source.deb
    ```

*   **Install Core Packages:**
    ```bash
    sudo apt update && sudo apt upgrade
    sudo apt install ros-jazzy-desktop ros-dev-tools -y
    ```
    *The **Desktop Install** includes RViz, demos, and tutorials essential for AMR development.*

#### **3. Re-deployment Automation (Environment Setup)**
To avoid manually running setup scripts every time you open a terminal, automate the environment variables.

*   **Clone this Repository:**
    ```bash
    git clone https://github.com/nj-ramadhan/polman-mbd-ros2-polebot-amr.git
    ```


*   **Configure Bash Session:**
    ```bash
    echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
    echo "source ~/polebot_amr_ws/install/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    ```
    *This command tells your computer how to find ROS 2 commands automatically in every new terminal window.*

*   **Initialize Dependency Manager:**
    ```bash
    sudo rosdep init
    rosdep update
    ```
    *`rosdep` helps manage and install dependencies for your specific Polebot AMR packages.*

#### **4. Simulation & Support Tools**
Polebot AMR relies on **Gazebo** for simulation and **NumPy** for scientific computing.

*   **Install Gazebo and Python Tools:**
    ```bash
    sudo apt-get install python3-pip python3-numpy -y
    sudo apt-get install ros-jazzy-ros-gz -y
    ```

*   **Verification:** Test the simulation environment by running:
    ```bash
    gz sim -v 4 shapes.sdf
    ```
    *If successful, you can also launch it via ROS 2:*
    `ros2 launch ros_gz_sim gz_sim.launch.py gz_args:="shapes.sdf"`.

#### **5. Validating Communication (The "Hello World")**
To ensure the Polebot nodes can talk to each other, use the built-in demo nodes.

1.  **Terminal 1 (Publisher):** `ros2 run demo_nodes_cpp talker`.
2.  **Terminal 2 (Subscriber):** `ros2 run demo_nodes_py listener`.

**Tip:** Install **Terminator** to manage multiple AMR processes in a single window.
```bash
sudo apt-get install terminator -y
```

---

### **Core Concepts for Developers**
When documenting new code for Polebot AMR, remember these three vital ROS 2 terms:
1.  **Nodes:** Programs that perform tasks.
2.  **Publishers:** Programs (like sensors) that send data (e.g., a depth camera).
3.  **Subscribers:** Programs (like wheel controllers) that receive data to act upon it (e.g., stopping if an obstacle is detected).

The Polebot AMR code is distributed under the **GPL-3.0 license**. For easier workspace management, you may use the provided `.code-workspace` file in the repository.


# Install all missing dependencies automatically
```bash
rosdep install --from-paths src --ignore-src -r -y
```

# Enter the src folder to clone depedencies
```bash
cd src
git clone https://github.com/nj-ramadhan/ros2_serial.git
git clone https://github.com/nj-ramadhan/ros2_lsc.git
git clone https://github.com/nj-ramadhan/ros2_roboteq.git
git clone https://github.com/nj-ramadhan/ros2_orbbec.git
```

## ORBBEC Installation Instructions
### Install ROS 2

Please refer to the official ROS 2 installation guide guidance
If your ROS 2 command does not auto-complete, put the following two lines into your .bashrc or .zshrc
```bash
eval "$(register-python-argcomplete3 ros2)"
eval "$(register-python-argcomplete3 colcon)"
```

### Install deb dependencies
# assume you have sourced ROS environment, same blow
```bash
sudo apt install libgflags-dev nlohmann-json3-dev  \
ros-$ROS_DISTRO-image-transport  ros-${ROS_DISTRO}-image-transport-plugins ros-${ROS_DISTRO}-compressed-image-transport \
ros-$ROS_DISTRO-image-publisher ros-$ROS_DISTRO-camera-info-manager \
ros-$ROS_DISTRO-diagnostic-updater ros-$ROS_DISTRO-diagnostic-msgs ros-$ROS_DISTRO-statistics-msgs \
ros-$ROS_DISTRO-backward-ros libdw-dev
```

### Install udev rules.
```bash
cd  ~/polebot_amr_ws/src/ros2_orbbec/orbbec_camera/scripts
sudo bash install_udev_rules.sh
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Getting start
```bash
cd ~/polebot_amr_ws/
```

# build release, Default is Debug
```bash
colcon build --event-handlers  console_direct+  --cmake-args  -DCMAKE_BUILD_TYPE=Release
```

Launch camera node

On terminal 1
. ./install/setup.bash
ros2 launch orbbec_camera astra.launch.py
On terminal 2
. ./install/setup.bash
rviz2
Select the topic you want to display

List topics / services/ parameters ( on terminal 3)
ros2 topic list
ros2 service list
ros2 param list
Get device info
ros2 service call /camera/get_device_info orbbec_camera_msgs/srv/GetDeviceInfo '{}'
Get SDK version
ros2 service call /camera/get_sdk_version orbbec_camera_msgs/srv/GetString '{}'
Get exposure
ros2 service call /camera/get_color_exposure orbbec_camera_msgs/srv/GetInt32 '{}'
If your check ir or depth, please change /camera/get_color_exposure to /camera/get_ir_exposure or /camera/get_depth_exposure, Same below.

Get gain
ros2 service call /camera/get_color_gain orbbec_camera_msgs/srv/GetInt32 '{}'
Get white balance
ros2 service call /camera/get_white_balance orbbec_camera_msgs/srv/GetInt32 '{}'
Set auto exposure
ros2 service call /camera/set_color_auto_exposure std_srvs/srv/SetBool '{data: false}'
Set white balance
ros2 service call /camera/set_white_balance orbbec_camera_msgs/srv/SetInt32 '{data: 4600}'
Set laser enable
ros2 service call  /camera/set_laser_enable std_srvs/srv/SetBool "{data: true}"
toggle sensor
ros2 service call /camera/toggle_ir std_srvs/srv/SetBool "{data : true}"
save point cloud
ros2 service call /camera/save_point_cloud std_srvs/srv/Empty "{}"


## Check for Roboteq Driver
Note on Permissions (Outside Source Information): If you find the correct port but still get an error, it might be a permission issue. You may need to grant access to the dialout group using 
```bash
sudo usermod -a -G dialout $USER
```
and then restart your session.

## install joint_state_publisher
```bash
sudo apt-get install ros-jazzy-joint-state-publisher
```

## install rtab_map
```bash
sudo apt-get install ros-jazzy-rtabmap ros-jazzy-rtabmap-odom ros-jazzy-rtabmap-viz ros-jazzy-rtabmap-slam -y
```

## install Nav2
```bash
sudo apt-get install ros-jazzy-nav2-bringup -y
```

## install Resbridge Server and NPM for GUI server
```bash
sudo apt install ros-jazzy-rosbridge-server -y
```

### Install NPM, makes sure it's latest version to support vite.js
```bash
curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.7/install.sh | bash
nvm install node
```

### Run GUI Front-end
```bash
cd ~/polebot_amr_ws/src/polebot_amr/polebot_amr_webserver
npm install
npm run dev
```

### Install Database back-end
#### Install MariaDB for Backend
```bash
sudo apt install mariadb-server
sudo systemctl status mariadb
```

#### Install PHP my Admin for Backend
```bash
sudo apt install phpmyadmin
```
choose apache
choose yes for password config
input your PASSWORD

```bash
sudo mysql
CREATE USER 'admin'@'localhost' IDENTIFIED BY 'amr2025';
GRANT ALL PRIVILEGES ON *.* TO 'admin'@'localhost' WITH GRANT OPTION;
FLUSH PRIVILEGES;

CREATE USER 'amr'@'localhost' IDENTIFIED BY 'amr2025';
GRANT ALL PRIVILEGES ON *.* TO 'amr'@'localhost' WITH GRANT OPTION;
FLUSH PRIVILEGES;

CREATE DATABASE polebot;
exit

sudo mysql -u amr -p polebot < polebot.sql
```

### Run GUI Back-end
```bash
cd ~/polebot_amr_ws/src/polebot_amr/polebot_amr_webserver_backend
npm install
node index.js
```