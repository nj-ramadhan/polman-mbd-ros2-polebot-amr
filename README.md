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

*   **Configure Bash Session:**
    ```bash
    echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
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
