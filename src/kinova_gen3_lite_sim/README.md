# 📋 Overview

**Quick Links:**
- [Data Generation for Vision Learning](#i--kinova-gen3-lite--simulated-data-generation-for-vision-based-learning)
- [Debugging Camera Plugins](#ii--debugging-camera-plugin-errors-in-gazebo-harmonic-ros-2-jazzy)
- [Camera & Bridge Integration Report](#iii--report-resolving-camera-plugin--bridge-issues-in-ros-2--ignition-gazebo)
- [Progress Report on Image Data Capture](#iv-progress-report-simulation--image-data-capture-in-ignition-gazebo-with-ros-2)

---

# I - 📘 Kinova Gen3 Lite – Simulated Data Generation for Vision-Based Learning

## 🧰 Objective
Use a **simulated Kinova Gen3 Lite robotic arm** in **Gazebo (via ros_gz_sim)** to generate **synthetic image data** from virtual sensors (cameras), for training vision-based robot policies.

---

## 1. ✅ Launch the Simulation

```bash
ros2 launch kinova_gen3_lite_sim sim_launch.py
```

This launches:
- The robot in a Gazebo world (`empty_world.sdf`)
- Basic plugins (e.g., state publisher)
- URDF from `robot_description`

---

## 2. 🛠️ Fix: Simulation Resource Path

To fix errors like:

> `Waiting messages on topic [robot_description]. Entity creation successful. But world has no model.`

You needed to export the model path for Gazebo to find the robot meshes and URDF:

```bash
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:~/ros2_ws/install/kortex_description/share
```

✅ **Add to `.bashrc`** for persistence.

---

## 3. 📷 Add Virtual Camera to Simulation

You extended the world file (`empty_world.sdf`) to include a **camera sensor**, using `<sensor>` and the correct Gazebo plugins (or Ignition equivalents).

However, Gazebo failed to load the camera plugin:

```
[Err] Failed to load system plugin [libgz-sim-camera-system.so]
```

This means the plugin or its path is missing.

---

## 4. 📦 Install Required ROS-Gazebo Bridge Packages

You ensured the bridge packages were installed:

```bash
sudo apt install ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-interfaces
```

✅ Rebuilt your workspace:

```bash
colcon build --symlink-install --packages-select kinova_gen3_lite_sim
```

---

## 5. 🔄 Bridge Camera Topic

You used `ros_gz_bridge` to bridge the Gazebo camera image topic:

```bash
ros2 run ros_gz_bridge parameter_bridge /camera/image@sensor_msgs/msg/Image[ignition.msgs.Image
```

This bridges Gazebo image data to ROS 2, under `/camera/image`.

---

## 6. 💾 Create Image Logging Node

You developed `image_saver`, a ROS 2 node that:
- Subscribes to `/camera/image`
- Saves received images to `/tmp/sim_images`

Initial error due to missing dependency:

```bash
ImportError: libfdk-aac.so.1: cannot open shared object file
```

✅ Fixed by installing:

```bash
sudo apt install libfdk-aac2 libfdk-aac-dev
```

---

## 7. 🤖 Automate Pose & Image Generation

You created `auto_data_collector`, which:
- Publishes random joint positions
- Waits for images and triggers `image_saver`

```bash
ros2 run kinova_gen3_lite_sim auto_data_collector
```

But encountered warnings:

```
Image not ready — skipping this round
```

📌 Root Cause: No image data was being published — the camera system plugin was not loaded successfully.

---

## 8. 🔍 Diagnosis of Camera Plugin Issue

Attempts made:
- Installed system plugin libraries: `libgazebo_ros_camera.so`, `libgz-sim-camera-system.so`
- Checked with `locate`, but no libraries were found.
- Verified plugin load errors in Gazebo logs.

```bash
locate libgazebo_ros_camera.so
locate libgz-sim-camera-system.so
```

No matches ⇒ plugin is likely **not installed** or Gazebo version mismatch.

---

## ✅ Next Steps

1. **Fix Camera Plugin**:
   - Install `gz-sim` camera system plugins (`libgz-sim-camera-system.so`)
   - Confirm correct Gazebo version (`Ignition Fortress`, `Gazebo Harmonic`, etc.)

2. **Verify Camera in World or Robot**:
   - Make sure the `<sensor>` tag is inside a valid `<link>`
   - Use Ignition’s supported camera plugin (e.g., `gz::sim::systems::CameraSensor`)

3. **Set Plugin Path** (if necessary):

```bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ros/jazzy/lib/
```

4. **Visual Debug**:
   - Open Gazebo client and check if the camera appears in the scene

---

## 🗂️ Files Used

- `sim_launch.py`: ROS 2 launch script
- `empty_world.sdf`: Custom world file with sensors
- `image_saver.py`: Image logging node
- `auto_data_collector.py`: Data generation controller

---

<br><br><br><br><br><br>


# II - 🛠️ Debugging Camera Plugin Errors in Gazebo Harmonic (ROS 2 Jazzy)

This guide documents the process of identifying and fixing plugin loading issues when using camera sensors with **Gazebo Harmonic (gz-sim 9)** and **ROS 2 Jazzy**.

---

## 🚨 Initial Error

After configuring the camera plugins in the SDF world file and launching the simulation using:

```bash
ros2 launch kinova_gen3_lite_sim sim_launch.py
```

Gazebo produced the following error:

```
[gz-2] [Err] [SystemLoader.cc:92] Failed to load system plugin [libgazebo_ros_camera.so] : Could not find shared library.
```

### 🔍 Cause

This happened because the SDF file was referencing a **Gazebo Classic** plugin (`libgazebo_ros_camera.so`) which is **not compatible with Gazebo Harmonic** (part of the Ignition/Gazebo Fortress+ line, now called "Gazebo").

---

## 🧪 Attempted Fixes

We tried to switch to Ignition-style plugins like:

```xml
<plugin name="gazebo_ros_camera" filename="ignition-gazebo-ros-camera-system">
```

However, this produced another error:

```
[gz-2] [Err] [Server.cc:86] Error parsing XML in file [...]: XMLElement name=plugin
```

This was due to:

* Incorrect or missing closing tags in the SDF.
* Plugin names that do **not match actual plugin class names**.

---

## 🔍 Discovering Available Plugins

We attempted to locate installed system plugins using:

```bash
find /usr/lib /usr/lib64 -name "*ros-gz-*-system.so"
```

Eventually, we discovered that the available plugins on the system were located at:

```
/usr/lib/x86_64-linux-gnu/gz-sim-9/plugins/
```

With files such as:

```
libgz-sim-logical-camera-system.so
libgz-sim-camera-video-recorder-system.so
```

But Gazebo still failed to load them, showing:

```
[gz-2] [Err] [SystemLoader.cc:164] Failed to load system plugin: (Reason: library does not contain requested plugin)
[gz-2] - Requested plugin name: [logical_camera_plugin]
[gz-2] - Requested library name: [libgz-sim-logical-camera-system.so]
[gz-2] - Detected Plugins:
[gz-2]   - gz::sim::v8::systems::LogicalCamera
```

---

## 🧩 Root Cause

The `<plugin>` tag in the SDF used an incorrect `name` attribute:

```xml
<plugin name="logical_camera_plugin" filename="libgz-sim-logical-camera-system.so">
```

Gazebo doesn’t load plugins by the `name` you choose — it requires the **actual exported class name** inside the `.so` file, which was printed in the error log:

```
gz::sim::v8::systems::LogicalCamera
```

---

## ✅ Final Working Configuration

Update your SDF file to use the correct plugin class name:

```xml
<plugin name="gz::sim::systems::LogicalCamera" filename="libgz-sim-logical-camera-system.so">
  <topic>/logical_camera</topic>
  <update_rate>30</update_rate>
</plugin>
```

Even though the plugin was built under gz-sim 9 (v9), Gazebo exports its class under the `v8` namespace — this is **normal and expected**.

---

## 🧠 Tips for Future Debugging

* Gazebo always prints **detected plugins** in its error message. Use these names exactly in your `<plugin>` tag.
* To inspect what a plugin library contains, use:

```bash
strings libgz-sim-logical-camera-system.so | grep systems
```

* Plugin `.so` filenames do **not** always match the class names — always refer to the Gazebo logs to confirm.
* System plugins live in:
  `/usr/lib/x86_64-linux-gnu/gz-sim-9/plugins/`

---


<br><br><br><br><br><br>


# III - 🧾 **Report: Resolving Camera Plugin & Bridge Issues in ROS 2 + Ignition Gazebo**

### 🎯 **Objective**

Enable RGBD camera simulation in Ignition Gazebo and bridge its output to ROS 2 so images can be accessed for data collection.

---

### 🔍 **Initial Issues**

* The **camera plugin (`libgz-sim-camera-system.so`)** failed to load in Gazebo.
* **Image data was not being published**, either in simulation or through ROS 2.
* World and robot failed to initialize correctly due to incomplete or incompatible SDF/plugin configuration.

---

### ✅ **Solution Approach**

#### 🧩 1. **Plugin System Composition**

We replaced the monolithic camera plugin with a **composite set of system plugins** to properly initialize simulation components. These included:

```xml
<plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"/>
<plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
<plugin filename="gz-sim-air-pressure-system" name="gz::sim::systems::AirPressure"/>
<plugin filename="gz-sim-altimeter-system" name="gz::sim::systems::Altimeter"/>
<plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
<plugin filename="gz-sim-magnetometer-system" name="gz::sim::systems::Magnetometer"/>
<plugin filename="gz-sim-forcetorque-system" name="gz::sim::systems::ForceTorque">
  <gz::system_priority>10</gz::system_priority>
</plugin>
<plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
<plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
```

✅ This resolved the **camera system load failure** and enabled sensor initialization.

---

#### 🔄 2. **Sensor & World Integration**

* Integrated a **standard RGBD camera model** from the GazeboSim GitHub repository into a custom world file.
* Ensured proper `<sensor>` tags and associated `<plugin>` tags were declared within the camera model SDF.
* Synced world names with service endpoints to avoid world load timing issues.

---

#### 🌉 3. **ROS 2 Bridge Setup**

* Used the `ros_gz_bridge` with a **parameter bridge**:

  ```bash
  ros2 run ros_gz_bridge parameter_bridge /camera/rgbd/image@sensor_msgs/msg/Image[ignition.msgs.Image
  ```
* Confirmed that image frames are now **correctly published** to a ROS 2 topic:
  `/camera/rgbd/image`

✅ The bridge successfully relays RGBD camera data from Ignition Gazebo to ROS 2.

---

### 📸 **Result**

* Every image frame from the simulated camera is now:

  * Published on a **Gazebo topic**
  * Bridged and available as a **ROS 2 topic**
* Camera visualization tools like `showimage` can access these images in real time.

---

### 🧭 **Next Step**

➡ **Subscribe to the ROS 2 topic** and **save each image frame to disk** using a custom ROS 2 node (`auto_data_collector`).

---


Here is a structured report summarizing the work and progress made during the simulation setup and image data collection in Ignition Gazebo with ROS 2:

---


<br><br><br><br><br><br>

# IV - **Progress Report: Simulation & Image Data Capture in Ignition Gazebo with ROS 2**

### **Objective**

Establish a robust Gazebo-based simulation for the Kinova Gen3 Lite robot with integrated RGBD camera, allowing automated data collection and robot movement control within the ROS 2 ecosystem.

---

### **1. Initial Challenges & Plugin Failures**

* Encountered **critical plugin load failure** for `libgz-sim-camera-system.so`, preventing RGBD camera initialization.
* Investigation revealed compatibility issues between **Gazebo Classic and Ignition Gazebo (gz-sim)**.
* Reviewed plugin paths, shared library availability, and model URIs to debug the issue.
* Adjusted the simulation world to use **system plugins** (`Sensors`, `Physics`, `ForceTorque`, etc.) to ensure proper camera support and sensor processing.

---

### **2. World File and Camera Model Fixes**

* Analyzed and merged two world files:

  * A minimal base world with logical camera setup.
  * A complete RGBD world model from **Gazebo Fuel**.
* Created a **custom SDF world** that integrates the RGBD camera.
* Resolved issues related to:

  * World name mismatches in the service namespace (`/world/.../create`).
  * Missing `<inertial>` tags and fixed joint mishandling in URDF → SDF conversion.

---

### **3. ROS-Gazebo Bridge Integration**

* Successfully launched the `ros_gz_bridge` node with the appropriate bridge configuration:

  ```bash
  /camera/rgbd/image@sensor_msgs/msg/Image[ignition.msgs.Image
  ```
* Confirmed image data publication from Gazebo to ROS 2 over `/camera/rgbd/image`.

---

### **4. Camera Image Calibration**

* Before saving data, camera **intrinsic parameters** (focal length, principal point, distortion coefficients) were reviewed and validated.
* The simulated camera model was configured with appropriate parameters in my **custom SDF world**, image dimensions, and near/far clipping range.

---

### **5. Image Collection Node Refactor**

#### **Issue**

* The original `auto_data_collector` node failed to subscribe correctly to the RGBD topic.

  * This was due to timing issues, incorrect topic remapping, and coupling of unrelated logic.

#### **Solution**

* **Split the node into two focused classes**:

  1. `ImageSaver` — handles image subscription and saving to disk.
  2. `AutoMotionExplorer` — handles robot movement and control.

---

### **6. ImageSaver Implementation**

* Implemented `ImageSaver` as a minimal `rclpy` node:

  * Subscribes to `/camera/rgbd/image`.
  * Converts incoming `sensor_msgs/Image` using OpenCV (`cv_bridge`).
  * Saves images with timestamps to a specified directory.
* Verified functionality by confirming disk writes of RGB images during simulation runtime.

---

### **7. Current Status**

| Component               | Status                                    |
| ----------------------- | ----------------------------------------- |
| Gazebo Simulation       | ✅ Working                                 |
| Kinova URDF Integration | ✅ Working                                 |
| RGBD Camera in World    | ✅ Working                                 |
| Sensor Plugins          | ✅ Working                                 |
| ros\_gz\_bridge         | ✅ Working                                 |
| Camera Calibration      | ✅ Verified                                |
| ImageSaver Node         | ✅ Saving images correctly                 |
| auto\_motion\_explorer  | 🛠️ To be integrated with motion planning |

---

### **Next Steps**

* Finalize and integrate `auto_motion_explorer` to automate robot movement.
* Sync image capture with robot pose data for dataset generation.
* Add depth image and TF recording for full scene annotation.
* Package and document launch/config setup for reproducibility.

---