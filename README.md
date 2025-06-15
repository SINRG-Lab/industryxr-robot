# **IndustryXR - Robot**

This repository contains the source code for the Hiwonder robot. It provides an overview of the included packages and detailed instructions for running the project.  

---

## **Overview**

This project uses **ROS2 Foxy** for development. It includes several ROS2 packages and a separate workspace for camera-related code.  

---

## **Packages**

Here’s the list of packages currently included in the project:  

1. **`ros_tcp_endpoint`**  
   - Handles communication between the robot and external systems.  
   - Allows other systems to connect to the robot over TCP.  

2. **`sinrg_robot_sdk`**  
   - Manages all communication with the hardware.  
   - Creates necessary subscribers and publishers for the application.  

3. **`sinrg_launch`**  
   - Provides launch files for running multiple packages together.  

**Additional Note:**  
The camera-related code is located in the root folder of the system and is set up in a separate workspace named `cam_ws`.  

---

## **System Configuration**

- **ROS Distribution**: ROS2 Foxy  
- **Dependencies**: Ensure all ROS2 dependencies for the listed packages are installed.  
- **Networking**: Update the `ROS_IP` argument to the system's IP address for proper communication.  

---

## **Instructions**

Follow these steps to run the application:  

**Option 1:** Run packages individually 

### **1. Start the TCP Endpoint**  
This enables external systems to connect to the robot.  

```bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=<YOUR_SYSTEM_IP>
```

Example
```bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=10.188.58.100
```
### **2. Start the robot sdk**
```bash
ros2 run sinrg_robot_sdk robot_controller_manager
```

---

**Option 2:** Run all packages using the launch file

```bash
ros2 launch sinrg_servo_controller sinrg_servo.launch.py
```


### License

- The project is under the [MIT License](LICENSE)