
# Positioning-system   🤖&🚗&🛰️

## Project Content
### 1.  **Efficient Object Detection Module**

-  Deploy a YOLOv8 detection model based on PyTorch, performing image preprocessing, model inference, and result parsing to obtain object bounding boxes. Integrate OpenCV text annotation to display target depth information.
    

### 2.  **Spatial Position Parsing Module**

-   Construct **a four-coordinate transformation model (world-camera-image plane-pixel)**, derive the analytical formula for the intrinsic matrix  **K**, and apply **median filtering** to reduce depth map noise. Compute target positions in the camera frame based on camera intrinsics, achieving stable centimeter-level object localization.
    

### 3.  **Robot System Engineering Implementation**

-   Implement image subscription and position publishing via ROS Topic, enabling seamless integration with upstream and downstream modules in the robotic system.


## Preparations before Running

- This project is fully implemented based on ROS. Therefore, you need to prepare a ROS environment in advance.
- Download the YOLOv8 Model
    
    -   Visit the Ultralytics YOLOv8 documentation:  
        🔗  [YOLOv8](https://docs.ultralytics.com/zh/models/yolov8/#performance-metrics)
        
    -   Download the  `yolov8l.pt`  pre-trained weights.
    - Default save path:`/home/qrobo/yolo/weights/yolov8l.pt`
    - If you want to use a different directory, modify the path in`src/detection/launch/detection.launch`

## Running and Results

### Running
- `source devel/setup.bash`: Open a new terminal and source the workspace

- `roslaunch src/sim.launch`: Start the Gazebo simulation and keyboard control node

- `source devel/setup.bash`: Open another terminal and source the workspace again

- `roslaunch src/bringup.launch`: Start the core functionality nodes

### Presentation

- Upon successful execution of the `sim.launch` file, the following Gazebo simulation environment will be displayed (You can use the keyboard node in this terminal to drive the car.):

![Gazebo Simulation](https://raw.githubusercontent.com/11penguins/Positioning-system/main/image/Gazebo.png)

- Upon successful execution of the `bringup.launch` file, the camera's first-person view feed will process and display real-time object detection results (Choose the Detection node and use the keyboard node to move).


![Detection Result](https://raw.githubusercontent.com/11penguins/Positioning-system/main/image/Detection.png)

- Launch the RViz window, select **"Orbit"** in the **Type** dropdown, and you will see detected objects relative to the car's position—marked by the purple spheres in the visualization.

![Detection Result](https://raw.githubusercontent.com/11penguins/Positioning-system/main/image/Position.png)


## Some Thoughts and Subsequent Possible Works
In this project, the performance of our detection and localization systems has not met expectations, presenting several challenges and difficulties. We have summarized the key issues as follows, and our subsequent work will focus on addressing these challenges for improvement.

### 1. Covariate Shift
Note that YOLO exhibits some performance degradation in simulated images—for instance, misclassifying a "wall" as a "dining table." Analysis reveals this stems from a **Covariate Shift** between the training dataset and the test environment (real-world conditions). To address this, we plan to employ **fine-tuning** with domain-specific data to enhance YOLO's accuracy for task-oriented imagery.

### 2. Object Tracking
YOLO provides only per-frame bounding boxes without leveraging inter-frame correlations. To establish continuity between detections in adjacent frames, algorithmic approaches such as **IoU matching**, **the Hungarian algorithm**, **Kalman filtering**, or **feature-based tracking (DeepSORT)** could be employed to associate bounding boxes with persistent objects.
