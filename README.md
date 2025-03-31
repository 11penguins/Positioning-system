
# Positioning-system   🤖&🚗&🛰️

## Project Content
### 1.  **Efficient Object Detection Module**

-  Deploy a YOLOv8 detection model based on PyTorch, performing image preprocessing, model inference, and result parsing to obtain object bounding boxes. Integrate OpenCV text annotation to display target depth information.
    

### 2.  **Spatial Position Parsing Module**

-   Construct **a four-coordinate transformation model (world-camera-image plane-pixel)**, derive the analytical formula for the **intrinsic matrix  K**, and apply **median filtering** to reduce depth map noise. Compute target positions in the camera frame based on camera intrinsics, achieving stable centimeter-level object localization.
    

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

## A Four-Coordinate Transformation Model


###  1. World Coordinate System ——> Camera Coordinate System

To transform from one coordinate system to another, only a single **translation + rotation** is needed—essentially, a homogeneous transformation. Specifically:

$$ 
\begin{bmatrix}
X_c \\
Y_c \\
Z_c \\
1 
\end{bmatrix} = 
\begin{bmatrix}
R & t \\
0 & 1
\end{bmatrix}
\begin{bmatrix}
X_w \\
Y_w \\
Z_w \\
1
\end{bmatrix}
$$
The table below uses **c** to denote points in the **camera coordinate system**, while **w** represents in the **world coordinate system**.

###  2. Camera Coordinate System ——> Image Plane Coordinate System
<div style="text-align: center;">
  <img src="https://raw.githubusercontent.com/11penguins/Positioning-system/main/image/CameratoPlane.png" 
       alt="Detection Result" 
       width="400">
</div>

As illustrated in the figure, the transformation from the **camera coordinate system** to the **image plane coordinate system** can be derived using the geometric principle of **similar triangles**, with calculations based on **the camera's focal length**. The specific formulation is as follows:
$$
\frac{Z_{C}}{f} = \frac{X_{C}}{X_{I}} = \frac{Y_{C}}{Y_{I}}
$$

The table below uses **I** to denote points in the **image plane coordinate system**.


###  3. Image Plane Coordinate System ——> Pixel Coordinate System
<div style="text-align: center;">
  <img src="https://raw.githubusercontent.com/11penguins/Positioning-system/main/image/PlanetoPixel.png" 
       alt="Detection Result" 
       width="400">
</div>

Typically, the origin of the pixel coordinate system is located at the top-left corner with units in pixels. Both the pixel coordinate system and the image plane coordinate system lie on the imaging plane, differing only in their origins and units of measurement. The coordinate transformation between these two systems is calculated as follows:

$$
\begin{aligned}
u &= \frac{X_{I}}{d_{x}} + c_{x} \\
v &= \frac{Y_{I}}{d_{y}} + c_{y}
\end{aligned}
$$

Here, `dₓ` and `d_y` represent the actual length and height of a pixel, which are typically known parameters of a camera. `cₓ` and `cᵧ` are the translation amounts of the image plane coordinate system origin (i.e., the optical center).

###  4. Integrated Transformation Pipeline
Based on the above derivation, we can establish the relationship between the coordinates of a point in the image plane coordinate system and its coordinates in the camera coordinate system.

$$
\begin{aligned}
u &= \frac{f X_C}{d_x Z_C} + c_x = f_x \frac{X_C}{Z_C} + c_x \\
v &= \frac{f Y_C}{d_y Z_C} + c_y = f_y \frac{Y_C}{Z_C} + c_y
\end{aligned}
$$

By converting the above equations into matrix form, we obtain:

$$
\begin{bmatrix} 
u \\ 
v \\ 
1 
\end{bmatrix}=
\frac{1}{Z_C}
\begin{bmatrix}
\frac{1}{d_x} & 0 & c_x \\
0 & \frac{1}{d_y} & c_y \\
0 & 0 & 1
\end{bmatrix}
\begin{bmatrix}
f & 0 & 0 \\
0 & f & 0 \\
0 & 0 & 1
\end{bmatrix}
\begin{bmatrix}
X_C \\ 
Y_C \\ 
Z_C 
\end{bmatrix}=
\frac{1}{Z_C}
\begin{bmatrix}
f_x & 0 & c_x \\
0 & f_y & c_y \\
0 & 0 & 1
\end{bmatrix}
\begin{bmatrix}
X_C \\ 
Y_C \\ 
Z_C 
\end{bmatrix}=
\frac{1}{Z_C} \mathbf{K} \mathbf{P}_C 
$$
The matrix **K** is referred to as the **intrinsic matrix** because its parameters are solely determined by the camera's internal parameters. Therefore, we derive the coordinate transformation formula from the camera coordinate system to the object coordinates as:
$$
\begin{aligned}
\mathbf{P}_C = Z_C \mathbf{K}^{-1} \begin{bmatrix} 
u \\ 
v \\ 
1 
\end{bmatrix}
\end{aligned}
$$

## Some Thoughts and Subsequent Possible Works
In this project, the performance of our detection and localization systems has not met expectations, presenting several challenges and difficulties. We have summarized the key issues as follows, and our subsequent work will focus on addressing these challenges for improvement.

### 1. Covariate Shift
Note that YOLO exhibits some performance degradation in simulated images—for instance, misclassifying a "wall" as a "dining table." Analysis reveals this stems from a **Covariate Shift** between the training dataset and the test environment (real-world conditions). To address this, we plan to employ **fine-tuning** with domain-specific data to enhance YOLO's accuracy for task-oriented imagery.

### 2. Object Tracking
YOLO provides only per-frame bounding boxes without leveraging inter-frame correlations. To establish continuity between detections in adjacent frames, algorithmic approaches such as **IoU matching**, **the Hungarian algorithm**, **Kalman filtering**, or **feature-based tracking (DeepSORT)** could be employed to associate bounding boxes with persistent objects.
