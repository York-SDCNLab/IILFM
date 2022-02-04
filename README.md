# Intensity Image-based LiDAR Fiducial Marker System
Extensive research has been carried out on the Visual Fiducial Marker (VFM) system. However, no single study utilizes this system to its fullest potential in LiDAR applications. In this work, we develop an Intensity Image-based LiDAR Fiducial Marker (IILFM) system which fills the above-mentioned gap. The proposed system only requires an unstructured point cloud with intensity as the input and it outputs the detected markers' information and the 6-DOF pose that describes the transmission from the world coordinate system to the LiDAR coordinate system. An introduction video is available at: https://www.youtube.com/watch?v=AYBQHAEWBLM. The use of the IIFLM system is as convenient as the conventional VFM systems with no restrictions on marker placement and shape. Different VFM systems, such as Apriltag, ArUco, CCTag, can be easily embedded into the system. Hence, the proposed system inherits the functionality of the VFM systems, such as the coding and decoding methods.<br>
<img width="400" height="400" src="https://user-images.githubusercontent.com/58899542/151822834-e7758e70-849f-483d-b2fd-df93b1fe0aa5.png"/> <br>
## Marker Detection Demos
One and Two markers detection:
![demo1](https://user-images.githubusercontent.com/58899542/151841293-f2f4f2d0-f5ba-427e-b5e7-ff6106e4a8d0.gif)
Apriltag grid (35 markers) detection:<br>
<img width="480" height="320" src="https://user-images.githubusercontent.com/58899542/152581823-ca10f8db-8d3e-4025-91e9-eb111489b911.jpeg"/> <br>
![demo2](https://user-images.githubusercontent.com/58899542/152580373-71096105-8b6a-47ba-a852-767922dcf39a.gif)
![demo3](https://user-images.githubusercontent.com/58899542/152580126-5306eb2e-7899-494a-a7bd-bb0f43427daa.gif)
## LiDAR Pose Estimation Demo
![demo4](https://user-images.githubusercontent.com/58899542/152581365-ff25f9c3-3fd2-4a1d-9525-2383717266b3.gif)
Detailed qualitative and quantitative evaluations are provided in our paper.
## Other Applications
The proposed system shows potential in augmented reality, SLAM, multisensor calbartion, etc. Here, an augumented reality demo using the proposed system is presented. The teapot point cloud is transmitted to the location of the marker in the LiDAR point cloud based on the pose provided by the IILFM system. <br>
![demo5](https://user-images.githubusercontent.com/58899542/152583787-add4a9f2-59c6-4e15-a112-f1d2ad10324e.gif)




## Requirements
PCL 1.3 <br>
OpenCV <br>
catkin <br>
yaml-cpp <br>
Boost <br>

## Commands
```git clone https://github.com/York-SDCNLab/IILFM.git```<br>
```cd IILFM```<br>
```catkin build```<br>
Modify the ```yorktag.launch``` in ~/IILFM/src/yorkapriltag/launch according to your LiDAR model (e.g. rostopic, angular resolutions, and so on) and the employed tag family. Then modify the ``config.yaml`` in ~/IILFM/src/yorkapriltag/resources based on your setup. Otherwise, the outputted pose is meaningless. Afterward, run <br>
```source ./devel/setup.bash```<br>
```roslaunch yorkapriltag yorktag.launch```<br>
Open a new terminal in ~/IILFM/src/yorkapriltag/resources and run <br>
```rosbag play -l bagname.bag```<br>
<br>
To view the 6-DOF pose, open a new terminal and run<br>
``rostopic echo /iilfm/pose`` <br>
<br>
To view the point could of the detected 3D fiducials in rviz, open a new terminal and run ``rviz``. In rviz, change the 'Fixed Frame' to 'livox_frame'. ``add/ By topic/ iilfm/ features/ PointCloud2``
