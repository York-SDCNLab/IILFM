# Intensity Image-based LiDAR Fiducial Marker System
Extensive research has been carried out on the Visual Fiducial Marker (VFM) system. However, no single study utilizes this system to its fullest potential in LiDAR applications. In this paper, we develop an Intensity Image-based LiDAR Fiducial Marker (IILFM) system which fills the above-mentioned gap. The proposed system only requires an unstructured point cloud with intensity as the input and it outputs the detected markers' information and the 6-DOF pose that describes the transmission from the world coordinate system to the LiDAR coordinate system. The use of the IIFLM system is as convenient as the conventional VFM systems with no restrictions on marker placement and shape. Different VFM systems, such as Apriltag, ArUco, CCTag, can be easily embedded into the system. Hence, the proposed system inherits the functionality of the VFM systems, such as the coding and decoding methods.<br>
<img width="600" height="600" src="https://user-images.githubusercontent.com/58899542/151822834-e7758e70-849f-483d-b2fd-df93b1fe0aa5.png"/> <br>




## Requirements
PCL 1.3 <br>
OpenCV <br>
catkin <br>
yaml-cpp <br>
Boost <br>

## Command
```git clone https://github.com/York-SDCNLab/IILFM.git```<br>
```cd IILFM```<br>
```catkin build```<br>
Modify the ```yorktag.launch``` in ~/IILFM/src/yorkapriltag/launch according to your LiDAR model and the employed tag family. Then modify the ``config.yaml`` in ~/IILFM/src/yorkapriltag/resources based on your setup. Otherwise, the outputted pose is meaningless. Afterward, run <br>
```source ./devel/setup.bash```<br>
```roslaunch yorkapriltag yorktag.launch```<br>
Open a new terminal in ~/IILFM/src/yorkapriltag/resources and run <br>
```rosbag play -l bagname.bag```<br>
<br>
To view the 6-DOF pose, open a new terminal and run<br>
``rostopic echo /iilfm/pose`` <br>
<br>
To view the point could of the detected 3D fiducials in rviz, open a new terminal and run ``rviz``. In rviz, change the 'Fixed Frame' to 'livox_frame'. ``add/ By topic/ iilfm/ features/ PointCloud2``
