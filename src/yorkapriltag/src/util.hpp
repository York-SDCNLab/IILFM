//standard imports
#include <iostream>
#include <limits>
#include <float.h>
#include <math.h>
#include "yaml-cpp/yaml.h"
#include <algorithm>
#include <vector>
#include <iterator>
#include <cmath>
#include <sstream>

//ros imports
#include <ros/ros.h>
#include <ros/package.h>

//opencv imports
#include <cv_bridge/cv_bridge.h>
#include "image_transport/image_transport.h"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

//pcl imports
#include <pcl/common/common_headers.h>
#include <pcl/console/parse.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/png_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/common/float_image_utils.h>

using namespace std;

void pose_estimation_3d3d(const vector<cv::Point3f>&, const vector<cv::Point3f>&, cv::Mat&, cv::Mat&);
Eigen::Quaterniond rotation_to_quaternion(Eigen::Matrix3d);
