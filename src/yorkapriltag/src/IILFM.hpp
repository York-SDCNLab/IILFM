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

//topic imports
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/PoseStamped.h>

//pcl imports
#include <pcl/common/common_headers.h>
#include <pcl/console/parse.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/png_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/common/float_image_utils.h>

//utility functions
#include "util.hpp"

extern "C" {
	#include "apriltag.h"
	#include "tag36h11.h"
	#include "tag25h9.h"
	#include "tag16h5.h"
	#include "tagCircle21h7.h"
	#include "tagCircle49h12.h"
	#include "tagCustom48h12.h"
	#include "tagStandard41h12.h"
	#include "tagStandard52h13.h"
	#include "common/getopt.h"
}

typedef pcl::PointXYZ PointType;
using namespace std;
using namespace cv;

//c++ queue for holding pointclouds based on integration size
template<typename T, typename Container=std::deque<T>>
class iterable_queue : public std::queue<T, Container>{
public:
    typedef typename Container::iterator iterator;
    typedef typename Container::const_iterator const_iterator;

    iterator begin() { return this->c.begin(); }
    iterator end() { return this->c.end(); }
    const_iterator begin() const { return this->c.begin(); }
    const_iterator end() const { return this->c.end(); }
};

class IILFM{
	public:
		IILFM(int argc, char** argv);
		~IILFM();
		
		void lidar_cb(const sensor_msgs::PointCloud2&);
		void image_cb(const sensor_msgs::ImageConstPtr&);
		void load_YAML();
		//void pose_estimation_3d3d(const vector<cv::Point3f>&, const vector<cv::Point3f>&, cv::Mat&, cv::Mat&);
		//void parse_options(string);
		void create_tag_detector();
		void detect_tag(const std::vector<std::vector<float>>&);
		void publish_tag_cloud(vector<cv::Point3f>);
	
	private:
		//ros publishers and subscribers
		ros::Subscriber lidar_sub;
		ros::Publisher  iilfm_pose_pub;
		ros::Publisher  iilfm_feature_pub;
	
		//pcl vars
		pcl::RangeImage::CoordinateFrame coordinate_frame;
		
		//for yaml setup
		YAML::Node conf;
		
		//yorktag variables
		iterable_queue<sensor_msgs::PointCloud2> pcq; //pointcloud queue
		vector<cv::Point3f> pts0;
		vector<int> tag_id;
		vector<int>::iterator iter;
		geometry_msgs::PoseStamped pose_stamped;
		Eigen::Quaterniond pose_rotation;
		
		//tag detector and family parameters
		const char* famname; //tag family name
		apriltag_family_t* tf;
		apriltag_detector_t* td;
		
		double td_decimate;
		double td_blur;
		int    td_threads;
		bool   td_debug;
		bool   td_refine_edges;
		
		
		//this is where we define the ros parameters
		string lidar_topic;
		string tag_family;
		double tag_decimate;
		double tag_blur;
		int tag_threads;
		bool tag_debug;
		bool tag_refine_edges;

		int    integration_size;
		float  angular_resolution_x_deg;
		float  angular_resolution_y_deg;
		float  max_angular_width_deg;
		float  max_angular_height_deg;
		double image_threshold;
		bool   add_blur;
};
