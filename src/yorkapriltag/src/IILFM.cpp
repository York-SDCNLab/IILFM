#include "IILFM.hpp"

IILFM::IILFM(int argc, char** argv){
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");
	
	//setup ros parameters
	nh_private.param("lidar_topic", lidar_topic, string("/livox/lidar"));
	nh_private.param("integration_size", integration_size, 20);
	nh_private.param("angular_resolution_x_deg", angular_resolution_x_deg, 0.05f);
	nh_private.param("angular_resolution_y_deg", angular_resolution_y_deg, 0.05f);
	nh_private.param("max_angular_width_deg", max_angular_width_deg, 39.0f);
	nh_private.param("max_angular_height_deg", max_angular_height_deg, 39.0f);
	nh_private.param("image_threshold", image_threshold, 60.0);
	nh_private.param("add_blur", add_blur, false);

	//tag settings
	nh_private.param("tag_family", tag_family, string("tag36h11"));
	nh_private.param("tag_decimate", tag_decimate, 2.0);
	nh_private.param("tag_blur", tag_blur, 0.0);
	nh_private.param("tag_threads", tag_threads, 1);
	nh_private.param("tag_debug", tag_debug, false);
	nh_private.param("tag_refine_edges", tag_refine_edges, true);
	
	//general config
	ROS_INFO("Configuring Tag Settings");
	load_YAML();
	create_tag_detector();
	ROS_INFO("Configuration Complete");
	
	//create subscriber once config is complete
	//lidar_sub = nh.subscribe("/livox/lidar", 5, &IILFM::lidar_cb, this);
	lidar_sub = nh.subscribe(lidar_topic, 5, &IILFM::lidar_cb, this);
	iilfm_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/iilfm/pose", 10);
	iilfm_feature_pub = nh.advertise<sensor_msgs::PointCloud2>("/iilfm/features", 10);
	
	ROS_INFO("Subscribers and Publishers Created");
	
	ROS_INFO("Waiting for Integration Time to be Fulfilled");
}

void IILFM::lidar_cb(const sensor_msgs::PointCloud2 &cloud_msg){
	pcq.push(cloud_msg); //add pointcloud to queue
	
	//vector of valid points
	vector<std::vector<float>> valid_points;
	vector<float> valid_point;
	
	//Container for pointcloud
	pcl::PointCloud<pcl::PointXYZI> cloud;
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloudptr (new pcl::PointCloud<pcl::PointXYZI>);
	
	if(pcq.size() >= integration_size){
		//iterate through the queue and create a vector of valid points
		for(auto it=pcq.begin(); it != pcq.end(); ++it){
			pcl::fromROSMsg(*it, cloud);
			*cloudptr=cloud;
			
			for(size_t i = 0; i < cloudptr->points.size(); ++i){
				//ensure the point is not too close to the sensor in x
				if(cloudptr->points[i].x != 0){
					valid_point = {cloudptr->points[i].x, cloudptr->points[i].y, cloudptr->points[i].z, cloudptr->points[i].intensity};
					valid_points.push_back(valid_point);
				}
			}
			
		}
		
		detect_tag(valid_points);
		pcq.pop();
	}
}

void image_cb(const sensor_msgs::ImageConstPtr& msg){
	cv_bridge::CvImagePtr cv_ptr;
	
	try{
     cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
     ROS_ERROR("cv_bridge exception: %s", e.what());
     return;
    }
	
	cv::imshow("test", cv_ptr->image);
    cv::waitKey(3);
}

void IILFM::detect_tag(const std::vector<std::vector<float>> &points){
	ROS_INFO("Beginning Tag Detection");

	//create an empty pointcloud to fill witht the valid points, and valid points with intensity
	pcl::PointCloud<pcl::PointXYZ> valid_cloud;
	valid_cloud.width = points.size();
	valid_cloud.height = 1;
	valid_cloud.is_dense = false;
	valid_cloud.points.resize(valid_cloud.width * valid_cloud.height);
	
	pcl::PointCloud<pcl::PointXYZ> valid_cloud_i;
	valid_cloud_i.width = points.size();
	valid_cloud_i.height = 1;
	valid_cloud_i.is_dense = false;
	valid_cloud_i.points.resize(valid_cloud_i.width * valid_cloud_i.height);
	
	//write the valid points to the new pointcloud
	for(size_t p = 0; p < points.size(); ++p){
		valid_cloud.points[p].x = points[p][0];
		valid_cloud.points[p].y = points[p][1];
		valid_cloud.points[p].z = points[p][2];
		
		valid_cloud_i.points[p].x = (points[p][3] != 0) ? points[p][0] * points[p][3] : points[p][0];
		valid_cloud_i.points[p].y = (points[p][3] != 0) ? points[p][1] * points[p][3] : points[p][1];
		valid_cloud_i.points[p].z = (points[p][3] != 0) ? points[p][2] * points[p][3] : points[p][2];
	}
	
	//initialize the range images
	float noise_level = 0.0;
	float min_range = 0.0f;
	int border_size = 1;
	
	pcl::RangeImage::Ptr range_image_ptr(new pcl::RangeImage);
	pcl::RangeImage& range_image = *range_image_ptr;
	
	pcl::RangeImage::Ptr range_image_i_ptr(new pcl::RangeImage);
	pcl::RangeImage& range_image_i = *range_image_i_ptr;   
	
	float angular_resolution_x = (float) (angular_resolution_x_deg * (M_PI/180.0f));
	float angular_resolution_y = (float) (angular_resolution_y_deg * (M_PI/180.0f));
	float max_angular_width    = (float) (max_angular_width_deg * (M_PI/180.0f));
	float max_angular_height   = (float) (max_angular_height_deg * (M_PI/180.0f));
	Eigen::Affine3f sensor_pose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
	pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::LASER_FRAME;

	range_image.createFromPointCloud(valid_cloud, angular_resolution_x, angular_resolution_y, max_angular_width, 
									 max_angular_height, sensor_pose, coordinate_frame, noise_level, min_range, border_size);
	
	range_image_i.createFromPointCloud(valid_cloud_i, angular_resolution_x, angular_resolution_y, max_angular_width, 
									   max_angular_height, sensor_pose, coordinate_frame, noise_level, min_range, border_size);					 
		
	//normalize the intensity, otherwise we cannot transfer it into CV Mat
	float* ranges = range_image_i.getRangesArray(); //this will get the intensity values
	float max = 0;
	float val;
	for(int i = 0; i < range_image_i.width*range_image_i.height; ++i){
		val = *(ranges + i);

		if(val < -FLT_MAX || val > FLT_MAX){
			//std::cout << "is nan or inf" << std::endl;
		}
		else{
			max = (val > max) ? val : max;
		}
	}
	
	// Create cv::Mat
	Mat image(range_image_i.height, range_image_i.width, CV_8UC4);
	unsigned char r, g, b;

	// pcl::PointCloud to cv::Mat
	#pragma omp parallel for
	for(int y = 0; y < range_image_i.height; y++ ) {
		for(int x = 0; x < range_image_i.width; x++ ) {


			pcl::PointWithRange rangePt = range_image_i.getPoint(x, y);
			float value = rangePt.range / max;

			pcl::visualization::FloatImageUtils::getColorForFloat(value, r, g, b);

			image.at<cv::Vec4b>( y, x )[0] = b;
			image.at<cv::Vec4b>( y, x )[1] = g;
			image.at<cv::Vec4b>( y, x )[2] = r;
			image.at<cv::Vec4b>( y, x )[3] = 255;
		}
	}
	
	//create greyscale image for detections
	cv::Mat gray;
	cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
	cv::threshold(gray, gray, image_threshold, 255, cv::THRESH_BINARY);
	
	if(add_blur){
		cv::GaussianBlur(gray, gray, Size(3, 3), 25, 0, 4);
	}
	
	// Make an image_u8_t header for the Mat data
	image_u8_t im = { 
		.width = gray.cols,
		.height = gray.rows,
		.stride = gray.cols,
		.buf = gray.data
	};
	
	//detect the tag in the image
	zarray_t *detections = apriltag_detector_detect(td, &im);
	
	//go over the detections
	vector<cv::Point3f> pts1;
	vector<cv::Point3f> pts_tag;
	vector<float> PointL;
	vector<vector<float>> PointsL;
	bool do_pub = true;
	
	pcl::PointWithRange rangePt_ap;
	pcl::PointWithRange point_up;
	pcl::PointWithRange point_down;
	float ratio;
	vector<int> id_num;
	
	for(int i = 0; i < zarray_size(detections); i++){
		apriltag_detection_t *det;
		zarray_get(detections, i, &det);

		iter = find(tag_id.begin(), tag_id.end(),det->id);
		int index = iter - tag_id.begin(); //find the index of the given id in the tag_id vector
		
		
		id_num.push_back(det->id);
		pts_tag.push_back(pts0[4*index]);    
		pts_tag.push_back(pts0[4*index+1]);  
		pts_tag.push_back(pts0[4*index+2]);  
		pts_tag.push_back(pts0[4*index+3]);
		
		//go over the four vertices
		for(int j = 0; j<4; j++){
			//if the vertex is unobserved.
			if (!range_image.isObserved(round(det->p[j][0]), round(det->p[j][1]))){
				// check if there exists a pair of neighbor points that is symmetric to each other with respect to the unobserved point 
				for(int m =0; m < 3; m++){
					//fix the azimuth
					if(range_image.isObserved(round(det->p[j][0]), round(det->p[j][1]+m))){
						if(range_image.isObserved(round(det->p[j][0]), round(det->p[j][1]-m))){
							range_image.calculate3DPoint(round(det->p[j][0]), round(det->p[j][1]+m), point_up);
							range_image.calculate3DPoint(round(det->p[j][0]), round(det->p[j][1]-m), point_down);
							
							ratio = point_down.range/point_up.range;
							pts1.push_back(
								cv::Point3f(
									point_up.x*(1/(1+ratio))+point_down.x*(ratio/(1+ratio)), 
									point_up.y*(1/(1+ratio))+point_down.y*(ratio/(1+ratio)), 
									point_up.z*(1/(1+ratio))+point_down.z*(ratio/(1+ratio))
								)
							);
									
							break;
						}
					}
					
					if(m == 2){
						do_pub = false;
					}
				}
			}
			//if the vertex is observed
			else{
				range_image.calculate3DPoint(round(det->p[j][0]), round(det->p[j][1]), rangePt_ap);
				PointL = {rangePt_ap.x, rangePt_ap.y, rangePt_ap.z};               
				PointsL.push_back(PointL);
				
				pts1.push_back(cv::Point3f(rangePt_ap.x, rangePt_ap.y, rangePt_ap.z));
			}
		}
	}
	cout <<"The number of detected marker:"<<id_num.size()<<". "<<" ID number(s):"<< endl;
	copy(id_num.begin(), id_num.end(), ostream_iterator<int>(cout, "; "));
    	cout << endl;
	cv::Mat R(3, 3, CV_32FC1);
	cv::Mat R_inv(3, 3, CV_32FC1);
	cv::Mat t(3, 1, CV_32FC1);
	cv::Mat t_inv(3, 1, CV_32FC1);
	Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>> eigen_R(R_inv.ptr<double>(), R_inv.rows, R_inv.cols);
	
	//if a detection is made, determine the pose
	if(!pts1.empty() && do_pub){
		publish_tag_cloud(pts1);
		

		
		pose_estimation_3d3d(pts1, pts_tag, R, t);
		R_inv = -R.t();
		t_inv = -R.t() * t;
		
		//cout << R_inv.rowRange(0, 3) << endl;
		
		pose_rotation = rotation_to_quaternion(eigen_R);
		
		//create the ros topics to publish
		pose_stamped.header.frame_id = "livox_frame";
		pose_stamped.header.stamp = ros::Time::now();
		
		pose_stamped.pose.position.x = t_inv.at<double>(0,0);
		pose_stamped.pose.position.y = t_inv.at<double>(0,1);
		pose_stamped.pose.position.z = t_inv.at<double>(0,2);
		
		pose_stamped.pose.orientation.x = pose_rotation.x();
		pose_stamped.pose.orientation.y = pose_rotation.y();
		pose_stamped.pose.orientation.z = pose_rotation.z();
		pose_stamped.pose.orientation.w = pose_rotation.w();
		
		//publish the pose
		iilfm_pose_pub.publish(pose_stamped);
	}
	
	ROS_INFO("Finished Tag Detection");
}

void IILFM::publish_tag_cloud(vector<cv::Point3f> feature_points){
	sensor_msgs::PointCloud2 pc_out;
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr features(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ> &feature_cloud = *features;
    pcl::PCLPointCloud2 pc2;

	feature_cloud.width = feature_points.size();
	feature_cloud.height = 1;
	feature_cloud.is_dense = false;
	feature_cloud.points.resize(feature_cloud.width * feature_cloud.height);

	for(size_t p = 0; p < feature_points.size(); ++p){
		pcl::PointXYZ point;  
		feature_cloud.points[p].x = feature_points[p].x;  
		feature_cloud.points[p].y = feature_points[p].y;  
		feature_cloud.points[p].z = feature_points[p].z;   
		//point.intensity = int(255);
	}
	
	pcl::toPCLPointCloud2(feature_cloud, pc2);
	pcl_conversions::fromPCL(pc2, pc_out);
	
	pc_out.header.frame_id = "livox_frame";
	pc_out.header.stamp = ros::Time::now();
	
	iilfm_feature_pub.publish(pc_out);
}

void IILFM::load_YAML(){
	string package_path = ros::package::getPath("yorkapriltag");
	conf = YAML::LoadFile(package_path + "/resources/config.yaml");
	
	for(YAML::const_iterator it=conf.begin(); it!=conf.end(); ++it){
		// const std::string &key=it->first.as<std::string>();
		
		YAML::Node attributes = it->second;
		YAML::Node tagid = attributes["id"];

		tag_id.push_back(tagid.as<int>());

		YAML::Node firstv = attributes["firstvertex"];
		YAML::Node secondv = attributes["secondvertex"];
		YAML::Node thirdv = attributes["thirdvertex"];
		YAML::Node fourthv = attributes["fourthvertex"];
		pts0.push_back ( cv::Point3f ( firstv[0].as<float>(),firstv[1].as<float>(),firstv[2].as<float>() ) );       
		pts0.push_back ( cv::Point3f ( secondv[0].as<float>(),secondv[1].as<float>(),secondv[2].as<float>() ) );       
		pts0.push_back ( cv::Point3f ( thirdv[0].as<float>(),thirdv[1].as<float>(),thirdv[2].as<float>() ) );       
		pts0.push_back ( cv::Point3f ( fourthv[0].as<float>(),fourthv[1].as<float>(),fourthv[2].as<float>() ) );
	}
}

/*void IILFM::parse_options(int argc, char** argv){
	getopt_t *getopt = getopt_create();

	getopt_add_bool(getopt, 'h', "help", 0, "Show this help");
	getopt_add_bool(getopt, 'd', "debug", 0, "Enable debugging output (slow)");
	getopt_add_bool(getopt, 'q', "quiet", 0, "Reduce output");
	getopt_add_string(getopt, 'f', "family", "tag36h11", "Tag family to use");
	getopt_add_int(getopt, 't', "threads", "1", "Use this many CPU threads");
	getopt_add_double(getopt, 'x', "decimate", "2.0", "Decimate input image by this factor");
	getopt_add_double(getopt, 'b', "blur", "0.0", "Apply low-pass blur to input");
	getopt_add_bool(getopt, '0', "refine-edges", 1, "Spend more time trying to align edges of tags");

	if (!getopt_parse(getopt, argc, argv, 1) || getopt_get_bool(getopt, "help")){
		printf("Usage: %s [options]\n", argv[0]);
		getopt_do_usage(getopt);
		exit(0);
	}
	
	//parse options for tag family and tag detector args
	famname = getopt_get_string(getopt, "family");
	td_decimate = getopt_get_double(getopt, "decimate");
	td_blur = getopt_get_double(getopt, "blur");
	td_threads = getopt_get_int(getopt, "threads");
	td_debug = getopt_get_bool(getopt, "debug");
	td_refine_edges = getopt_get_bool(getopt, "refine-edges");
}*/

void IILFM::create_tag_detector(){
	//check if provided tag family name is valid
	if (!strcmp(tag_family.c_str(), "tag36h11")) {
		tf = tag36h11_create();
	} else if (!strcmp(tag_family.c_str(), "tag25h9")) {
		tf = tag25h9_create();
	} else if (!strcmp(tag_family.c_str(), "tag16h5")) {
		tf = tag16h5_create();
	} else if (!strcmp(tag_family.c_str(), "tagCircle21h7")) {
		tf = tagCircle21h7_create();
	} else if (!strcmp(tag_family.c_str(), "tagCircle49h12")) {
		tf = tagCircle49h12_create();
	} else if (!strcmp(tag_family.c_str(), "tagStandard41h12")) {
		tf = tagStandard41h12_create();
	} else if (!strcmp(tag_family.c_str(), "tagStandard52h13")) {
		tf = tagStandard52h13_create();
	} else if (!strcmp(tag_family.c_str(), "tagCustom48h12")) {
		tf = tagCustom48h12_create();
	} else {
		printf("Unrecognized tag family name. Use e.g. \"tag36h11\".\n");
		exit(-1);
	}


	td = apriltag_detector_create();
	//tf = tag16h5_create();
	apriltag_detector_add_family(td, tf);
	
	td->quad_decimate = tag_decimate;
	td->quad_sigma = tag_blur;
	td->nthreads = tag_threads;
	td->debug = tag_debug;
	td->refine_edges = tag_refine_edges;
}

IILFM::~IILFM(){
}
