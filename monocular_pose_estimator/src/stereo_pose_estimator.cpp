// This file is part of RPG-MPE - the RPG Monocular Pose Estimator
//
// RPG-MPE is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// RPG-MPE is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with RPG-MPE.  If not, see <http://www.gnu.org/licenses/>.

/*
 * monocular_pose_estimator.cpp
 *
 * Created on: Jul 29, 2013
 * Author: Karl Schwabe
 */

/** \file monocular_pose_estimator_node.cpp
 * \brief File containing the main function of the package
 *
 * This file is responsible for the flow of the program.
 *
 */

#include "monocular_pose_estimator/stereo_pose_estimator.h"
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <tf_conversions/tf_eigen.h>


namespace monocular_pose_estimator
{

/**
 * Constructor of the Stereo Pose Estimation Node class
 *
 */
SPENode::SPENode(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
  : nh_(nh), nh_private_(nh_private), have_camera_info_(false), rgb_have_camera_info_(false), right_ir_have_camera_info_(false), tfs_requested_(false), busy_(false),
	ir_sub_(nh_, "/camera/image_raw",1), rgb_sub_(nh_, "/camera/image_rgb",1),right_ir_sub_(nh_, "/camera/image_right_ir",1),
	sync_three_(SyncPolicyThree(10), ir_sub_, right_ir_sub_, rgb_sub_)
{
  // Set up a dynamic reconfigure server.
  // This should be done before reading parameter server values.
  dynamic_reconfigure::Server<monocular_pose_estimator::MonocularPoseEstimatorConfig>::CallbackType cb_;
  cb_ = boost::bind(&SPENode::dynamicParametersCallback, this, _1, _2);
  dr_server_.setCallback(cb_);

  // Initialize subscribers
  sync_three_.registerCallback(&SPENode::sync_callback_rgb_stereo_ir, this);


  camera_info_sub_ = nh_.subscribe("/camera/camera_info", 1, &SPENode::leftIRCameraInfoCallback, this);
  rgb_camera_info_sub_ = nh_.subscribe("/camera/image_rgb_camera_info", 1, &SPENode::rgbCameraInfoCallback, this);
  right_ir_camera_info_sub_ = nh_.subscribe("/camera/image_right_ir_camera_info", 1, &SPENode::rightIRCameraInfoCallback, this);




  // Initialize pose publisher
  pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("estimated_pose", 1);

  // Initialize image publisher for visualization
  image_transport::ImageTransport image_transport(nh_);
  image_pub_ = image_transport.advertise("image_with_detections", 1);
  rgb_image_pub_ = image_transport.advertise("rgb_image_with_detections", 1);

  // Create the marker positions from the test points
  List4DPoints positions_of_markers_on_object;

  //initialize the marker publisher
  vis_pub_ = nh_.advertise<visualization_msgs::Marker>( "LEDs", 5 );

  // Read the object mesh file
  if (nh_private_.getParam("object_mesh", mesh_path_))
  {
	ROS_INFO("Got param: %s", mesh_path_.c_str());
  }
  else
  {
	ROS_ERROR("Failed to get param 'object_mesh'");
  }


  // Read in the marker positions from the YAML parameter file
  XmlRpc::XmlRpcValue points_list;
  if (!nh_private_.getParam("marker_positions", points_list))
  {
    ROS_ERROR(
        "%s: No reference file containing the marker positions, or the file is improperly formatted. Use the 'marker_positions_file' parameter in the launch file.",
        ros::this_node::getName().c_str());
    ros::shutdown();
  }
  else
  {
    positions_of_markers_on_object.resize(points_list.size());
    for (int i = 0; i < points_list.size(); i++)
    {
      Eigen::Matrix<double, 4, 1> temp_point;
      temp_point(0) = points_list[i]["x"];
      temp_point(1) = points_list[i]["y"];
      temp_point(2) = points_list[i]["z"];
      temp_point(3) = 1;
      positions_of_markers_on_object(i) = temp_point;
    }
  }
  trackable_object_.setMarkerPositions(positions_of_markers_on_object);
  ROS_INFO("The number of markers on the object are: %d", (int )positions_of_markers_on_object.size());

}

/**
 * Destructor of the Monocular Pose Estimation Node class
 *
 */
SPENode::~SPENode()
{

}

void SPENode::requestCameraTFs(){
	//request the tf between IR and RGB optical frames
	tf::StampedTransform transform;
	try{

		bool rc = false;
		while( !have_camera_info_ && !rgb_have_camera_info_){
			ros::Duration(0.03).sleep();
		}

		ROS_INFO_STREAM("requesting TF: "<<rgb_cam_info_.header.frame_id<<" - "<<cam_info_.header.frame_id);
		tf_listener_.lookupTransform(rgb_cam_info_.header.frame_id, cam_info_.header.frame_id,
							   ros::Time(0), transform);
//	  if(!rc){
//		  ROS_ERROR_STREAM("tf not available: "<<rgb_cam_info_.header.frame_id<<" "<<cam_info_.header.frame_id);
//		  ros::shutdown();
//		  exit(0);
//	  }
	  Eigen::Affine3d affine3D;
	  tf::transformTFToEigen(tf::Transform(transform),affine3D);
	  rgb_T_ir = affine3D.matrix();
	  std::cout <<" requested TF rgb_T_ir="<<std::endl<<rgb_T_ir<<std::endl;
	}
	catch (tf::TransformException &ex) {
	  ROS_ERROR("%s",ex.what());
	  ros::Duration(1.0).sleep();
	}
}

void SPENode::sync_callback_rgb_stereo_ir(const sensor_msgs::Image::ConstPtr& ir_image_msg,const sensor_msgs::Image::ConstPtr& ir_right_image_msg, const sensor_msgs::Image::ConstPtr& rgb_image_msg){

	List2DPoints detected_led_positions, detected_led_positions2;
	List4DPoints detected_LEDs;
	double time_to_predict = ir_image_msg->header.stamp.toSec();
	ROS_INFO("SPENode::sync_callback_rgb_stereo_ir");

	// Check whether already received the camera calibration data
	if (!have_camera_info_ || !rgb_have_camera_info_|| !right_ir_have_camera_info_)
	{
		ROS_WARN("No camera infos yet...");
		return;
	}
	if(!tfs_requested_){
		  ROS_INFO("requesting TFs...");
		  requestCameraTFs();
		  tfs_requested_=true;
	  }

	// Import the image from ROS message to OpenCV mat
	cv_bridge::CvImagePtr cv_ptr, cv_rgb_ptr, cv_right_ir_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(ir_image_msg/*, sensor_msgs::image_encodings::MONO8*/);
		cv_rgb_ptr = cv_bridge::toCvCopy(rgb_image_msg, sensor_msgs::image_encodings::BGR8);
		cv_right_ir_ptr= cv_bridge::toCvCopy(ir_right_image_msg);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	cv::Mat ir = cv_ptr->image;
	cv::Mat right_ir = cv_right_ir_ptr->image;
	cv::Mat rgb = cv_rgb_ptr->image;

	cv::imshow("LEFT IR",ir);
	cv::imshow("RIGHT IR",right_ir);
	cv::imshow("RGB",rgb);
	trackable_object_.estimateFromStereo(ir, right_ir, time_to_predict,detected_led_positions, detected_led_positions2,detected_LEDs);
	publishLEDs(detected_LEDs);
	//cv::waitKey(0);
}




/**
 * The callback function that retrieves the camera calibration information
 *
 * \param msg the ROS message containing the camera calibration information
 *
 */
void SPENode::leftIRCameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
  if (!have_camera_info_)
  {
    cam_info_ = *msg;

    // Calibrated camera
    trackable_object_.camera_matrix_K_ = cv::Mat(3, 3, CV_64F);
    trackable_object_.camera_matrix_K_.at<double>(0, 0) = cam_info_.K[0];
    trackable_object_.camera_matrix_K_.at<double>(0, 1) = cam_info_.K[1];
    trackable_object_.camera_matrix_K_.at<double>(0, 2) = cam_info_.K[2];
    trackable_object_.camera_matrix_K_.at<double>(1, 0) = cam_info_.K[3];
    trackable_object_.camera_matrix_K_.at<double>(1, 1) = cam_info_.K[4];
    trackable_object_.camera_matrix_K_.at<double>(1, 2) = cam_info_.K[5];
    trackable_object_.camera_matrix_K_.at<double>(2, 0) = cam_info_.K[6];
    trackable_object_.camera_matrix_K_.at<double>(2, 1) = cam_info_.K[7];
    trackable_object_.camera_matrix_K_.at<double>(2, 2) = cam_info_.K[8];
    trackable_object_.camera_distortion_coeffs_ = cam_info_.D;

    have_camera_info_ = true;
    ROS_INFO("IR Camera calibration information obtained.");
  }

}


void SPENode::rightIRCameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg){


	if (!right_ir_have_camera_info_)
	{
		right_ir_cam_info_ = *msg;

		// Calibrated camera
		trackable_object_.right_ir_camera_matrix_K_ = cv::Mat(3, 3, CV_64F);
		trackable_object_.right_ir_camera_matrix_K_.at<double>(0, 0) = right_ir_cam_info_.K[0];
		trackable_object_.right_ir_camera_matrix_K_.at<double>(0, 1) = right_ir_cam_info_.K[1];
		trackable_object_.right_ir_camera_matrix_K_.at<double>(0, 2) = right_ir_cam_info_.K[2];
		trackable_object_.right_ir_camera_matrix_K_.at<double>(1, 0) = right_ir_cam_info_.K[3];
		trackable_object_.right_ir_camera_matrix_K_.at<double>(1, 1) = right_ir_cam_info_.K[4];
		trackable_object_.right_ir_camera_matrix_K_.at<double>(1, 2) = right_ir_cam_info_.K[5];
		trackable_object_.right_ir_camera_matrix_K_.at<double>(2, 0) = right_ir_cam_info_.K[6];
		trackable_object_.right_ir_camera_matrix_K_.at<double>(2, 1) = right_ir_cam_info_.K[7];
		trackable_object_.right_ir_camera_matrix_K_.at<double>(2, 2) = right_ir_cam_info_.K[8];
		trackable_object_.right_ir_camera_distortion_coeffs_ = right_ir_cam_info_.D;

		right_ir_have_camera_info_ = true;
		ROS_INFO("Right IR Camera calibration information obtained.");
	}

	for (int i=0; i<3; i++)
	{
	 for (int j=0; j<3; j++)
	 {
		 camera_matrix_right_ir_(i, j) = trackable_object_.right_ir_camera_matrix_K_.at<double>(i, j);
	 }
	 camera_matrix_right_ir_(i, 3) = 0.0;
	}
	std::cout<<"camera_matrix_right_ir_ initialised="<<std::endl<<camera_matrix_right_ir_<<std::endl;

}

/**
 * The callback function that retrieves the camera calibration information
 *
 * \param msg the ROS message containing the camera calibration information
 *
 */
void SPENode::rgbCameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
{


	if (!rgb_have_camera_info_)
	{
		rgb_cam_info_ = *msg;

		// Calibrated camera
		trackable_object_.rgb_camera_matrix_K_ = cv::Mat(3, 3, CV_64F);
		trackable_object_.rgb_camera_matrix_K_.at<double>(0, 0) = rgb_cam_info_.K[0]; //fx
		trackable_object_.rgb_camera_matrix_K_.at<double>(0, 1) = rgb_cam_info_.K[1];
		trackable_object_.rgb_camera_matrix_K_.at<double>(0, 2) = rgb_cam_info_.K[2]; //cx
		trackable_object_.rgb_camera_matrix_K_.at<double>(1, 0) = rgb_cam_info_.K[3];
		trackable_object_.rgb_camera_matrix_K_.at<double>(1, 1) = rgb_cam_info_.K[4]; //fy
		trackable_object_.rgb_camera_matrix_K_.at<double>(1, 2) = rgb_cam_info_.K[5]; //cy
		trackable_object_.rgb_camera_matrix_K_.at<double>(2, 0) = rgb_cam_info_.K[6];
		trackable_object_.rgb_camera_matrix_K_.at<double>(2, 1) = rgb_cam_info_.K[7];
		trackable_object_.rgb_camera_matrix_K_.at<double>(2, 2) = rgb_cam_info_.K[8];
		trackable_object_.rgb_camera_distortion_coeffs_ = rgb_cam_info_.D;			  //distortion coefficients

		rgb_have_camera_info_ = true;
		ROS_INFO("RGB Camera calibration information obtained.");

		//initialise the renderer
		for(int i=0;i<3;i++){
			for(int j=0;j<3;j++){
				std::cout <<"i,j="<<i<<","<<j<<" = "<<trackable_object_.rgb_camera_matrix_K_.at<double>(i, j)<<std::endl;
			}
		}
		double fx = trackable_object_.rgb_camera_matrix_K_.at<double>(0, 0);
		double fy = trackable_object_.rgb_camera_matrix_K_.at<double>(1, 1);
		double cx = trackable_object_.rgb_camera_matrix_K_.at<double>(0, 2);
		double cy = trackable_object_.rgb_camera_matrix_K_.at<double>(1, 2);

		renderer_.init(fx, fy, cx, cy,  rgb_cam_info_.D, rgb_cam_info_.width, rgb_cam_info_.height, mesh_path_);

	}



	std::cout <<"trackable_object_.rgb_camera_matrix_K_.size()= "<<trackable_object_.rgb_camera_matrix_K_.size()<<std::endl;
	std::cout <<"camera_matrix_rgb_="<<camera_matrix_rgb_<<std::endl;
	for (int i=0; i<3; i++)
   {
     for (int j=0; j<3; j++)
     {
    	 camera_matrix_rgb_(i, j) = trackable_object_.rgb_camera_matrix_K_.at<double>(i, j);
     }
     camera_matrix_rgb_(i, 3) = 0.0;
   }
   std::cout<<"camera_matrix_rgb initialised="<<std::endl<<camera_matrix_rgb_<<std::endl;

}

void SPENode::publishLEDs(const List4DPoints& object_points_camera_frame){


	visualization_msgs::Marker marker;
	marker.header.frame_id = cam_info_.header.frame_id;
	marker.header.stamp = ros::Time();
	marker.ns = nh_.getNamespace();
	marker.id = 0;
	marker.type = visualization_msgs::Marker::SPHERE_LIST;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 0.01;
	marker.scale.y = 0.01;
	marker.scale.z = 0.01;
	marker.color.a = 1.0; // Don't forget to set the alpha!
	marker.color.r = 1.0;
	marker.color.g = 0.0;
	marker.color.b = 0.0;


	for(int i=0;i < object_points_camera_frame.size(); i++){
		const Eigen::Vector4d& X_k = object_points_camera_frame[i];
		geometry_msgs::Point p;
		p.x = X_k(0);
		p.y = X_k(1);
		p.z = X_k(2);
		marker.points.push_back(p);
	}
	vis_pub_.publish(marker);


}


/**
 * The dynamic reconfigure callback function. This function updates the variable within the program whenever they are changed using dynamic reconfigure.
 */
void SPENode::dynamicParametersCallback(monocular_pose_estimator::MonocularPoseEstimatorConfig &config, uint32_t level)
{
  trackable_object_.detection_threshold_value_ = config.threshold_value;
  trackable_object_.gaussian_sigma_ = config.gaussian_sigma;
  trackable_object_.min_blob_area_ = config.min_blob_area;
  trackable_object_.max_blob_area_ = config.max_blob_area;
  trackable_object_.max_width_height_distortion_ = config.max_width_height_distortion;
  trackable_object_.max_circular_distortion_ = config.max_circular_distortion;
  trackable_object_.roi_border_thickness_ = config.roi_border_thickness;

  trackable_object_.setBackProjectionPixelTolerance(config.back_projection_pixel_tolerance);
  trackable_object_.setNearestNeighbourPixelTolerance(config.nearest_neighbour_pixel_tolerance);
  trackable_object_.setCertaintyThreshold(config.certainty_threshold);
  trackable_object_.setValidCorrespondenceThreshold(config.valid_correspondence_threshold);

  ROS_INFO("Parameters changed");
}

} // namespace monocular_pose_estimator
