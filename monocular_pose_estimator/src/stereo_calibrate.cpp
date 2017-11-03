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

#include "monocular_pose_estimator/stereo_calibrate.h"
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <tf_conversions/tf_eigen.h>


#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>


namespace monocular_pose_estimator
{

/**
 * Constructor of the Stereo Pose Estimation Node class
 *
 */
SCNode::SCNode(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
  : nh_(nh), nh_private_(nh_private), have_camera_info_(false), rgb_have_camera_info_(false), right_ir_have_camera_info_(false), tfs_requested_(false), busy_(false),
	ir_sub_(nh_, "/camera/image_raw",1), rgb_sub_(nh_, "/camera/image_rgb",1),right_ir_sub_(nh_, "/camera/image_right_ir",1),
	sync_three_(SyncPolicyThree(10), ir_sub_, right_ir_sub_, rgb_sub_), calibrate_(false)
{

  camera_info_sub_ = nh_.subscribe("/camera/camera_info", 1, &SCNode::leftIRCameraInfoCallback, this);
  rgb_camera_info_sub_ = nh_.subscribe("/camera/image_rgb_camera_info", 1, &SCNode::rgbCameraInfoCallback, this);
  right_ir_camera_info_sub_ = nh_.subscribe("/camera/image_right_ir_camera_info", 1, &SCNode::rightIRCameraInfoCallback, this);


  // Initialize pose publisher
  pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("estimated_pose", 1);

  // Initialize image publisher for visualization
  image_transport::ImageTransport image_transport(nh_);

  //initialize the marker publishers
  chess_rgb_pub_= nh_.advertise<visualization_msgs::Marker>( "rgb_chessboard", 5 );
  chess_ir_pub_= nh_.advertise<visualization_msgs::Marker>( "ir_chessboard", 5 );
  chess_ir2_pub_= nh_.advertise<visualization_msgs::Marker>( "ir2_chessboard", 5 );

  if (nh_private_.getParam("calibrate_camera_base_link", calibrate_camera_base_link_))
  {
	  //if true, we calibrate the camera_link wrt the base_link
	  if(calibrate_camera_base_link_){
		  ROS_INFO("calibrate_camera_base_link_ = true");
		  rgb_img_subscriber_ = image_transport.subscribe("/camera/image_rgb",1,  &SCNode::rgbImageCallback, this);
	  }


	  //if false, we calibrate the camera RGB, IR, IR2 optical frames
	  else{
		  ROS_INFO("calibrate_camera_base_link_ = false");
		  sync_three_.registerCallback(&SCNode::calibrate_callback, this);
	  }

  }
  nh_private_.getParam("offset_base_chessboard_x", offset_base_chessboard_x_);
  nh_private_.getParam("offset_base_chessboard_y", offset_base_chessboard_y_);


}

/**
 * Destructor of the Monocular Pose Estimation Node class
 *
 */
SCNode::~SCNode()
{

}


void SCNode::rgbImageCallback(const sensor_msgs::ImageConstPtr& msg)
{

	cv::Mat rgb, gray;
	try{
		rgb = cv_bridge::toCvShare(msg, "bgr8")->image;
		cv::cvtColor(rgb, gray, cv::COLOR_RGB2GRAY);

	}
	catch (cv_bridge::Exception& e) {
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
		return;
	}
	//detect the chessboard on the rgb frame
	cv::Size boardSize(8,6);
	float squareSize = 0.025;
	std::vector<cv::Point3f> corners_3d;
	calcChessboardCorners( boardSize, squareSize,corners_3d);

	std::vector<cv::Point2f> corners; //this will be filled by the detected corners
	cv::Mat r, R, t;
	Eigen::Matrix4d pose_rgb = Eigen::Matrix4d::Identity();

	/*
	 * find the chessboard in the RGB frame and compute its 3D pose
	 */
	bool rgb_found = cv::findChessboardCorners( gray, boardSize, corners, cv::CALIB_CB_ADAPTIVE_THRESH );
	if(rgb_found) {
		cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
			cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

		cv::drawChessboardCorners(rgb, boardSize, cv::Mat(corners), rgb_found);


		//compute the pose

		cv::solvePnP(corners_3d, corners, trackable_object_.rgb_camera_matrix_K_, cv::Mat(), r, t);
		cv::Rodrigues(r, R);


		for(int i = 0;i<R.rows; i++)
			for(int j = 0;j<R.cols; j++){
				pose_rgb(i,j) = R.at<double>(i,j);
			}


		pose_rgb(0,3) = t.at<double>(0,0);
		pose_rgb(1,3) = t.at<double>(1,0);
		pose_rgb(2,3) = t.at<double>(2,0);



	}

	//visualize the RGB chessboard corners in 3D
	if(rgb_found ){
		//iterate over the 3D coordinates of the chessboard points
		List4DPoints object_points_camera_frame(corners_3d.size()), object_points_ir_camera_frame(corners_3d.size()), object_points_ir2_camera_frame(corners_3d.size());
		int i=0;
		for(cv::Point3f& corner_object_frame : corners_3d)
		{
			//the pose of the corner in camera frame = rgb_T_object * pose_object_frame
			Eigen::Vector4d corner_object_frame_eigen(corner_object_frame.x, corner_object_frame.y, corner_object_frame.z, 1.0);
			//if using the RGB estimated pose of the chessboard
			object_points_camera_frame[i] = pose_rgb * corner_object_frame_eigen;
			i++;

		}
		//if using the RGB estimated pose of the chessboard

		std::string colour = "green";
		publishChessboardCorners(rgb_cam_info_.header.frame_id, object_points_camera_frame, colour);


		Eigen::AngleAxisd rollAngle(0., Eigen::Vector3d::UnitZ());
		Eigen::AngleAxisd yawAngle(0., Eigen::Vector3d::UnitY());
		Eigen::AngleAxisd pitchAngle(M_PI/2., Eigen::Vector3d::UnitX());
		Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;
		Eigen::Matrix3d rot3 = q.toRotationMatrix();
		Eigen::Matrix4d rot4 = Eigen::Matrix4d::Identity();
		rot4.block(0,0,3,3) = rot3;
		pose_rgb = pose_rgb * rot4;

		Eigen::AngleAxisd rollAngle2(M_PI/2., Eigen::Vector3d::UnitZ());
		Eigen::AngleAxisd yawAngle2(0., Eigen::Vector3d::UnitY());
		Eigen::AngleAxisd pitchAngle2(0., Eigen::Vector3d::UnitX());
		q = rollAngle2 * yawAngle2 * pitchAngle2;
		rot3 = q.toRotationMatrix();
		rot4 = Eigen::Matrix4d::Identity();
		rot4.block(0,0,3,3) = rot3;
		pose_rgb = pose_rgb * rot4;

		//pull it down to the ground level ( 6 squares * 2,5cm)
		Eigen::Matrix4d translation = Eigen::Matrix4d::Identity();
		translation(2,3) = -6 * 0.025 - 0.022;
		pose_rgb = pose_rgb * translation;

		publishChessboardPose(pose_rgb);
		publishCodedBaseLink();


		//we can now estimate the transformation between camera_link and base_link
		estimateCameraBaseLinkPose();

	}
	cv::imshow("rgb", rgb);
	cv::waitKey(1);


}

void SCNode::estimateCameraBaseLinkPose(){


	tf::StampedTransform transformation;
	Eigen::Affine3d affine3D;

	ROS_INFO_STREAM("requesting TF: "<<rgb_cam_info_.header.frame_id<<" - "<<cam_info_.header.frame_id);
	try{

//		while(!tf_listener_.canTransform("base_link_estimation","camera_rgb_frame",
//								   ros::Time(0))){
//			ros::Duration(0.03).sleep();
//		}

		tf_listener_.lookupTransform( "base_link_estimation","camera_rgb_frame",
								   ros::Time(0), transformation);

	}


	catch (tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
		return;
	}


	tf::transformTFToEigen(tf::Transform(transformation),affine3D);
	Eigen::Matrix3d rot = affine3D.matrix().block(0,0,3,3);
	Eigen::Quaterniond q(rot);
	std::cout<<" estimated tf="<< affine3D.matrix()<<std::endl;
	std::cout <<"q="<<q.x()<<" "<<q.y()<<" "<<q.z()<<" "<<q.w()<<" "<<std::endl;


}

void SCNode::publishChessboardPose(Eigen::Matrix4d& P){

	//publish the tf
	tf::Transform P_tf;
	Eigen::Affine3d affinePose(P);
	tf::transformEigenToTF(affinePose, P_tf);
	static tf::TransformBroadcaster br;
	br.sendTransform(tf::StampedTransform(P_tf, ros::Time::now(), rgb_cam_info_.header.frame_id, "chessboard"));
	ros::Duration(0.1).sleep();

}

void SCNode::publishCodedBaseLink(){

	Eigen::Matrix4d translation = Eigen::Matrix4d::Identity();

	translation(0,3) = -offset_base_chessboard_x_;
	translation(1,3) = -offset_base_chessboard_y_;
	//publish the tf
	tf::Transform P_tf;
	Eigen::Affine3d affinePose(translation);
	tf::transformEigenToTF(affinePose, P_tf);
	static tf::TransformBroadcaster br;
	br.sendTransform(tf::StampedTransform(P_tf, ros::Time::now(), "chessboard", "base_link_estimation"));

}


void SCNode::requestCameraTFs(){
	//request the tf between IR and RGB optical frames
	tf::StampedTransform rgb_transform_ir, transform_irs, rgb_transform_ir2;
	try{


		bool rc = false;
		while( !have_camera_info_ && !rgb_have_camera_info_ && ! right_ir_have_camera_info_){
			ros::Duration(0.03).sleep();
		}

		ROS_INFO_STREAM("requesting TF: "<<rgb_cam_info_.header.frame_id<<" - "<<cam_info_.header.frame_id);
		tf_listener_.lookupTransform(rgb_cam_info_.header.frame_id, cam_info_.header.frame_id,
							   ros::Time(0), rgb_transform_ir);

		tf_listener_.lookupTransform(rgb_cam_info_.header.frame_id, right_ir_cam_info_.header.frame_id,
									   ros::Time(0), rgb_transform_ir2);

		tf_listener_.lookupTransform(right_ir_cam_info_.header.frame_id, cam_info_.header.frame_id,
									   ros::Time(0), transform_irs);



		Eigen::Affine3d affine3D;
		tf::transformTFToEigen(tf::Transform(rgb_transform_ir),affine3D);
		rgb_T_ir_ = affine3D.matrix();

		  /*
		   * translation offset=[-0.05032996520365117;
								 0.01222315349346847;
								 -0.009198530014921102]


								data obtained from a close look onto the chessboard
								 translation offset=[-0.05289814193997809;
													 0.00954434485606509;
													 0.001262808202716181]
								[link1_broadcaster-1] killing on exit
								 requested tf: rgb_T_ir2=        1         0         0 0.0180554
										0         1         0     0.009
										0         0         1    -0.001
										0         0         0         1



		   */
		  //if using Intel provided tfs need to adjust the translation..
	//	  rgb_T_ir_(0,3) = -0.052;
	//	  rgb_T_ir_(1,3) = 0.009;
	//	  rgb_T_ir_(2,3) = -0.001;


		std::cout <<" requested TF rgb_T_ir_="<<std::endl<<rgb_T_ir_<<std::endl;
		tf::transformTFToEigen(tf::Transform(rgb_transform_ir2),affine3D);
		rgb_T_ir2_ = affine3D.matrix();

		tf::transformTFToEigen(tf::Transform(transform_irs),affine3D);
		ir2_T_ir_ = affine3D.matrix();
	}
	catch (tf::TransformException &ex) {
	  ROS_ERROR("%s",ex.what());
	  ros::Duration(1.0).sleep();
	}
}

void SCNode::calibrate_callback(const sensor_msgs::Image::ConstPtr& ir_image_msg,const sensor_msgs::Image::ConstPtr& ir_right_image_msg, const sensor_msgs::Image::ConstPtr& rgb_image_msg){


	ROS_INFO("SCNode::calibrate_callback");

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
	cv::Mat ir2 = cv_right_ir_ptr->image;
	cv::Mat rgb = cv_rgb_ptr->image;
	cv::Mat gray, ir_color, ir2_color;
	cv::Mat rgb_debug = rgb.clone();
	cv::cvtColor(rgb, gray, cv::COLOR_RGB2GRAY);
	cv::cvtColor(ir, ir_color, cv::COLOR_GRAY2RGB);
	cv::cvtColor(ir2, ir2_color, cv::COLOR_GRAY2RGB);

	cv::Size boardSize(8,6);
	float squareSize = 0.025;
	std::vector<cv::Point3f> corners_3d;
	calcChessboardCorners( boardSize, squareSize,corners_3d);
	{

		std::vector<cv::Point2f> corners; //this will be filled by the detected corners
		std::vector<cv::Point2f> corners_ir; //this will be filled by the detected corners
		std::vector<cv::Point2f> corners_ir2; //this will be filled by the detected corners
		cv::Mat r, R, t, r_ir, t_ir, r_ir2, t_ir2;
		Eigen::Matrix4d pose_rgb = Eigen::Matrix4d::Identity();
		Eigen::Matrix4d pose_ir = Eigen::Matrix4d::Identity(), pose_ir2 = Eigen::Matrix4d::Identity();
		Eigen::Matrix4d ir_T_rgb = Eigen::Matrix4d::Identity();


		/*
		 * find the chessboard in the RGB frame
		 */
		bool rgb_found = cv::findChessboardCorners( gray, boardSize, corners, cv::CALIB_CB_ADAPTIVE_THRESH );
		if(rgb_found) {
			cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
				cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

			cv::drawChessboardCorners(rgb, boardSize, cv::Mat(corners), rgb_found);


			//compute the pose

			cv::solvePnP(corners_3d, corners, trackable_object_.rgb_camera_matrix_K_, cv::Mat(), r, t);
			cv::Rodrigues(r, R);
			//std::cout <<"estimated pose in RGB="<<R<<" translation="<<t<< " type="<<r.type()  <<std::endl;

			for(int i = 0;i<R.rows; i++)
				for(int j = 0;j<R.cols; j++){
					pose_rgb(i,j) = R.at<double>(i,j);
				}
			pose_rgb(0,3) = t.at<double>(0,0);
			pose_rgb(1,3) = t.at<double>(1,0);
			pose_rgb(2,3) = t.at<double>(2,0);
		}




		/*
		 * find the chessboard in the left IR frame
		 */
		bool ir_found = cv::findChessboardCorners( ir, boardSize, corners_ir, cv::CALIB_CB_ADAPTIVE_THRESH );
		if(ir_found){

			 cv::cornerSubPix(ir, corners_ir, cv::Size(11, 11), cv::Size(-1, -1),
						cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

			cv::drawChessboardCorners(ir_color, boardSize, cv::Mat(corners_ir), ir_found);

			//compute the pose
			cv::solvePnP(corners_3d, corners_ir, trackable_object_.camera_matrix_K_, cv::Mat(), r_ir, t_ir);
			cv::Rodrigues(r_ir, R);
			//std::cout <<"estimated pose in IR="<<R<<" translation="<<t<<std::endl;


			for(int i = 0;i<R.rows; i++)
				for(int j = 0;j<R.cols; j++){
					pose_ir(i,j) = R.at<double>(i,j);
				}
			pose_ir(0,3) = t_ir.at<double>(0,0);
			pose_ir(1,3) = t_ir.at<double>(1,0);
			pose_ir(2,3) = t_ir.at<double>(2,0);

		}
		/*
		 * find the chessboard in the right IR frame
		 */
		bool ir2_found = cv::findChessboardCorners( ir2, boardSize, corners_ir2, cv::CALIB_CB_ADAPTIVE_THRESH );
		if(ir2_found){

			 cv::cornerSubPix(ir2, corners_ir2, cv::Size(11, 11), cv::Size(-1, -1),
						cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

			cv::drawChessboardCorners(ir2_color, boardSize, cv::Mat(corners_ir2), ir2_found);

			//compute the pose
			cv::solvePnP(corners_3d, corners_ir2, trackable_object_.right_ir_camera_matrix_K_, cv::Mat(), r_ir2, t_ir2);
			cv::Rodrigues(r_ir2, R);
			//std::cout <<"estimated pose in IR2="<<R<<" translation="<<t_ir2<<std::endl;


			for(int i = 0;i<R.rows; i++)
				for(int j = 0;j<R.cols; j++){
					pose_ir2(i,j) = R.at<double>(i,j);
				}
			pose_ir2(0,3) = t_ir2.at<double>(0,0);
			pose_ir2(1,3) = t_ir2.at<double>(1,0);
			pose_ir2(2,3) = t_ir2.at<double>(2,0);

		}


		if(rgb_found && ir_found && ir2_found){
			// ir_T_rgb = ir_T_c * (rgb_T_c.inverse() )
			ir_T_rgb = pose_ir * (pose_rgb.inverse());
			std::cout <<"ir_T_rgb="<<ir_T_rgb<<std::endl;

			std::cout <<"translation offset="<<t - t_ir<<std::endl;

			//iterate over the RGB found corners
			for(cv::Point2f& rgb_corner : corners){
				cv::circle(rgb_debug, rgb_corner, 7, CV_RGB(255, 0, 0), 1);
			}
			//iterate over the 3D coordinates of the chessboard points
			List4DPoints object_points_camera_frame(corners_3d.size()), object_points_ir_camera_frame(corners_3d.size()), object_points_ir2_camera_frame(corners_3d.size());
			int i=0;
			for(cv::Point3f& corner_object_frame : corners_3d)
			{
				//the pose of the corner in camera frame = rgb_T_object * pose_object_frame
				Eigen::Vector4d corner_object_frame_eigen(corner_object_frame.x, corner_object_frame.y, corner_object_frame.z, 1.0);
				//if using the RGB estimated pose of the chessboard
				object_points_camera_frame[i] = pose_rgb * corner_object_frame_eigen;

				//if using the IR estimated pose of the chessboard
				//object_points_ir_camera_frame[i] = pose_ir * corner_object_frame_eigen;

				//to check the fixed calibration matrix...
				//the points are now in the RGB frame of reference
				object_points_ir_camera_frame[i] = rgb_T_ir_ *  pose_ir * corner_object_frame_eigen;


				//points in the IR2 frame of reference
				//when calibrating run this line:
				//object_points_ir2_camera_frame[i] = rgb_T_ir_ * (ir2_T_ir_.inverse())*  pose_ir2 * corner_object_frame_eigen;

				//to validate the calibration:
				object_points_ir2_camera_frame[i] = rgb_T_ir2_ *  pose_ir2 * corner_object_frame_eigen;


				i++;

			}
			//if using the RGB estimated pose of the chessboard
			std::string colour("red");
			publishChessboardCorners(rgb_cam_info_.header.frame_id, object_points_ir_camera_frame, colour);
			colour = "green";
			publishChessboardCorners(rgb_cam_info_.header.frame_id, object_points_camera_frame, colour);
			colour = "blue";
			publishChessboardCorners(rgb_cam_info_.header.frame_id, object_points_ir2_camera_frame, colour);
			//publishChessboardCorners(right_ir_cam_info_.header.frame_id, object_points_ir2_camera_frame, colour);

			//cv::imshow("debug color", rgb_debug);
			//Eigen::Matrix4d rgb_T_ir2 = rgb_T_ir_ * (ir2_T_ir_.inverse());
			//std::cout <<" requested tf: rgb_T_ir2="<<rgb_T_ir2<<std::endl;



		}



	}

	cv::imshow("rgb", rgb);
	cv::imshow("ir_color", ir_color);
	cv::imshow("ir2_color", ir2_color);
	cv::waitKey(1);



}



void SCNode::calcChessboardCorners(cv::Size boardSize, float squareSize, std::vector<cv::Point3f>& corners)
{
    corners.resize(0);

    for( int i = 0; i < boardSize.height; i++ )
        for( int j = 0; j < boardSize.width; j++ )
            corners.push_back(cv::Point3f(float(j*squareSize),
                                      float(i*squareSize), 0));
}



/**
 * The callback function that retrieves the camera calibration information
 *
 * \param msg the ROS message containing the camera calibration information
 *
 */
void SCNode::leftIRCameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
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


void SCNode::rightIRCameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg){


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



}

/**
 * The callback function that retrieves the camera calibration information
 *
 * \param msg the ROS message containing the camera calibration information
 *
 */
void SCNode::rgbCameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
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


		rgb_cam_model_.fromCameraInfo(msg);

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

}


void SCNode::publishChessboardCorners(const std::string& frame_id, const List4DPoints& object_points_camera_frame, std::string& colour){


	visualization_msgs::Marker marker;
	marker.header.frame_id = frame_id;
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
	if(colour =="red"){
		marker.color.r = 1.0;
		marker.color.g = 0.0;
		marker.color.b = 0.0;
	}
	else if(colour == "green"){
		marker.color.r = 0.0;
		marker.color.g = 1.0;
		marker.color.b = 0.0;
	}
	else if (colour == "blue"){
		marker.color.r = 0.0;
		marker.color.g = 0.0;
		marker.color.b = 1.0;
	}


	for(int i=0;i < object_points_camera_frame.size(); i++){
		const Eigen::Vector4d& X_k = object_points_camera_frame[i];
		geometry_msgs::Point p;
		p.x = X_k(0);
		p.y = X_k(1);
		p.z = X_k(2);
		marker.points.push_back(p);
	}
	if(colour =="red")
		chess_ir_pub_.publish(marker);
	else if(colour =="green")
		chess_rgb_pub_.publish(marker);
	else if(colour =="blue")
		chess_ir2_pub_.publish(marker);
}


} // namespace monocular_pose_estimator


int main(int argc, char* argv[])
{
  ros::init(argc, argv, "stereo_calibrate");

  monocular_pose_estimator::SCNode sc_node;


  ros::spin();

  return 0;
}
