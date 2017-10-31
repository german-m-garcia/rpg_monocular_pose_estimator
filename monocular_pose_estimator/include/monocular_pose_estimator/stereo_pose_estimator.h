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
 * monocular_pose_estimator_node.h
 *
 *  Created on: Mar 26, 2014
 *      Author: Matthias Fässler
 */

/** \file monocular_pose_estimator_node.h
 * \brief File containing the definition of the Monocular Pose Estimator Node class
 *
 */

#ifndef STEREO_POSE_ESTIMATOR_NODE_H_
#define STEREO_POSE_ESTIMATOR_NODE_H_

#include "ros/ros.h"

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>

#include <dynamic_reconfigure/server.h>
#include <monocular_pose_estimator/MonocularPoseEstimatorConfig.h>

#include "monocular_pose_estimator_lib/stereo_pose_estimator.h"

#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>

#include <tf/transform_listener.h>
#include <GLRenderer/renderLib.h>
#include <image_geometry/pinhole_camera_model.h>

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> SyncPolicyThree;

namespace monocular_pose_estimator
{

struct TrackedObject
{
	std::string name_;
	std::string mesh_path_;
	XmlRpc::XmlRpcValue points_list;

	List4DPoints object_points_camera_frame_;
};

class SPENode
{
private:

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  image_transport::Publisher image_pub_, rgb_image_pub_; //!< The ROS image publisher that publishes the visualisation image
  ros::Publisher pose_pub_; //!< The ROS publisher that publishes the estimated pose.

  ros::Subscriber image_sub_; //!< The ROS subscriber to the raw camera image
  ros::Subscriber camera_info_sub_, rgb_camera_info_sub_, right_ir_camera_info_sub_; //!< The ROS subscriber to the camera info

  ros::Publisher mesh_pub_,vis_pub_, chess_rgb_pub_, chess_ir_pub_; //!< The ROS publisher that publishes markers positions in camera frame

  dynamic_reconfigure::Server<monocular_pose_estimator::MonocularPoseEstimatorConfig> dr_server_; //!< The dynamic reconfigure server
  //dynamic_reconfigure::Server<monocular_pose_estimator::MonocularPoseEstimatorConfig>::CallbackType cb_; //!< The dynamic reconfigure callback type

  geometry_msgs::PoseWithCovarianceStamped predicted_pose_; //!< The ROS message variable for the estimated pose and covariance of the object

  bool have_camera_info_, rgb_have_camera_info_, right_ir_have_camera_info_, tfs_requested_, busy_; //!< The boolean variable that indicates whether the camera calibration parameters have been obtained from the camera
  sensor_msgs::CameraInfo cam_info_; //!< Variable to store the IR camera calibration parameters
  sensor_msgs::CameraInfo rgb_cam_info_; //!< Variable to store the RGB camera calibration parameters
  sensor_msgs::CameraInfo right_ir_cam_info_; //!< Variable to store the right IR camera calibration parameters

  StereoPoseEstimator trackable_object_; //!< Declaration of the object whose pose will be estimated

  TrackedObject tracked_object;

  message_filters::Subscriber<sensor_msgs::Image> ir_sub_, rgb_sub_, right_ir_sub_;
  message_filters::Synchronizer<SyncPolicyThree> sync_three_;

  tf::TransformListener tf_listener_;

  Eigen::Matrix4d rgb_T_ir_; // expressed the pose of the IR frame in the RGB frame
  Matrix3x4d camera_matrix_rgb_, camera_matrix_right_ir_;

  std::string mesh_path_;

  RenderLib renderer_;

  image_geometry::PinholeCameraModel rgb_cam_model_; //used to project 3D points to the RGB frame



  void calcChessboardCorners(cv::Size boardSize, float squareSize, std::vector<cv::Point3f>& corners);

  void publishTargetPose(Eigen::Matrix4d& P);

  void projectLEDsRGBFrame(const List4DPoints& detected_LEDs, cv::Mat& rgb);

  void requestCameraTFs();

  void calibrate_callback(const sensor_msgs::Image::ConstPtr& ir_image_msg,const sensor_msgs::Image::ConstPtr& ir_right_image_msg, const sensor_msgs::Image::ConstPtr& rgb_image_msg);

  void sync_callback_rgb_stereo_ir(const sensor_msgs::Image::ConstPtr& ir_image_msg,const sensor_msgs::Image::ConstPtr& ir_right_image_msg, const sensor_msgs::Image::ConstPtr& rgb_image_msg);

  void rgbCameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);

  void rightIRCameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);

  void leftIRCameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);

  void publishMeshMarker(Eigen::Matrix4d& object_pose);

public:

  SPENode(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  SPENode() : SPENode( ros::NodeHandle(), ros::NodeHandle("~") ){}
  ~SPENode();


  void publishChessboardCorners(const std::string& frame_id, const List4DPoints& object_points_camera_frame);

  void publishLEDs(const List4DPoints& object_points_camera_frame);


  void dynamicParametersCallback(monocular_pose_estimator::MonocularPoseEstimatorConfig &config, uint32_t level);
};

} // monocular_pose_estimator namespace

#endif /* STEREO_POSE_ESTIMATOR_NODE_H_ */
