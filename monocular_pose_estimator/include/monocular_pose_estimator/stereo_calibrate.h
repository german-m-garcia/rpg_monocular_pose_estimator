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

#ifndef STEREO_CALIBRATE_NODE_H_
#define STEREO_CALIBRATE_NODE_H_

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
#include <image_geometry/pinhole_camera_model.h>

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> SyncPolicyThree;

namespace monocular_pose_estimator
{


class SCNode
{
private:

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  image_transport::Publisher image_pub_, rgb_image_pub_; //!< The ROS image publisher that publishes the visualisation image
  ros::Publisher pose_pub_; //!< The ROS publisher that publishes the estimated pose.

  ros::Subscriber image_sub_; //!< The ROS subscriber to the raw camera image
  ros::Subscriber camera_info_sub_, rgb_camera_info_sub_, right_ir_camera_info_sub_; //!< The ROS subscriber to the camera info

  ros::Publisher chess_rgb_pub_, chess_ir_pub_, chess_ir2_pub_; //!< The ROS publisher that publishes markers positions in camera frame

  bool have_camera_info_, rgb_have_camera_info_, right_ir_have_camera_info_, tfs_requested_, busy_; //!< The boolean variable that indicates whether the camera calibration parameters have been obtained from the camera
  bool calibrate_; //!< If true, runs the calibration procdure with a checkerboard, otherwise verifies the current calibration
  bool calibrate_camera_base_link_; //!< If true, calibrates the pose of the camera link wrt the base link
  sensor_msgs::CameraInfo cam_info_; //!< Variable to store the IR camera calibration parameters
  sensor_msgs::CameraInfo rgb_cam_info_; //!< Variable to store the RGB camera calibration parameters
  sensor_msgs::CameraInfo right_ir_cam_info_; //!< Variable to store the right IR camera calibration parameters

  message_filters::Subscriber<sensor_msgs::Image> ir_sub_, rgb_sub_, right_ir_sub_;
  image_transport::Subscriber rgb_img_subscriber_;
  message_filters::Synchronizer<SyncPolicyThree> sync_three_;

  StereoPoseEstimator trackable_object_; //!< Declaration of the object whose pose will be estimated

  tf::TransformListener tf_listener_;

  Eigen::Matrix4d rgb_T_ir_; // expresses the pose of the IR frame in the RGB frame
  Eigen::Matrix4d rgb_T_ir2_; // expresses the pose of the IR frame in the RGB frame
  Eigen::Matrix4d ir2_T_ir_; // expresses the pose of the IR frame in the IR2 frame

  Matrix3x4d camera_matrix_rgb_, camera_matrix_right_ir_;

  image_geometry::PinholeCameraModel rgb_cam_model_; //used to project 3D points to the RGB frame

  double offset_base_chessboard_x_, offset_base_chessboard_y_;


  void calcChessboardCorners(cv::Size boardSize, float squareSize, std::vector<cv::Point3f>& corners);


  void requestCameraTFs();

  void calibrate_callback(const sensor_msgs::Image::ConstPtr& ir_image_msg,const sensor_msgs::Image::ConstPtr& ir_right_image_msg, const sensor_msgs::Image::ConstPtr& rgb_image_msg);

  void rgbCameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);

  void rightIRCameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);

  void leftIRCameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);

  void publishMeshMarker(Eigen::Matrix4d& object_pose);

  void publishChessboardCorners(const std::string& frame_id, const List4DPoints& object_points_camera_frame, std::string& colour);

  void rgbImageCallback(const sensor_msgs::ImageConstPtr& msg);

  void publishChessboardPose(Eigen::Matrix4d& P);

  void publishCodedBaseLink();

  void estimateCameraBaseLinkPose();
public:

  SCNode(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  SCNode() : SCNode( ros::NodeHandle(), ros::NodeHandle("~") ){}
  ~SCNode();


};

} // monocular_pose_estimator namespace

#endif /* STEREO_CALIBRATE_NODE_H_ */
