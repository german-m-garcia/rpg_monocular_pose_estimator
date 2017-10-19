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
 double  * StereoPoseEstimator.cpp
 *
 *  Created on: July 29, 2013
 *      Author: Karl Schwabe
 */

/**
 * \file pose_estimator.cpp
 * \brief File containing the definitions of the functions for the StereoPoseEstimator class
 *
 */

#include "monocular_pose_estimator_lib/stereo_pose_estimator.h"
#include "monocular_pose_estimator_lib/cliques.h"
#include "ros/ros.h"
#include "hungarian/hungarian.h"

namespace monocular_pose_estimator
{

StereoPoseEstimator::StereoPoseEstimator()
{
	back_projection_pixel_tolerance_ = 3;
	nearest_neighbour_pixel_tolerance_ = 5;
	certainty_threshold_ = 0.75;
	valid_correspondence_threshold_ = 0.7;

	it_since_initialized_ = 0;

}


void StereoPoseEstimator::permute(std::vector<int>& array,int i,int length) {

	if (length == i){
		for (auto elem: array)
		  std::cout << elem << ' ';
		return;
	}

	int j = i;
	for (j = i; j < length; j++) {

		std::swap(array[i],array[j]);
		permute(array,i+1,length);
		std::swap(array[i],array[j]);
	}
	return;
}


bool StereoPoseEstimator::isDistanceValid(double dist){
	for(auto elem : dist_vector_){
		double value = dist / elem;
		if (0.9 < value && value < 1.1)
			return true;
	}
	return false;
}

void StereoPoseEstimator::computeHistograms(List4DPoints& points, std::vector<std::vector<int> >& graph){


	cv::Mat distances(points.size(), points.size(), CV_64FC1, 0.);
	graph.resize(points.size());
	for(int i=0;i<points.size();i++)
		graph[i].resize(points.size());

	//iterate over each pair
	for(int i=0;i<points.size();i++){

		std::vector<double> histogram(points.size());
		Eigen::Vector4d& p_i = points(i);
		for(int j=i;j<points.size();j++){
			Eigen::Vector4d& p_j = points(j);
			double dx = p_i[0]-p_j[0];
			double dy = p_i[1]-p_j[1];
			double dz = p_i[2]-p_j[2];
			distances.at<double>(i,j) = std::sqrt(dx*dx + dy*dy + dz*dz);
			//histogram[j] = distances.at<double>(i,j);
			if(distances.at<double>(i,j)> 0.){
				//check if the distance is valid
				if(isDistanceValid(distances.at<double>(i,j))){
					std::cout <<"distances.at<double>(i,j)="<<distances.at<double>(i,j)<<std::endl;
					//if so, add the edges (i,j) (j,i) to the graph
					graph[i][j] = 1;
					graph[j][i] = 1;
				}
			}
		}
	}
	debug_graph(graph);



}



void StereoPoseEstimator::computeHistograms(List4DPoints& points){

	cv::Mat distances(points.size(), points.size(), CV_64FC1, 0.);

	//iterate over each pair
	for(int i=0;i<points.size();i++){

		std::vector<double> histogram(points.size());
		Eigen::Vector4d& p_i = points(i);
		for(int j=i;j<points.size();j++){
			Eigen::Vector4d& p_j = points(j);
			double dx = p_i[0]-p_j[0];
			double dy = p_i[1]-p_j[1];
			double dz = p_i[2]-p_j[2];
			distances.at<double>(i,j) = std::sqrt(dx*dx + dy*dy + dz*dz);
			//histogram[j] = distances.at<double>(i,j);
			if(distances.at<double>(i,j)> 0.){
				dist_vector_.push_back(distances.at<double>(i,j));
			}
		}
	}
	std::sort(dist_vector_.begin(), dist_vector_.end());
	std::cout << "dist_vector = ";
	for(auto elem : dist_vector_)
		std::cout <<elem<<" ";
	std::cout <<std::endl;
}

//double StereoPoseEstimator::computeCost(){
//
//}

/*
 * returns the detected undistorted LED positions in each IR stereo image
 * @ir: the left IR image
 * @ir2: the right IR image
 * @time_to_predict: allowed time to process
 * @detected_led_positions: the undistorted pixel coordinates of the LEDs in the left IR image
 * @detected_led_positions2: the undistorted pixel coordinates of the LEDs in the right IR image
 */
bool StereoPoseEstimator::estimateFromStereo(cv::Mat& ir, cv::Mat& ir2, double time_to_predict, List2DPoints& detected_led_positions,List2DPoints& detected_led_positions2){

	List4DPoints triangulated_LEDs;
	bool right = true;

	region_of_interest_ = cv::Rect(0, 0, ir.cols, ir.rows);

	// Do detection of LEDs in image
	LEDDetector::findLeds(ir, region_of_interest_, detection_threshold_value_, gaussian_sigma_, min_blob_area_,
						  max_blob_area_, max_width_height_distortion_, max_circular_distortion_,
						  detected_led_positions, distorted_detection_centers_, camera_matrix_K_,
						  camera_distortion_coeffs_, !right);

	// Do detection of LEDs in image
	LEDDetector::findLeds(ir2, region_of_interest_, detection_threshold_value_, gaussian_sigma_, min_blob_area_,
						  max_blob_area_, max_width_height_distortion_, max_circular_distortion_,
						  detected_led_positions2, distorted_detection_centers_, right_ir_camera_matrix_K_,
						  right_ir_camera_distortion_coeffs_, right);

	//create a confusion matrix with the distances between the detections
	cv::Mat confusion_distances_left = cv::Mat::zeros(detected_led_positions.size(), detected_led_positions2.size(), CV_32FC2);
	cv::Mat confusion_distances_right = cv::Mat::zeros(detected_led_positions.size(), detected_led_positions2.size(), CV_32FC2);

	//iterate for each detection in the left image
	for(int i=0; i< detected_led_positions.size(); i++){
		Eigen::Vector2d& p_left = detected_led_positions(i);
		for(int j=i; j< detected_led_positions.size(); j++){
			Eigen::Vector2d& p_left_other = detected_led_positions(j);
			confusion_distances_left.at<cv::Vec2f>(i,j)[0] = p_left[0] - p_left_other[0];
			confusion_distances_left.at<cv::Vec2f>(i,j)[1] = p_left[1] - p_left_other[1];
		}
	}
	for(int i=0; i< detected_led_positions2.size(); i++){
		Eigen::Vector2d& p_right = detected_led_positions2(i);
		for(int j=i; j< detected_led_positions2.size(); j++){
			Eigen::Vector2d& p_right_other = detected_led_positions2(j);
			confusion_distances_right.at<cv::Vec2f>(i,j)[0] = p_right[0] - p_right_other[0];
			confusion_distances_right.at<cv::Vec2f>(i,j)[1] = p_right[1] - p_right_other[1];
		}
	}

	std::cout <<"LED left histogram="<<std::endl<<confusion_distances_left<<std::endl;
	std::cout <<"LED right histogram="<<std::endl<<confusion_distances_right<<std::endl;


	//print the points
	for(int i=0; i< detected_led_positions.size(); i++){
			Eigen::Vector2d& p_left = detected_led_positions(i);
			std::cout <<"Left point #"<<i<<" = "<<p_left<<std::endl;
	}
	for(int i=0; i< detected_led_positions2.size(); i++){
			Eigen::Vector2d& p_right = detected_led_positions2(i);
			std::cout <<"Right point #"<<i<<" = "<<p_right<<std::endl;
	}
	std::vector<int> matches(detected_led_positions2.size());

	//obtain the best match between LED detections in the stereo pair
	getBestMatch(detected_led_positions, detected_led_positions2,matches);
	for(auto elem: matches)
		std::cout<< elem << " ";
	std::cout << std::endl;
	//compute the disparity
	findDisparities(detected_led_positions, detected_led_positions2,matches,triangulated_LEDs);
	 std::vector<std::vector<int> > graph;
	computeHistograms(triangulated_LEDs, graph);
	//find the maximum clique of size the number of markers in the model
	int K = object_points_.size();
	std::vector<std::vector<int> > cliques;
	find_cliques(graph, K, cliques);
	debug_graph(cliques);

}

/**
 * @detected_led_positions: input, the detected undistorted centres of the LEDs in the left IR image
 * @detected_led_positions2: input, the detected undistorted centres of the LEDs in the right IR image
 * @matches: output, the indices of the corresponding right LEDs detections wrt the left IR detections
 * @triangulated_LEDs: output, the detected LEDs 3D coordinates
 *
 */
void StereoPoseEstimator::findDisparities(const List2DPoints& detected_led_positions,const List2DPoints& detected_led_positions2, std::vector<int>& matches, List4DPoints& triangulated_LEDs){

	triangulated_LEDs.resize(detected_led_positions.size());
	std::cout <<"detected_led_positions.size()="<<detected_led_positions.size()<<" triangulated_LEDs.size()"<<triangulated_LEDs.size()<<std::endl;
	for(int i=0; i< detected_led_positions.size(); i++){
		const Eigen::Vector2d& p_left = detected_led_positions(i);

		const Eigen::Vector2d& p_right = detected_led_positions2(matches[i]);
		const double disparity = p_left[0] - p_right[0];
		const double fx = camera_matrix_K_.at<double>(0, 0);
		const double fy = camera_matrix_K_.at<double>(1, 1);
		const double Z = B*fx/disparity;
		const double X = Z /fx * (p_left[0] -camera_matrix_K_.at<double>(2, 0));
		const double Y = Z /fy * (p_left[1] -camera_matrix_K_.at<double>(2, 1));
		std::cout <<"pair #"<<i<<" disparity="<<disparity<<" B="<<B<<" f="<<fx<<" Z="<<Z<<std::endl;
		std::cout <<"X="<<X<<" Y="<<Y<<std::endl;

		Eigen::Vector4d LED;
		LED[0] = X;
		LED[1] = Y;
		LED[2] = Z;
		LED[3] = 1.;
		triangulated_LEDs[i]=LED;
	}

}

void StereoPoseEstimator::getBestMatch( List2DPoints& detected_led_positions,List2DPoints& detected_led_positions2, std::vector<int>& matches){
	//iterate over each point in Left
	for(int i=0; i< detected_led_positions.size(); i++){
		Eigen::Vector2d& p_left = detected_led_positions(i);
		//look for the closest point (in the same row)
		double min = std::numeric_limits<double>::max();
		int index = -1;
		for(int j=0; j< detected_led_positions2.size(); j++){
			Eigen::Vector2d& p_right = detected_led_positions2(j);
			double d = std::fabs(p_left[1] - p_right[1]);
			//std::cout <<"i,j,d="<<i<<" "<<j<<" "<<d<<std::endl;
			if(d <min){
				min = d;
				index = j;
				//std::cout <<"index, min="<<index<<" "<<min<<std::endl;
			}

		}
		matches[i] = index;
	}
}


void StereoPoseEstimator::setMarkerPositions(List4DPoints positions_of_markers_on_object)
{
  object_points_ = positions_of_markers_on_object;
  object_points_camera_frame_ = positions_of_markers_on_object;
  for(int i=0;i < object_points_.size();i++){
	  const Eigen::Vector4d& LED = object_points_[i];
	  std::cout <<"LED #"<<i<<" = "<<LED<<std::endl;
  }
  computeHistograms(object_points_);
}

List4DPoints StereoPoseEstimator::getMarkerPositions()
{
  return object_points_;
}

void StereoPoseEstimator::setPredictedPose(const Eigen::Matrix4d & pose, double time)
{
  predicted_pose_ = pose;
  predicted_time_ = time;

}

Eigen::Matrix4d StereoPoseEstimator::getPredictedPose()
{
  return predicted_pose_;
}

void StereoPoseEstimator::setBackProjectionPixelTolerance(double tolerance)
{
  back_projection_pixel_tolerance_ = tolerance;
}

double StereoPoseEstimator::getBackProjectionPixelTolerance()
{
  return back_projection_pixel_tolerance_;
}

void StereoPoseEstimator::setNearestNeighbourPixelTolerance(double tolerance)
{
  nearest_neighbour_pixel_tolerance_ = tolerance;
}

double StereoPoseEstimator::getNearestNeighbourPixelTolerance()
{
  return nearest_neighbour_pixel_tolerance_;
}

void StereoPoseEstimator::setCertaintyThreshold(double threshold)
{
  certainty_threshold_ = threshold;
}

double StereoPoseEstimator::getCertaintyThreshold()
{
  return certainty_threshold_;
}

void StereoPoseEstimator::setValidCorrespondenceThreshold(double threshold)
{
  valid_correspondence_threshold_ = threshold;
}

double StereoPoseEstimator::getValidCorrespondenceThreshold()
{
  return valid_correspondence_threshold_;
}


} // namespace

