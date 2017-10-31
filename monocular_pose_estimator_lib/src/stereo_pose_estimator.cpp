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
	for(auto elem : target_.dist_vector_){
		double value = dist / elem;
		if (0.95 < value && value < 1.05)
			return true;
	}
	return false;
}

void StereoPoseEstimator::computeDetectionsGraph(List4DPoints& points, std::vector<std::vector<int> >& graph){

	detections_distances_matrix_ = cv::Mat::zeros(points.size(), points.size(), CV_64FC1);
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
			double d = std::sqrt(dx*dx + dy*dy + dz*dz);
			//histogram[j] = distances.at<double>(i,j);
			detections_distances_matrix_.at<double>(i,j) = d;
			detections_distances_matrix_.at<double>(j,i) = d;
			if(d > 0.){

				//check if the distance is valid
				if(isDistanceValid(d)){
					//if so, add the edges (i,j) (j,i) to the graph
					graph[i][j] = 1;
					graph[j][i] = 1;
				}
			}
		}
	}
}

void StereoPoseEstimator::extractCliqueDistsMatrix(std::vector<int>& clique, cv::Mat& dists_matrix_clique){
	dists_matrix_clique = cv::Mat::zeros(clique.size(), clique.size(), CV_64FC1);
	for(int i = 0;i < clique.size();i++){
		for(int j = i;j < clique.size();j++){
			dists_matrix_clique.at<double>(i,j) =  detections_distances_matrix_.at<double>(clique[i],clique[j]);
			dists_matrix_clique.at<double>(j,i) = dists_matrix_clique.at<double>(i,j);
		}
	}
}

void StereoPoseEstimator::extractOrderedDistsVectorsFromMatrix(cv::Mat& dists_matrix_clique, std::vector <std::vector<double>>& dists_vector_clique){
	dists_vector_clique.resize(dists_matrix_clique.rows);
	for(int i=0;i< dists_matrix_clique.rows;i++)
		dists_vector_clique[i].resize(dists_matrix_clique.cols);
	for(int i=0;i< dists_matrix_clique.rows;i++){
		for(int j=i; j<dists_matrix_clique.cols; j++ ){
			dists_vector_clique[i][j] = dists_matrix_clique.at<double>(i,j);
			dists_vector_clique[j][i] = dists_matrix_clique.at<double>(i,j);
		}
	}

	for(int i=0;i< dists_vector_clique.size();i++){
		//sort the distance vector of element #i
		std::sort(dists_vector_clique[i].begin(), dists_vector_clique[i].end());
		std::cout <<"extracted ordered dists_vector:"<<std::endl;
		for(auto elem : dists_vector_clique[i])
			std::cout <<elem<<" ";
		std::cout <<std::endl;

	}


}

void StereoPoseEstimator::hornPoseEstimation(List4DPoints& d_i, List4DPoints& m_i, Eigen::Matrix4d& P){

	Eigen::Vector4d d_bar4(0.,0.,0.,0.), m_bar4(0.,0.,0.,0.);

	for(int i=0;i< d_i.size();i++){
		d_bar4 = d_bar4 + d_i[i];
		m_bar4 = m_bar4 + m_i[i];
	}
	Eigen::Vector3d d_bar = d_bar4.head<3>() / d_i.size();
	Eigen::Vector3d m_bar = m_bar4.head<3>() / m_i.size();

	//compute correlation matrix
	Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
	for(int i=0;i< d_i.size();i++){
		Eigen::Vector3d d_ci = d_i[i].head<3>() - d_bar;
		Eigen::Vector3d m_ci = m_i[i].head<3>() - m_bar;
		H = H + (m_ci * d_ci.transpose());
	}
	std::cout <<"H: "<<H<<std::endl;
	//compute SVD decomposition
	Eigen::JacobiSVD<Eigen::MatrixXd> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
	std::cout << "U: " << svd.matrixU() << std::endl;
	std::cout << "V: " << svd.matrixV() << std::endl;
	//rotation matrix R=VU'
	Eigen::Matrix3d Norm = Eigen::Matrix3d::Identity();
	Eigen::Matrix3d tmp = svd.matrixU() * svd.matrixV().transpose();
	Norm(2,2) = tmp.determinant();
	std::cout <<"Norm="<<Norm<<std::endl;
	Eigen::Matrix3d R = svd.matrixV() * Norm * svd.matrixU().transpose();
	//translation T = d_bar - R*m_bar
	Eigen::Vector3d t = d_bar - R*m_bar;
	std::cout <<"t="<<t <<" d_bar="<<d_bar<<" m_bar="<<m_bar<<std::endl;
	P = Eigen::Matrix4d::Identity();
	P.block<3,3>(0,0) = R;
	P(0,3) = t[0];
	P(1,3) = t[1];
	P(2,3) = t[2];

}

double StereoPoseEstimator::measureDist(std::vector<double>& model,std::vector<double>& detection){
	double dist = 0.;
	for(int i=0;i< model.size();i++){
		double d = model[i] - detection[i];
		d = d*d;
		dist += d;
	}
	return std::sqrt(dist);
}


void StereoPoseEstimator::matchLEDDetectionsToTargetModel(std::vector <std::vector<double>>& dists_vector_clique,std::vector<int>& clique ,  TargetLEDsModel& target, std::vector<int>& detections_labels){
	detections_labels.resize(dists_vector_clique.size());
	//iterate over the detections
	for(int i=0;i < dists_vector_clique.size();i++){
		double min_dist = std::numeric_limits<double>::max();
		int index = -1;
		//iterate over each LED in the model
		for(int j=0; j< target.distances_matrix_.rows; j++){

			double d = measureDist(dists_vector_clique[i],target.dists_vector_[j]);
			std::cout<<"dist(LED="<<j<<",Detection="<<clique[i]<<")="<<d<<std::endl;
			if(d < min_dist){
				min_dist = d;
				index = j;
			}
		}
		//contains, for filtered detection #i the #index of the corresponding model LED
		detections_labels[i] = index;
	}
	for(int i=0;i < detections_labels.size();i++){
		std::cout <<"detection #"<<clique[i]<<" matched to LED "<<detections_labels[i]<<std::endl;
	}
	//cv::waitKey(0);
}


/**
 * meant to be calle with the LEDs points of the target model
 *
 */
void StereoPoseEstimator::computeTargetModelHistograms(List4DPoints& points){

	target_.distances_matrix_ = cv::Mat::zeros(points.size(), points.size(), CV_64FC1);

	//iterate over each pair
	for(int i=0;i<points.size();i++){

		std::vector<double> histogram(points.size());
		Eigen::Vector4d& p_i = points(i);
		for(int j=i;j<points.size();j++){
			Eigen::Vector4d& p_j = points(j);
			double dx = p_i[0]-p_j[0];
			double dy = p_i[1]-p_j[1];
			double dz = p_i[2]-p_j[2];
			target_.distances_matrix_.at<double>(i,j) = std::sqrt(dx*dx + dy*dy + dz*dz);
			//histogram[j] = distances.at<double>(i,j);
			if(target_.distances_matrix_.at<double>(i,j)> 0.){
				target_.dist_vector_.push_back(target_.distances_matrix_.at<double>(i,j));
			}
		}
	}
	std::sort(target_.dist_vector_.begin(), target_.dist_vector_.end());
	std::cout << "dist_vector = ";
	for(auto elem : target_.dist_vector_)
		std::cout <<elem<<" ";
	std::cout <<std::endl;
	extractOrderedDistsVectorsFromMatrix(target_.distances_matrix_, target_.dists_vector_);


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
bool StereoPoseEstimator::estimateFromStereo(cv::Mat& ir, cv::Mat& ir2, double time_to_predict, List2DPoints& detected_led_positions,List2DPoints& detected_led_positions2, List4DPoints& detected_LEDs, Eigen::Matrix4d& P){

	List4DPoints detected_LEDs_3D;
	bool right = true;

	region_of_interest_ = cv::Rect(0, 0, ir.cols, ir.rows);

	// Do detection of LEDs in image
	LEDDetector::findLeds(ir, region_of_interest_, detection_threshold_value_, gaussian_sigma_, min_blob_area_,
						  max_blob_area_, max_width_height_distortion_, max_circular_distortion_,
						  detected_led_positions, distorted_detection_centers_, camera_matrix_K_,
						  camera_distortion_coeffs_, !right);

	std::cout << "-------- COMPARING DISTORTED AND UNDISTORTED PIXEL CENTRES"<<std::endl;
	for(int i=0;i< distorted_detection_centers_.size();i++){
		const cv::Point& p_dist = distorted_detection_centers_[i];
		const Eigen::Vector2d& p_undist = detected_led_positions[i];
		std::cout <<"distorted: "<<p_dist.x<<" "<<p_dist.y<<std::endl;
		std::cout <<"undistorted: "<<p_undist[0]<<" "<<p_undist[1]<<std::endl;

	}


	// Do detection of LEDs in image
	LEDDetector::findLeds(ir2, region_of_interest_, detection_threshold_value_, gaussian_sigma_, min_blob_area_,
						  max_blob_area_, max_width_height_distortion_, max_circular_distortion_,
						  detected_led_positions2, distorted_detection_centers_, right_ir_camera_matrix_K_,
						  right_ir_camera_distortion_coeffs_, right);




	//print the points
	for(int i=0; i< detected_led_positions.size(); i++){
			Eigen::Vector2d& p_left = detected_led_positions(i);
			std::cout <<"Left point #"<<i<<" = "<<p_left<<std::endl;
	}
	for(int i=0; i< detected_led_positions2.size(); i++){
			Eigen::Vector2d& p_right = detected_led_positions2(i);
			std::cout <<"Right point #"<<i<<" = "<<p_right<<std::endl;
	}
	std::vector<int> matches(detected_led_positions.size());

	//obtain the best match between LED detections in the stereo pair
	getBestStereoMatch(detected_led_positions, detected_led_positions2,matches);
	for(auto elem: matches)
		std::cout<< elem << " ";
	std::cout << std::endl;

	//compute the disparity
	findDisparities(detected_led_positions, detected_led_positions2,matches,detected_LEDs_3D);
	std::vector<std::vector<int> > graph;
	computeDetectionsGraph(detected_LEDs_3D, graph);

	//find the maximum clique of size the number of markers in the model
	int K = object_points_.size();
	std::vector<std::vector<int> > cliques;
	find_cliques(graph, K, cliques);
	debug_graph(cliques);
	int i=0;
	if(cliques.size()>=1){
		detected_LEDs.resize(object_points_.size());
		std::vector<int> & clique = cliques[0];
		std::cout <<"triangulated_LEDs.size()="<<detected_LEDs_3D.size()<<std::endl;
		for(auto elem : clique){
			std::cout <<"adding led detection #"<<elem<<std::endl;
			//construct the detections list in the right order
			detected_LEDs[i] = detected_LEDs_3D[elem];
			i++;
		}
		//at this point we have the detected 3D LEDs indexed by clique[0] in detected_LEDs
		cv::Mat dists_matrix_clique;
		std::vector<int> detections_labels;
		std::vector <std::vector<double>> dists_vector_clique;
		extractCliqueDistsMatrix(cliques[0], dists_matrix_clique);
		extractOrderedDistsVectorsFromMatrix(dists_matrix_clique, dists_vector_clique);
		//contains, for filtered detection #i the #index of the corresponding model LED
		matchLEDDetectionsToTargetModel(dists_vector_clique, cliques[0], target_,detections_labels);

		List4DPoints detected_LEDs_reordered;
		detected_LEDs_reordered.resize(detected_LEDs.size());
		//reorder the detections
		for(int i=0;i<detections_labels.size();i++){
			detected_LEDs_reordered[detections_labels[i]] = detected_LEDs[i];
		}

		//figure out the pose using Horn's method
		hornPoseEstimation(detected_LEDs_reordered, object_points_,P);

	}


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
		const double X = Z /fx * (p_left[0] -camera_matrix_K_.at<double>(0, 2));
		const double Y = Z /fy * (p_left[1] -camera_matrix_K_.at<double>(1, 2));
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

void StereoPoseEstimator::getBestStereoMatch( List2DPoints& detected_led_positions,List2DPoints& detected_led_positions2, std::vector<int>& matches){
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
  computeTargetModelHistograms(object_points_);
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

