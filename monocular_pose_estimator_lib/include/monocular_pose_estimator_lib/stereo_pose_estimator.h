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
 * PoseEstimator.h
 *
 *  Created on: Jul 29, 2013
 *      Author: Karl Schwabe
 */

/** \file pose_estimator.h
 * \brief File containing the headers and prototypes for the PoseEstimator class.
 *
 */

#ifndef STEREOPOSEESTIMATOR_H_
#define STEREOPOSEESTIMATOR_H_

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <math.h>
#include <vector>
#include "monocular_pose_estimator_lib/datatypes.h"
#include "monocular_pose_estimator_lib/led_detector.h"
#include "monocular_pose_estimator_lib/visualization.h"
#include "monocular_pose_estimator_lib/combinations.h"
#include "monocular_pose_estimator_lib/p3p.h"
#include <iostream>
#include <opencv2/opencv.hpp>

namespace monocular_pose_estimator
{



struct TargetLEDsModel {
	cv::Mat distances_matrix_; // !< contains the confusion matrix with the pairwise distances between the LED markers in the model
	std::vector< std::vector<double> > dists_vector_;
	std::vector<double> dist_vector_; // !< contains the distances between the LED markers in the model
};


/**
 * The PoseEstimator class for tracking objects with infrared LEDs and an infrared camera.
 *
 * This class produces instances of objects that can be tracked with infrared LEDs and an infrared camera.
 *
 */
class StereoPoseEstimator
{

private:
  Eigen::Matrix4d current_pose_; //!< Homogeneous transformation matrix for storing the current pose of the tracked object
  Eigen::Matrix4d previous_pose_; //!< Homogeneous transformation matrix for storing the previously estimated pose of the tracked object
  Eigen::Matrix4d predicted_pose_; //!< Homogeneous transformation matrix for storing the predicted pose of the object. \see predictPose
  Matrix6d pose_covariance_; //!< A 6x6 covariance matrix that stores the covariance of the calculated pose
  double current_time_; //!< Stores the time of the current pose
  double previous_time_; //!< Stores the time of the previous pose
  double predicted_time_; //!< Stores the time of the predicted pose
  List4DPoints object_points_; //!< Stores the positions of the makers/LEDs on the object being tracked in the object-fixed coordinate frame using homogeneous coordinates. It is a vector of 4D points.
  List4DPoints object_points_camera_frame_; //!< Stores the positions of the makers/LEDs on the object being tracked in the camera coordinate frame using homogeneous coordinates. It is a vector of 4D points.
  List2DPoints image_points_; //!< Stores the positions of the detected marker points found in the image. It is a vector of 2D points.
  List2DPoints predicted_pixel_positions_; //!< Stores the predicted pixel positions of the markers in the image. \see predictMarkerPositionsInImage
  List3DPoints image_vectors_; //!< Stores the unit vectors leading from the camera centre of projection out to the world points/marker points - these are used in the P3P algorithm. \see setImagePoints, calculateImageVectors
  VectorXuPairs correspondences_; //!< Stores the correspondences of the LED markers and the detected markers in the image
  double back_projection_pixel_tolerance_; //!< Stores the pixel tolerance that determines whether a back projection is valid or not. \see checkCorrespondences, initialise
  double nearest_neighbour_pixel_tolerance_; //!< Stores the pixel tolerance that determines the correspondences between the LEDs and the detections in the image when predicting the position of the LEDs in the image. \see findCorrespondences
  double certainty_threshold_; //!< Stores the ratio of how many of the back-projected points must be within the #back_projection_pixel_tolerance_ for a correspondence between the LEDs and the detections to be correct.
  double valid_correspondence_threshold_; //!< Stores the ratio of how many correspondences must be considered to be correct for the total correspondences to be considered correct. \see checkCorrespondences, initialise


  std::vector<cv::Point2f> distorted_detection_centers_;  //left IR camera
  std::vector<cv::Point2f> distorted_detection_centers2_; //right IR camera

  unsigned it_since_initialized_; //!< Counter to determine whether the system has been initialised already
  cv::Rect region_of_interest_; //!< OpenCV rectangle that defines the region of interest to be processd to find the LEDs in the image
  static const unsigned min_num_leds_detected_ = 4; //!< Minimum number of LEDs that need to be detected for a pose to be calculated
  bool pose_updated_;

  TargetLEDsModel target_;
  cv::Mat detections_distances_matrix_;



public:
  cv::Mat camera_matrix_K_; //!< Variable to store the camera matrix as an OpenCV matrix
  std::vector<double> camera_distortion_coeffs_; //!< Variable to store the camera distortion parameters

  cv::Mat rgb_camera_matrix_K_; //!< Variable to store the camera matrix as an OpenCV matrix
  std::vector<double> rgb_camera_distortion_coeffs_; //!< Variable to store the camera distortion parameters

  cv::Mat right_ir_camera_matrix_K_; //!< Variable to store the camera matrix as an OpenCV matrix
  std::vector<double> right_ir_camera_distortion_coeffs_; //!< Variable to store the camera distortion parameters

  const double B = 0.070; //!< Variable to store the Baseline of the stereo pair. Hardcoded for the R200


  int detection_threshold_value_; //!< The current threshold value for the image for LED detection
  double gaussian_sigma_; //!< The current standard deviation of the Gaussian that will be applied to the thresholded image for LED detection
  double min_blob_area_; //!< The the minimum blob area (in pixels squared) that will be detected as a blob/LED. Areas having an area smaller than this will not be detected as LEDs.
  double max_blob_area_; //!< The the maximum blob area (in pixels squared) that will be detected as a blob/LED. Areas having an area larger than this will not be detected as LEDs.
  double max_width_height_distortion_; //!< This is a parameter related to the circular distortion of the detected blobs. It is the maximum allowable distortion of a bounding box around the detected blob calculated as the ratio of the width to the height of the bounding rectangle. Ideally the ratio of the width to the height of the bounding rectangle should be 1.
  double max_circular_distortion_; //!< This is a parameter related to the circular distortion of the detected blobs. It is the maximum allowable distortion of a bounding box around the detected blob, calculated as the area of the blob divided by pi times half the height or half the width of the bounding rectangle.
  unsigned roi_border_thickness_; //!< This is the thickness of the boarder (in pixels) around the predicted area of the LEDs in the image that defines the region of interest for image processing and detection of the LEDs.


private:

  /**
   * Given the model points m_i, their corresponding detections d_i, it computes the pose P of the model
   * in the detection frame of reference
   *
   */
  void hornPoseEstimation(List4DPoints& d_i, List4DPoints& m_i, Eigen::Matrix4d& P);

  void matchLEDDetectionsToTargetModel(std::vector <std::vector<double>>& dists_vector_clique,std::vector<int>& clique, TargetLEDsModel& target, std::vector<int>& detections_labels);

  /**
   * computes the histogram distance (L2 Euclidean Norm)
   *
   */
  double measureDist(std::vector<double>& model,std::vector<double>& detection);
  /**
   * matches each detected LED with the Target Model
   * returns the association between the indices
   *
   */
  void matchLEDDetectionsToTargetModel(std::vector <std::vector<double>>& dists_vector_clique, TargetLEDsModel& target);
  /**
   * from the distances confusion matrix between LED detections, extracts those corresponding
   * to those LEDs in the parameter clique, and stores the result confusion matrix in dists_matrix_clique
   */
  void extractCliqueDistsMatrix(std::vector<int>& clique, cv::Mat& dists_matrix_clique);


  /**
   * from the detections distances confusion matrix, extracts a vector of vectors
   * so that the distances for each detection can be sorted, in order to be compared with the model
   *
   *
   */
  void extractOrderedDistsVectorsFromMatrix(cv::Mat& dists_matrix_clique, std::vector <std::vector<double>>& dists_vector_clique);

  /**
  * looks in the dist_vector_ for a distance within a 10% difference. If there is one the measured distance is valid
  *
  */
  bool isDistanceValid(double dist);
  void computeDetectionsGraph(List4DPoints& points, std::vector<std::vector<int> >& graph); //!< call this one with the detections to build a graph
  void computeTargetModelHistograms(List4DPoints& points); //!< call this one with the model markers
  /**
   * Generates all permutations of a vector of indices
   *
   *
   */
  void permute(std::vector<int>& array,int i,int length);

  void findDisparities(const List2DPoints& detected_led_positions,const List2DPoints& detected_led_positions2, std::vector<int>& matches, List4DPoints& detected_LEDs);

  void getBestStereoMatch( List2DPoints& detected_led_positions,List2DPoints& detected_led_positions2, std::vector<int>& matches);



public:
  /**
   * Constructor.
   *
   */
  StereoPoseEstimator();


  /**
   * Sets the positions of the markers on the object.
   *
   * \param positions_of_markers_on_object vector containing the position of the LEDs/markers on the object in the object-fixed coordinate frame. The coordinates are given in homogeneous coordinates.
   *
   * \see object_points
   *
   */
  void setMarkerPositions(List4DPoints positions_of_markers_on_object);

  /**
   * Returns the positions of the markers on the object in the object-fixed coordinate frame
   *
   * \return vector containing the position of the LEDs/markers on the object in the object-fixed coordinate frame. The coordinates are given in homogeneous coordinates.
   *
   * \see object_points
   *
   */
  List4DPoints getMarkerPositions();

  /**
    * Returns the positions of the markers on the object in the camera coordinate frame
    *
    * \return vector containing the position of the LEDs/markers on the object in the camera coordinate frame. The coordinates are given in homogeneous coordinates.
    *
    * \see object_points
    *
    */
  List4DPoints getMarkerCameraFramePositions();

  /**
   * Estimates the pose of the tracked object from a pair of stereo images
   */
  bool estimateFromStereo(cv::Mat& ir, cv::Mat& ir2, double time_to_predict, List2DPoints& detected_led_positions,List2DPoints& detected_led_positions2, List4DPoints& detected_LEDs, Eigen::Matrix4d& P);


  /**
   * Sets the predicted pose of the object.
   *
   * \param pose the homogeneous pose of the object with respect to the camera
   * \param time the time of the predicted pose
   *
   * \see predicted_pose predicted_time
   */
  void setPredictedPose(const Eigen::Matrix4d & pose, double time);

  /**
   * Returns the predicted pose of the object.
   *
   * \return the homogeneous pose of the object with respect to the camera
   *
   * \see predicted_pose
   *
   */
  Eigen::Matrix4d getPredictedPose();

  /**
     * Sets the back-projection pixel tolerance that is used to determine whether an LED and image detection correspondence is correct.
     *
     * \param tolerance the back-projection pixel tolerance
     *
     * \see back_projection_pixel_tolerance checkCorrespondences
     *
     */
    void setBackProjectionPixelTolerance(double tolerance);

    /**
     * Returns the back-projection pixel tolerance that is used to determine whether an LED and image detection correspondence is correct.
     *
     * \return tolerance the back-projection pixel tolerance
     *
     * \see back_projection_pixel_tolerance checkCorrespondences
     *
     */
    double getBackProjectionPixelTolerance();

    /**
     * Sets the nearest-neighbour pixel tolerance that is used to determine whether the predicted position of a marker corresponds to a detection in the image.
     *
     * \param tolerance the nearest-neighbour pixel tolerance
     *
     * \see nearest_neighbour_pixel_tolerance findCorrespondences
     *
     */
    void setNearestNeighbourPixelTolerance(double tolerance);

    /**
     * Returns the nearest-neighbour pixel tolerance that is used to determine whether the predicted position of a marker corresponds to a detection in the image.
     *
     * \return tolerance the nearest-neighbour pixel tolerance
     *
     * \see nearest_neighbour_pixel_tolerance findCorrespondences
     *
     */
    double getNearestNeighbourPixelTolerance();

    /**
     * Sets the ratio of how many of the back-projected points must be within the #back_projection_pixel_tolerance_ for a correspondence between the LEDs and the detections to be correct.
     *
     * \param threshold the certainty threshold
     *
     * \see certainty_threshold
     *
     */
    void setCertaintyThreshold(double threshold);

    /**
     * Returns the ratio of how many of the back-projected points must be within the #back_projection_pixel_tolerance_ for a correspondence between the LEDs and the detections to be correct.
     *
     * \return the certainty threshold
     *
     * \see certainty_threshold
     *
     */
    double getCertaintyThreshold();

    /**
     * Sets the ratio of how many correspondences must be considered to be correct for the total correspondences to be considered correct.
     *
     * \param threshold valid_correspondence_threshold
     *
     * \see valid_correspondence_threshold
     *
     */
    void setValidCorrespondenceThreshold(double threshold);

    /**
     * Returns the ratio of how many correspondences must be considered to be correct for the total correspondences to be considered correct.
     *
     * \returns valid_correspondence_threshold
     *
     * \see valid_correspondence_threshold
     *
     */
    double getValidCorrespondenceThreshold();


};

}

#endif /* POSEESTIMATOR_H_ */
