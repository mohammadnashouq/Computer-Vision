#include <stdio.h>
#include <iostream>
#include <fstream>
#include <ctime>
#include "PLYWriter.h"

#include "stereo_vision.h"

cv::Mat read_matrix_from_file(const std::string file_name, const int rows, const int cols)
{
	std::ifstream file_stream(file_name);

	cv::Mat K = cv::Mat::zeros(rows, cols, CV_64F);

	for (int i = 0; i < rows; i++)
	{
		for (int j = 0; j < cols; j++)
		{
			double e;
			file_stream >> e;
			K.at<double>(i, j) = e;
		}
	}

	file_stream.close();

	return K;
}

const double threshold = 3.;
const int ransac_max_iteration = 5000;
const double ransac_confidence = 0.99;

int main(int argc, char** argv)
{
	// Setting the random seed to get random results in each run.
	srand(time(NULL));

	const std::string keys =
		"{ @image_src_name |  |  }"
		"{ @image_dest_name |  |  }"
		"{@matching | | }"
		"{ @calibration_matrix_name |  |  }";

	cv::CommandLineParser parser(argc, argv, keys);

	if (!parser.has("@image_src_name")) {
		std::cerr << "image_src_name is required" << std::endl;
		return 0;
	}
	if (!parser.has("@image_dest_name")) {
		std::cerr << "image_dest_name is required" << std::endl;
		return 0;
	}
	if (!parser.has("@matching")) {
		std::cerr << "matching is required" << std::endl;
		return 0;
	}
	if (!parser.has("@calibration_matrix_name")) {
		std::cerr << "calibration_matrix_name is required" << std::endl;
		return 0;
	}

	// Read arguments
	std::string image_src_name = parser.get<std::string>("@image_src_name");
	std::string image_dest_name = parser.get<std::string>("@image_dest_name");
	std::string matching_name = parser.get<std::string>("@matching");
	std::string calibration_matrix_name = parser.get<std::string>("@calibration_matrix_name");

	// read images
	cv::Mat image_src = cv::imread(image_src_name);
	cv::Mat image_dest = cv::imread(image_dest_name);

	// Define features detector

	std::cout << matching_name << std::endl;

	std::vector<cv::Point2d> src_points, dest_points;
	stereo_vision::read_matching_file(matching_name, src_points, dest_points);

	std::cout << "Number of correspondences: " << src_points.size() << std::endl;

	// Show the correspondences
	cv::Mat draw_matches_output;
	stereo_vision::draw_correspondences(image_src, src_points, image_dest, dest_points, draw_matches_output);
	cv::imwrite("data/all_correspondences.jpg", draw_matches_output);

	// Normalize points
	std::cout << "Normalize points..." << std::endl;
	std::vector<cv::Point2d> normalized_src_points, normalized_dest_points;
	cv::Mat T_src, T_dest;
	stereo_vision::normalize_points(src_points, normalized_src_points, T_src);
	stereo_vision::normalize_points(dest_points, normalized_dest_points, T_dest);
	std::cout << "T_src: " << std::endl
		<< T_src << std::endl;
	std::cout << "T_dest: " << std::endl
		<< T_dest << std::endl;

	// Find fundamental matrix
	//std::cout << "Finding fundamental matrix..." << std::endl;
	cv::Mat F_normalized;
	std::vector<int> inliers;

	// Since we use the normalized coordinates, we have to normalize the ransac threshold
	// We scale the threshold with the average scales of the src and dest images
	// (another solution is to find the inliers according to original coordinates after denormalize the fundamental matrix)

	double normalized_threshold = 3 * ((T_src.at<double>(0, 0) + T_dest.at<double>(0, 0)) / 2.);
	std::cout << "ransac threshold : " << normalized_threshold << std::endl;

	stereo_vision::ransac_fundamental_matrix_8points_method(
		normalized_src_points,
		normalized_dest_points,
		F_normalized,
		inliers,
		ransac_max_iteration,      // max iterations
		normalized_threshold,      // ransac threshold
		ransac_confidence          // ransac confidence
	);

	// Local optimization
	stereo_vision::get_fundamental_matrix_LSQ(
		normalized_src_points,
		normalized_dest_points,
		inliers,
		F_normalized
	);

	std::cout << "Number of inliers: " << inliers.size() << std::endl;
	std::cout << "F_normalized: " << std::endl
		<< F_normalized << std::endl;
	cv::Mat F = T_dest.t() * F_normalized * T_src;
	std::cout << "F:" << std::endl
		<< F << std::endl;

	// Draw inliers correspondences
	stereo_vision::draw_correspondences(image_src, src_points, image_dest, dest_points, inliers, draw_matches_output);
	cv::imwrite("data/inliers_correspondences.jpg", draw_matches_output);

	// Calibration matrix
	cv::Mat K = read_matrix_from_file(calibration_matrix_name, 3, 3);
	// Essential matrix
	cv::Mat E = K.t() * F * K;
	//
	/*
	cv::Mat R1, R2, t;
	stereo_vision::decompose_essential_matrix(
		E,
		R1,
		R2,
		t
	);
	*/

	/*cv::Mat P1, P2, R, correct_t;
	stereo_vision::get_correct_rotation_translation(
		src_points,
		dest_points,
		inliers,
		K,
		R1,
		R2,
		t,
		R,
		correct_t,
		P1,
		P2
	);
	*/
	std::vector<cv::Point3f> word_3d;
	for (int i = 0; i < inliers.size(); i++)
	{
		float f = 3.8;
		float h = 60;
		float b = 120;
		float to_melimiter_covert =  166.67;
		cv::Point3d triongulated_point;
		stereo_vision::geometry_triangulation(
			src_points[inliers[i]],
			dest_points[inliers[i]],
			f,
			h,
			b,
			to_melimiter_covert,
			triongulated_point

		);
		
		if (triongulated_point.z < 100000) {
		word_3d.push_back(triongulated_point);
		}
	}
	
	WritePLY("res.xyz", word_3d);

	
	return 0;
}